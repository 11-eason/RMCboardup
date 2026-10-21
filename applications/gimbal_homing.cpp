#include "gimbal_homing.hpp"
#include <cmath>
#include "FreeRTOS.h"
#include "task.h"
#include "control_constants.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "trajectory_follow.hpp"

namespace gimbal_homing {
namespace {
namespace c = control_constants;
static_assert(configTICK_RATE_HZ == 1000, "Homing requires 1ms ticks");
sp::RM_Motor pitch(c::HOME_PITCH_ID, sp::RM_Motors::GM6020, c::HOME_PITCH_RATIO);
sp::RM_Motor m2006(c::HOME_M2006_ID, sp::RM_Motors::M2006, c::HOME_M2006_RATIO);
Status latest = {};
uint32_t pitch_stamp=0, m_stamp=0, pitch_rx=0, m_rx=0, tx_errors=0;
float pitch_request=0.0f, m_request=0.0f;
uint32_t command_stamp=0, last_tx=0;
bool sent_once=false, start_requested=c::HOME_AUTO_START, stop_requested=false;
float clamp(float v,float lo,float hi) { return v<lo ? lo : (v>hi ? hi : v); }
struct PI {
  float integral=0.0f;
  float step(float target,float actual,float kp,float ki,float limit,float dt) {
    const float error=target-actual;
    const float inc=ki*error*dt;
    const float candidate=clamp(integral+inc,-limit,limit);
    const float raw=kp*error+candidate;
    if (!((raw>limit && inc>0.0f)||(raw< -limit && inc<0.0f))) integral=candidate;
    return clamp(kp*error+integral,-limit,limit);
  }
};
struct Feedback { float pa,ps,pt,ma,ms; bool fresh; };
struct Controller {
  Status out={};
  PI pitch_pi, m_pi;
  uint32_t entered=0, stall_since=0, settled_since=0;
  bool stall_active=false, settle_active=false;
  float pitch_start=0.0f, stall_angle=0.0f, m_origin=0.0f;
  float pitch_speed_cmd=0.0f, m_speed_cmd=0.0f;
  void fault(Fault f) {
    out.stage=Stage::Fault;out.fault=f;out.homed=false;
    out.pitch_torque_nm=out.m2006_torque_nm=0.0f;
    pitch_pi={};m_pi={};
  }
  void begin(uint32_t now) {
    *this=Controller{};out.stage=Stage::WaitFeedback;entered=now;
    const bool config=c::HOME_PITCH_SPEED_KP>0.0f && c::HOME_PITCH_SPEED_KI>=0.0f &&
      c::HOME_M2006_POSITION_KP>0.0f && c::HOME_M2006_SPEED_KP>0.0f &&
      c::HOME_M2006_SPEED_KI>=0.0f && c::HOME_TARGET_ACCEL_RAD_S2>0.0f &&
      c::HOME_PITCH_MAX_TORQUE_NM<=sp::GM6020_MAX_TORQUE &&
      c::HOME_M2006_MAX_TORQUE_NM>0.0f &&
      c::HOME_M2006_MAX_TORQUE_NM<=sp::M2006_P36_MAX_TORQUE &&
      c::HOME_PITCH_MAX_TRAVEL_RAD>0.0f && c::HOME_M2006_MOVE_RAD>0.0f &&
      c::HOME_PITCH_SEEK_SPEED_RAD_S>0.0f && c::HOME_M2006_MAX_SPEED_RAD_S>0.0f;
    if (!config) fault(Fault::Config);
  }
  void step(const Feedback & f,uint32_t now,float dt) {
    out.stamp_ms=now;out.feedback_fresh=f.fresh;
    out.pitch_angle_rad=f.pa;out.m2006_relative_rad=f.ma-m_origin;
    out.pitch_torque_nm=out.m2006_torque_nm=0.0f;
    if (out.stage==Stage::Idle || out.stage==Stage::Fault) return;
    if (out.stage==Stage::WaitFeedback) {
      if (!f.fresh) { if (now-entered>=c::HOME_READY_TIMEOUT_MS) fault(Fault::Feedback); return; }
      pitch_start=f.pa;m_origin=f.ma;entered=now;
      out.stage=Stage::SeekPitch;
    }
    if (!f.fresh) { fault(Fault::Feedback);return; }
    if (!std::isfinite(f.pa)||!std::isfinite(f.ps)||!std::isfinite(f.pt)||
        !std::isfinite(f.ma)||!std::isfinite(f.ms)) { fault(Fault::Feedback);return; }
    if (dt<=0.0f || dt>c::HOME_MAX_STEP_MS*0.001f) { fault(Fault::Timing);return; }
    if (std::fabs(f.ps)>c::HOME_OVERSPEED_RAD_S || std::fabs(f.ms)>c::HOME_OVERSPEED_RAD_S) {
      fault(Fault::Overspeed);return;
    }
    if (out.stage==Stage::SeekPitch) {
      if (now-entered>=c::HOME_PITCH_TIMEOUT_MS) {fault(Fault::SeekTimeout);return;}
      if (std::fabs(f.pa-pitch_start)>c::HOME_PITCH_MAX_TRAVEL_RAD) {fault(Fault::SeekTravel);return;}
      const float target=c::HOME_PITCH_DIRECTION*c::HOME_PITCH_SEEK_SPEED_RAD_S;
      const float dv=c::HOME_TARGET_ACCEL_RAD_S2*dt;
      pitch_speed_cmd+=clamp(target-pitch_speed_cmd,-dv,dv);
      out.pitch_torque_nm=pitch_pi.step(pitch_speed_cmd,f.ps,c::HOME_PITCH_SPEED_KP,
        c::HOME_PITCH_SPEED_KI,c::HOME_PITCH_MAX_TORQUE_NM,dt);
      const bool candidate=now-entered>=c::HOME_PITCH_STARTUP_BLANK_MS &&
        std::fabs(f.ps)<=c::HOME_PITCH_STALL_SPEED_RAD_S &&
        c::HOME_PITCH_DIRECTION*f.pt>=c::HOME_PITCH_STALL_TORQUE_NM &&
        c::HOME_PITCH_DIRECTION*out.pitch_torque_nm>=c::HOME_PITCH_STALL_TORQUE_NM;
      if (!candidate) stall_active=false;
      else {
        if (!stall_active || std::fabs(f.pa-stall_angle)>c::HOME_PITCH_STALL_TRAVEL_RAD) {
          stall_active=true;stall_since=now;stall_angle=f.pa;
        }
        if (now-stall_since>=c::HOME_PITCH_STALL_MS) {
          // Seat reached: release GM6020 BEFORE starting the other mechanism.
          out.pitch_torque_nm=0.0f;pitch_pi={};
          m_origin=f.ma;out.m2006_relative_rad=0.0f;
          out.m2006_target_relative_rad=c::HOME_M2006_DIRECTION*c::HOME_M2006_MOVE_RAD;
          out.stage=Stage::MoveM2006;entered=now;m_pi={};m_speed_cmd=0.0f;
        }
      }
      return; // M2006 remains zero for this entire phase.
    }
    if (out.stage==Stage::MoveM2006 || out.stage==Stage::Done) {
      const float delta=f.ma-m_origin;
      const float target=c::HOME_M2006_DIRECTION*c::HOME_M2006_MOVE_RAD;
      out.m2006_relative_rad=delta;out.m2006_target_relative_rad=target;
      if (out.stage==Stage::Done && !c::HOME_HOLD_AFTER_DONE) return;
      if (out.stage==Stage::MoveM2006 && now-entered>=c::HOME_M2006_TIMEOUT_MS) {
        fault(Fault::MoveTimeout);return;
      }
      if (std::fabs(delta)>c::HOME_M2006_MOVE_RAD+c::HOME_M2006_OVERSHOOT_RAD) {
        fault(Fault::MoveOvershoot);return;
      }
      const float error=target-delta;
      const float wanted=clamp(c::HOME_M2006_POSITION_KP*error,
        -c::HOME_M2006_MAX_SPEED_RAD_S,c::HOME_M2006_MAX_SPEED_RAD_S);
      const float dv=c::HOME_TARGET_ACCEL_RAD_S2*dt;
      m_speed_cmd+=clamp(wanted-m_speed_cmd,-dv,dv);
      out.m2006_torque_nm=m_pi.step(m_speed_cmd,f.ms,c::HOME_M2006_SPEED_KP,
        c::HOME_M2006_SPEED_KI,c::HOME_M2006_MAX_TORQUE_NM,dt);
      const bool settled=std::fabs(error)<=c::HOME_M2006_TOLERANCE_RAD &&
                         std::fabs(f.ms)<=c::HOME_M2006_SETTLE_SPEED_RAD_S;
      if (!settled) settle_active=false;
      else if (!settle_active) {settle_active=true;settled_since=now;}
      else if (out.stage==Stage::MoveM2006 && now-settled_since>=c::HOME_M2006_SETTLE_MS) {
        out.stage=Stage::Done;out.homed=true;
        out.pitch_zero_rad=f.pa;out.m2006_zero_rad=f.ma;
        if (!c::HOME_HOLD_AFTER_DONE) out.m2006_torque_nm=0.0f;
      }
    }
  }
};
bool submit(uint32_t id,uint8_t * data) {
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0) {++tx_errors;return false;}
  CAN_TxHeaderTypeDef h={};h.StdId=id;h.IDE=CAN_ID_STD;h.RTR=CAN_RTR_DATA;h.DLC=8;
  uint32_t mailbox=0;
  if (HAL_CAN_AddTxMessage(&hcan1,&h,data,&mailbox)!=HAL_OK) {++tx_errors;return false;}
  return true;
}
} // namespace
bool selected() {return c::HOME_MODE;}
void start() {taskENTER_CRITICAL();start_requested=true;stop_requested=false;taskEXIT_CRITICAL();}
void cancel() {taskENTER_CRITICAL();stop_requested=true;start_requested=false;
  pitch_request=m_request=0.0f;taskEXIT_CRITICAL();}
Status snapshot() {
  taskENTER_CRITICAL();Status s=latest;s.pitch_rx_frames=pitch_rx;s.m2006_rx_frames=m_rx;
  s.tx_errors=tx_errors;taskEXIT_CRITICAL();return s;
}
bool receive_can1(const CAN_RxHeaderTypeDef & h,uint8_t * data,uint32_t now) {
  if (!selected()||!data||h.IDE!=CAN_ID_STD||h.RTR!=CAN_RTR_DATA||h.DLC!=8) return false;
  if (h.StdId==pitch.rx_id) {pitch.read(data,now);pitch_stamp=now;++pitch_rx;return true;}
  if (h.StdId==m2006.rx_id) {m2006.read(data,now);m_stamp=now;++m_rx;return true;}
  return false;
}
void transmit_can1(uint32_t now) {
  if (!selected() || (sent_once && now-last_tx<c::HOME_CAN_INTERVAL_MS)) return;
  taskENTER_CRITICAL();
  const bool live=now-command_stamp<c::HOME_COMMAND_TIMEOUT_MS && !stop_requested;
  const float pt=live?pitch_request:0.0f, mt=live?m_request:0.0f;
  taskEXIT_CRITICAL();
  uint8_t pd[8]={},md[8]={}; // One owner for both groups; yaw and other slots stay zero.
  pitch.cmd(pt);pitch.write(pd);m2006.cmd(mt);m2006.write(md);
  // Independent submissions: a busy group is retried on the next interval.
  submit(pitch.tx_id,pd);submit(m2006.tx_id,md);
  last_tx=now;sent_once=true;
}
void run() {
  Controller ctrl;
  TickType_t wake=xTaskGetTickCount(),previous=wake;
  for (;;) {
    vTaskDelayUntil(&wake,pdMS_TO_TICKS(1));
    const uint32_t now=xTaskGetTickCount();
    const uint32_t elapsed=now-previous;
    if (!elapsed) {wake=now;continue;}
    previous=now;
    taskENTER_CRITICAL();
    const Feedback f={pitch.angle,pitch.speed,pitch.torque,m2006.angle,m2006.speed,
      pitch.is_open() && m2006.is_open() && now-pitch_stamp<c::HOME_FEEDBACK_TIMEOUT_MS &&
      now-m_stamp<c::HOME_FEEDBACK_TIMEOUT_MS};
    const bool go=start_requested, stop=stop_requested;
    start_requested=false;stop_requested=false;
    taskEXIT_CRITICAL();
    if (stop) ctrl=Controller{};
    else if (go) ctrl.begin(now);
    ctrl.step(f,now,elapsed*0.001f);
    taskENTER_CRITICAL();
    latest=ctrl.out;pitch_request=ctrl.out.pitch_torque_nm;m_request=ctrl.out.m2006_torque_nm;
    command_stamp=now;
    taskEXIT_CRITICAL();
    if (elapsed>c::HOME_MAX_STEP_MS) wake=now;
  }
}
} // namespace gimbal_homing
extern "C" void gimbal_homing_task(void const * argument) {
  if (!gimbal_homing::selected()) {trajectory_follow_task(argument);return;}
  gimbal_homing::run();
}
