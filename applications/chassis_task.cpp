#include "cmsis_os.h"
#include "power_model.hpp"
#include "power_meter.hpp"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "shared_data.hpp"
#include "tools/pid/pid.hpp"

namespace
{
constexpr float WHEEL_RADIUS_M = 0.077f;
constexpr float HALF_W = 0.185f;
constexpr float HALF_L = 0.165f;
constexpr float MAX_LINEAR_SPEED = 2.0f;
constexpr float MAX_ANGULAR_SPEED = 1.0f;
constexpr float GEAR_RATIO = 14.9f;
constexpr float MAX_TORQUE = 3.0f;
}  // namespace

sp::CAN can1(&hcan1);  // Chassis motors.
sp::CAN power_meter_can2(&hcan2);  // Dedicated power-meter bus.

sp::RM_Motor motor_fl(1, sp::RM_Motors::M3508, GEAR_RATIO);
sp::RM_Motor motor_fr(2, sp::RM_Motors::M3508, GEAR_RATIO);
sp::RM_Motor motor_rl(3, sp::RM_Motors::M3508, GEAR_RATIO);
sp::RM_Motor motor_rr(4, sp::RM_Motors::M3508, GEAR_RATIO);

sp::PID pid_fl(1e-3f, 500.0f, 0.0f, 0.0f, MAX_TORQUE, 0.0f, 1.0f, false, false);
sp::PID pid_fr(1e-3f, 500.0f, 0.0f, 0.0f, MAX_TORQUE, 0.0f, 1.0f, false, false);
sp::PID pid_rl(1e-3f, 500.0f, 0.0f, 0.0f, MAX_TORQUE, 0.0f, 1.0f, false, false);
sp::PID pid_rr(1e-3f, 500.0f, 0.0f, 0.0f, MAX_TORQUE, 0.0f, 1.0f, false, false);

namespace
{
// Keep all model mathematics in power_model.cpp.
float update_power_prediction(float fl, float fr, float rl, float rr)
{
  const uint32_t now = osKernelSysTick();
  const bool feedback_valid = motor_fl.is_alive(now) && motor_fr.is_alive(now) &&
                              motor_rl.is_alive(now) && motor_rr.is_alive(now);
  const chassis_power::MotorState motors[chassis_power::MOTOR_COUNT] = {
    {fl, motor_fl.speed}, {fr, motor_fr.speed},
    {rl, motor_rl.speed}, {rr, motor_rr.speed}};
  chassis_power::g_limit_result = chassis_power::limit(motors, feedback_valid);
  chassis_power::g_prediction = chassis_power::g_limit_result.before;
  return chassis_power::g_limit_result.torque_scale;
}

void set_all_zero()
{
  update_power_prediction(0.0f, 0.0f, 0.0f, 0.0f);
  motor_fl.cmd(0.0f);
  motor_fr.cmd(0.0f);
  motor_rl.cmd(0.0f);
  motor_rr.cmd(0.0f);
}

void drive_with(float vx, float vy, float omega)
{
  float v_fl = vx - vy - (HALF_W + HALF_L) * omega;
  float v_fr = vx + vy + (HALF_W + HALF_L) * omega;
  float v_rl = vx + vy - (HALF_W + HALF_L) * omega;
  float v_rr = vx - vy + (HALF_W + HALF_L) * omega;

  float w_fl = v_fl / WHEEL_RADIUS_M;
  float w_fr = v_fr / WHEEL_RADIUS_M;
  float w_rl = v_rl / WHEEL_RADIUS_M;
  float w_rr = v_rr / WHEEL_RADIUS_M;

  pid_fl.calc(w_fl, motor_fl.speed);
  pid_fr.calc(w_fr, motor_fr.speed);
  pid_rl.calc(w_rl, motor_rl.speed);
  pid_rr.calc(w_rr, motor_rr.speed);

  const float power_scale =
    update_power_prediction(pid_fl.out, pid_fr.out, pid_rl.out, pid_rr.out);

  motor_fl.cmd(power_scale * pid_fl.out);
  motor_fr.cmd(power_scale * pid_fr.out);
  motor_rl.cmd(power_scale * pid_rl.out);
  motor_rr.cmd(power_scale * pid_rr.out);
}

void chassis_update()
{
  // ===== 优先级 1：失能 =====
  if (g_data.sw_r == 0) {
    set_all_zero();
    return;
  }

  // ===== 优先级 2：自主任务 =====
  if (g_data.auto_trigger) {
    drive_with(g_data.auto_vx, g_data.auto_vy, g_data.auto_omega);
    return;
  }

  // ===== 优先级 3：手动遥控 =====
  if (g_data.sw_r == 1) {
    float vx = g_data.ch_lv * MAX_LINEAR_SPEED;
    float vy = g_data.ch_lh * MAX_LINEAR_SPEED;
    float omega = g_data.ch_lu * MAX_ANGULAR_SPEED;
    drive_with(vx, vy, omega);
    return;
  }

  // 其他情况：失能
  set_all_zero();
}
}  // namespace

extern "C" void chassis_task()
{
  can1.config();
  can1.start();
  power_meter_can2.config();
  power_meter_can2.start();

  uint32_t last_wake = osKernelSysTick();

  for (;;) {
    osDelayUntil(&last_wake, 1);

    chassis_update();

    motor_fl.write(can1.tx_data);
    motor_fr.write(can1.tx_data);
    motor_rl.write(can1.tx_data);
    motor_rr.write(can1.tx_data);
    can1.send(motor_fl.tx_id);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan != &hcan1 && hcan != &hcan2) return;
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan2) {
      CAN_RxHeaderTypeDef header = {};
      uint8_t data[8] = {};
      if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        break;
      power_meter::receive(header, data, stamp_ms);
    }
    else if (hcan == &hcan1) {
      // CAN1 is reserved for chassis motor feedback.
      CAN_RxHeaderTypeDef header = {};
      if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, can1.rx_data) != HAL_OK)
        break;
      can1.frame_type = (header.IDE == CAN_ID_EXT);
      can1.rx_id = can1.frame_type ? header.ExtId : header.StdId;
      if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC != 8)
        continue;
      if (can1.rx_id == motor_fl.rx_id)
        motor_fl.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motor_fr.rx_id)
        motor_fr.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motor_rl.rx_id)
        motor_rl.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motor_rr.rx_id)
        motor_rr.read(can1.rx_data, stamp_ms);
    }
  }
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef * hcan) { (void)hcan; }