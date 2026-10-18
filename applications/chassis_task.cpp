#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "power_model.hpp"
#include "power_meter.hpp"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "shared_data.hpp"
#include "tools/pid/pid.hpp"
#include "trajectory_follow.hpp"
#include "control_constants.hpp"


namespace
{
constexpr float WHEEL_RADIUS_M = 0.077f;

// ������������ֹ����
constexpr float MAX_LINEAR_SPEED = 0.8f;
constexpr float MAX_ANGULAR_SPEED = 0.8f;
constexpr float GEAR_RATIO = 14.9f;
constexpr float MAX_TORQUE = 0.8f;
constexpr float DEADZONE = 0.05f;

float apply_deadzone(float val)
{
  if (val > -DEADZONE && val < DEADZONE) return 0.0f;
  return val;
}

constexpr float DIR_VX = 1.0f;
constexpr float DIR_VY = -1.0f;
constexpr float DIR_OMEGA = 1.0f;

constexpr float DIR_M_FL = 1.0f;   // 1
constexpr float DIR_M_FR = -1.0f;  // 2
constexpr float DIR_M_RL = -1.0f;  // 3
constexpr float DIR_M_RR = 1.0f;   // 4

}  // namespace

sp::CAN can1(&hcan1);

// �ϸ��Ӧ��ĵ�� ID
sp::RM_Motor motor_fl(1, sp::RM_Motors::M3508, GEAR_RATIO);
sp::RM_Motor motor_fr(2, sp::RM_Motors::M3508, GEAR_RATIO);
sp::RM_Motor motor_rl(3, sp::RM_Motors::M3508, GEAR_RATIO);
sp::RM_Motor motor_rr(4, sp::RM_Motors::M3508, GEAR_RATIO);

sp::PID pid_fl(1e-3f, 2.0f, 0.0f, 0.0f, MAX_TORQUE, 0.0f, 1.0f, false, false);
sp::PID pid_fr(1e-3f, 2.0f, 0.0f, 0.0f, MAX_TORQUE, 0.0f, 1.0f, false, false);
sp::PID pid_rl(1e-3f, 2.0f, 0.0f, 0.0f, MAX_TORQUE, 0.0f, 1.0f, false, false);
sp::PID pid_rr(1e-3f, 2.0f, 0.0f, 0.0f, MAX_TORQUE, 0.0f, 1.0f, false, false);

namespace
{
float update_power_prediction(float fl, float fr, float rl, float rr)
{
  const uint32_t now = osKernelSysTick();
  taskENTER_CRITICAL();
  const bool valid = motor_fl.is_alive(now) && motor_fr.is_alive(now) &&
                     motor_rl.is_alive(now) && motor_rr.is_alive(now);
  // Torque and speed use the same physical motor-output direction convention.
  const chassis_power::MotorState motors[chassis_power::MOTOR_COUNT] = {
    {fl, motor_fl.speed}, {fr, motor_fr.speed},
    {rl, motor_rl.speed}, {rr, motor_rr.speed}};
  taskEXIT_CRITICAL();
  chassis_power::g_limit_result = chassis_power::limit(motors, valid);
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
  float v_1 = (vx + vy - omega) / WHEEL_RADIUS_M;
  float v_2 = (vx - vy + omega) / WHEEL_RADIUS_M;
  float v_3 = (vx + vy + omega) / WHEEL_RADIUS_M;
  float v_4 = (vx - vy - omega) / WHEEL_RADIUS_M;

  pid_fl.calc(v_1, motor_fl.speed * DIR_M_FL);
  pid_fr.calc(v_2, motor_fr.speed * DIR_M_FR);
  pid_rl.calc(v_3, motor_rl.speed * DIR_M_RL);
  pid_rr.calc(v_4, motor_rr.speed * DIR_M_RR);

  const float fl = pid_fl.out * DIR_M_FL;
  const float fr = pid_fr.out * DIR_M_FR;
  const float rl = pid_rl.out * DIR_M_RL;
  const float rr = pid_rr.out * DIR_M_RR;
  const float power_scale = update_power_prediction(fl, fr, rl, rr);
  motor_fl.cmd(power_scale * fl);
  motor_fr.cmd(power_scale * fr);
  motor_rl.cmd(power_scale * rl);
  motor_rr.cmd(power_scale * rr);
}

void chassis_update()
{
  if (g_data.sw_r == 0) {
    set_all_zero();
    return;
  }

  if (g_data.sw_r == 1) {
    // �ϸ����Ҫ��ӳ�䣺
    // ǰ�� = ch_lv (vx)
    // ���� = ch_lh (vy)
    // ���� = ch_lu (omega)
    float target_vx = apply_deadzone(g_data.ch_lv) * MAX_LINEAR_SPEED * DIR_VX;
    float target_vy = apply_deadzone(g_data.ch_lh) * MAX_LINEAR_SPEED * DIR_VY;
    float target_omega = apply_deadzone(g_data.ch_lu) * MAX_ANGULAR_SPEED * DIR_OMEGA;

    drive_with(target_vx, target_vy, target_omega);
    return;
  }

  set_all_zero();
}
}  // namespace

extern "C" void chassis_task()
{
  can1.config();
  can1.start();

  uint32_t last_wake = osKernelSysTick();

  for (;;) {
    osDelayUntil(&last_wake, 1);
    chassis_update();

    motor_fl.write(can1.tx_data);
    motor_fr.write(can1.tx_data);
    motor_rl.write(can1.tx_data);
    motor_rr.write(can1.tx_data);
    can1.send(motor_fl.tx_id);
    trajectory_follow::transmit_can1(osKernelSysTick(), !control_constants::FOLLOW_USE_RC_STOP || g_data.sw_r != 0);
  }
}

// One dispatcher, one FIFO read per frame. No undefined gimbal objects.
namespace
{
void receive_can_fifo(CAN_HandleTypeDef * hcan, uint32_t fifo)
{
  if (hcan != &hcan1 && hcan != &hcan2) return;
  while (HAL_CAN_GetRxFifoFillLevel(hcan, fifo) > 0) {
    CAN_RxHeaderTypeDef header = {};
    uint8_t data[8] = {};
    if (HAL_CAN_GetRxMessage(hcan, fifo, &header, data) != HAL_OK) break;
    // CAN2 has no configured application devices yet; consume and ignore its frames.
    if (hcan != &hcan1) continue;
    can1.frame_type = (header.IDE == CAN_ID_EXT);
    can1.rx_id = can1.frame_type ? header.ExtId : header.StdId;
    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC != 8)
      continue;
    for (unsigned i = 0; i < 8; ++i) can1.rx_data[i] = data[i];
    const uint32_t stamp_ms = osKernelSysTick();
    if (header.StdId == power_meter::FEEDBACK_ID)
      power_meter::receive(header, data, stamp_ms);
    else if (header.StdId == motor_fl.rx_id)
      motor_fl.read(data, stamp_ms);
    else if (header.StdId == motor_fr.rx_id)
      motor_fr.read(data, stamp_ms);
    else if (header.StdId == motor_rl.rx_id)
      motor_rl.read(data, stamp_ms);
    else if (header.StdId == motor_rr.rx_id)
      motor_rr.read(data, stamp_ms);
    else
      trajectory_follow::receive_can1(header, data, stamp_ms);
  }
}
}  // namespace

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  receive_can_fifo(hcan, CAN_RX_FIFO0);
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  receive_can_fifo(hcan, CAN_RX_FIFO1);
}
