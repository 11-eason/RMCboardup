#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "shared_data.hpp"
#include "tools/pid/pid.hpp"

extern sp::CAN can2;
extern sp::RM_Motor motor_yaw;
extern sp::RM_Motor motor_pitch;
extern sp::RM_Motor motor_fold;

namespace
{
constexpr float WHEEL_RADIUS_M = 0.077f;
constexpr float HALF_W = 0.185f;
constexpr float HALF_L = 0.165f;
constexpr float MAX_LINEAR_SPEED = 0.3f;
constexpr float MAX_ANGULAR_SPEED = 0.3f;
constexpr float GEAR_RATIO = 14.9f;
constexpr float MAX_TORQUE = 0.5f;  // �����޷�����ȫ����

// �������ĳ���������ˣ�����Ը�����ļ���
constexpr float DIR_VX = 1.0f;     // ֱ�з���
constexpr float DIR_VY = 1.0f;     // ������
constexpr float DIR_OMEGA = 1.0f;  // ת�䷽��

// ����������ԣ����������ֱ�ߣ�������ӷ���
constexpr float DIR_M_FL = 1.0f;
constexpr float DIR_M_FR = 1.0f;
constexpr float DIR_M_RL = 1.0f;
constexpr float DIR_M_RR = 1.0f;

// �������˲�
constexpr float DEADZONE = 0.1f;
float apply_deadzone(float val)
{
  if (val > -DEADZONE && val < DEADZONE) return 0.0f;
  return val;
}

constexpr float ALPHA_FILTER = 0.1f;
float smooth_vx = 0.0f;
float smooth_vy = 0.0f;
float smooth_omega = 0.0f;

}  // namespace

sp::CAN can1(&hcan1);

sp::RM_Motor motor_fl(1, sp::RM_Motors::M3508, GEAR_RATIO);
sp::RM_Motor motor_fr(2, sp::RM_Motors::M3508, GEAR_RATIO);
sp::RM_Motor motor_rl(3, sp::RM_Motors::M3508, GEAR_RATIO);
sp::RM_Motor motor_rr(4, sp::RM_Motors::M3508, GEAR_RATIO);

// kp ���� 2.0 ����ֵ
sp::PID pid_fl(1e-3f, 2.0f, 0.0f, 0.0f, MAX_TORQUE, 0.0f, 1.0f, false, false);
sp::PID pid_fr(1e-3f, 2.0f, 0.0f, 0.0f, MAX_TORQUE, 0.0f, 1.0f, false, false);
sp::PID pid_rl(1e-3f, 2.0f, 0.0f, 0.0f, MAX_TORQUE, 0.0f, 1.0f, false, false);
sp::PID pid_rr(1e-3f, 2.0f, 0.0f, 0.0f, MAX_TORQUE, 0.0f, 1.0f, false, false);

namespace
{
void set_all_zero()
{
  motor_fl.cmd(0.0f);
  motor_fr.cmd(0.0f);
  motor_rl.cmd(0.0f);
  motor_rr.cmd(0.0f);

  smooth_vx = 0.0f;
  smooth_vy = 0.0f;
  smooth_omega = 0.0f;
}

void drive_with(float vx, float vy, float omega)
{
  float v_fl = (vx - vy - (HALF_W + HALF_L) * omega) / WHEEL_RADIUS_M;
  float v_fr = (vx + vy + (HALF_W + HALF_L) * omega) / WHEEL_RADIUS_M;
  float v_rl = (vx + vy - (HALF_W + HALF_L) * omega) / WHEEL_RADIUS_M;
  float v_rr = (vx - vy + (HALF_W + HALF_L) * omega) / WHEEL_RADIUS_M;

  pid_fl.calc(v_fl, motor_fl.speed * DIR_M_FL);
  pid_fr.calc(v_fr, motor_fr.speed * DIR_M_FR);
  pid_rl.calc(v_rl, motor_rl.speed * DIR_M_RL);
  pid_rr.calc(v_rr, motor_rr.speed * DIR_M_RR);

  motor_fl.cmd(pid_fl.out * DIR_M_FL);
  motor_fr.cmd(pid_fr.out * DIR_M_FR);
  motor_rl.cmd(pid_rl.out * DIR_M_RL);
  motor_rr.cmd(pid_rr.out * DIR_M_RR);
}

void chassis_update()
{
  if (g_data.sw_r == 0) {
    set_all_zero();
    return;
  }

  if (g_data.sw_r == 1) {
    float raw_vx = apply_deadzone(g_data.ch_lu) * MAX_LINEAR_SPEED * DIR_VX;
    float raw_vy = apply_deadzone(g_data.ch_lv) * MAX_LINEAR_SPEED * DIR_VY * 1.5f;
    float raw_omega = apply_deadzone(g_data.ch_lh) * MAX_ANGULAR_SPEED * DIR_OMEGA;

    // ƽ���˲�
    smooth_vx = (1.0f - ALPHA_FILTER) * smooth_vx + ALPHA_FILTER * raw_vx;
    smooth_vy = (1.0f - ALPHA_FILTER) * smooth_vy + ALPHA_FILTER * raw_vy;
    smooth_omega = (1.0f - ALPHA_FILTER) * smooth_omega + ALPHA_FILTER * raw_omega;

    drive_with(smooth_vx, smooth_vy, smooth_omega);
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
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv(CAN_RX_FIFO0);
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

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) > 0) {
    if (hcan == &hcan2) {
      can2.recv(CAN_RX_FIFO1);
      if (can2.rx_id == motor_yaw.rx_id)
        motor_yaw.read(can2.rx_data, stamp_ms);
      else if (can2.rx_id == motor_pitch.rx_id)
        motor_pitch.read(can2.rx_data, stamp_ms);
      else if (can2.rx_id == motor_fold.rx_id)
        motor_fold.read(can2.rx_data, stamp_ms);
    }
  }
}