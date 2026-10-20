#include <limits>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "control_constants.hpp"
#include "infrared_status.hpp"
#include "io/plotter/plotter.hpp"
#include "power_meter.hpp"
#include "power_model.hpp"
#include "task.h"
#include "trajectory_follow.hpp"
#include "trajectory_task.hpp"
#include "usart.h"

// USART1 / physical 4-pin UART2: AA BB + byte length + little-endian floats.
// Power mode (default): measured W, predicted W after torque limiting; 2 floats.
// Infrared mode, when explicitly enabled: distance and reception diagnostics.
// Other plot modes below are inactive while INFRARED_PLOT_TEST is true.

sp::Plotter plotter(&huart1, false);

// 底盘诊断快照（定义在 chassis_task.cpp）：0=FL 1=FR 2=RL 3=RR
extern "C" void chassis_diag(float * target, float * actual, float * out, float * released);

extern "C" void plot_task()
{
  uint32_t last_wake = osKernelSysTick();
  for (;;) {
    osDelayUntil(&last_wake, 20);
    if (control_constants::INFRARED_PLOT_TEST) {
      static uint32_t previous_bytes = 0, previous_frames = 0;
      const auto status = infrared::snapshot();
      const float missing = std::numeric_limits<float>::quiet_NaN();
      const float bytes = static_cast<float>(status.received_bytes - previous_bytes);
      const float frames = static_cast<float>(status.valid_frames - previous_frames);
      if (control_constants::INFRARED_PLOT_RAW_FIELDS) {
        // Append ONLY fields from the last checksum-correct frame. No smoothing.
        plotter.plot(
          status.distance_cm, status.valid ? 1.0f : 0.0f, bytes, frames,
          static_cast<float>(status.raw_distance), static_cast<float>(status.payload_byte2),
          static_cast<float>(status.payload_byte3), static_cast<float>(status.last_payload_length));
      }
      else {
        plotter.plot(
          status.valid ? status.distance_cm : missing, status.valid ? 1.0f : 0.0f, bytes, frames);
      }
      previous_bytes = status.received_bytes;
      previous_frames = status.valid_frames;
    }
    else if (control_constants::FOLLOW_CONSTANT_TORQUE_TEST) {
      const auto state = trajectory_follow::read();
      // 恒力矩测试（当前用于方向符号测试）：
      //   通道 1：实际角度（rad）—— 正力矩下应该增大；若减小说明 FOLLOW_YAW_DIRECTION 反了
      //   通道 2：实际转速（rad/s）—— 用来判断电机是否真的在转（不转=力矩不够或卡住）
      //   通道 3：输出力矩（N·m）
      plotter.plot(state.yaw.actual_angle_rad, state.yaw.actual_speed_rad_s,
                   state.yaw.control_output);
    }
    else if (control_constants::FOLLOW_PLOT_MODE == 0U) {
      const auto meter = power_meter::snapshot();
      // Chassis task updates the limit result at 1 kHz; copy before UART transmit.
      taskENTER_CRITICAL();
      const float demand_w = chassis_power::g_limit_result.before.total_w;  // 限幅前的期望
      const float scale = chassis_power::g_limit_result.torque_scale;       // 实际施加的限幅系数
      taskEXIT_CRITICAL();
      // 通道 1：实际功率（功率计实测，原始值未加工）
      // 通道 2：期望功率（限幅前的模型预测 = PID 原始指令想要的功率）
      // 通道 3：限幅系数 torque_scale（1.0 = 没限幅；越小 = 削得越狠）
      // 实测功率超限时，如果通道 3 还是 1.0 → 限幅器没参与；如果 <1 → 限幅器在工作，
      // 那超调就来自模型误差或物理响应延迟。
      plotter.plot(meter.power_w, demand_w, scale);
    }
    else if (control_constants::FOLLOW_PLOT_MODE == 5U) {
      const auto meter = power_meter::snapshot();
      taskENTER_CRITICAL();
      const auto power = chassis_power::g_limit_result;
      taskEXIT_CRITICAL();
      // Optional PI diagnostic: measured, target, K, PI trim, guard, CAN interval,
      // forecast and final-command prediction. Raw measured power is unchanged.
      plotter.plot(
        meter.power_w, power.limit_w, power.torque_scale, power.pi_trim_w, power.trend_scale,
        static_cast<float>(power.meter_interval_ms), power.measured_forecast_w,
        power.after.total_w);
    }
    else if (control_constants::FOLLOW_PLOT_MODE == 2U) {
      const TrajectoryPoint target = trajectory_read();
      plotter.plot(target.yaw, target.pitch);
    }
    else if (control_constants::FOLLOW_PLOT_MODE == 4U) {
      // 功率模型标定（4 通道），用于离线最小二乘拟合：
      //   通道 1：机械功率 Σ(τ·ω)（W）
      //   通道 2：Στ² × 100   （读数 / 100 才是 Στ²）
      //   通道 3：Σω² / 20    （读数 × 20 才是 Σω²）
      //   通道 4：功率计实测（W）
      // 拟合式：P_实测 = Στω + K1·Στ² + K2·Σω² + K3
      // 注意：4 条都做了 0.5 s 低通，因为瞬时值抖动很大，只有滤波后才有"平台"可读。
      const auto meter = power_meter::snapshot();
      taskENTER_CRITICAL();
      const float mech = chassis_power::g_limit_result.before.mechanical_w;
      const float tau2 = chassis_power::g_limit_result.before.torque_sq_sum;
      const float omega2 = chassis_power::g_limit_result.before.speed_sq_sum;
      taskEXIT_CRITICAL();
      static float filtered[4] = {0.0f, 0.0f, 0.0f, 0.0f};
      static bool primed = false;
      const float raw[4] = {mech, tau2 * 100.0f, omega2 / 20.0f, meter.power_w};
      constexpr float kAlpha = 0.02f / (0.5f + 0.02f);  // 20 ms 采样、0.5 s 时间常数
      for (int i = 0; i < 4; ++i) {
        if (!primed)
          filtered[i] = raw[i];
        else
          filtered[i] += kAlpha * (raw[i] - filtered[i]);
      }
      primed = true;
      plotter.plot(filtered[0], filtered[1], filtered[2], filtered[3]);
    }
    else if (control_constants::FOLLOW_PLOT_MODE == 3U) {
      // 底盘诊断（4 通道）：FL 目标速度、FL 实际速度、FL 输出力矩、静止释放标志
      // 判读：静止时 1 和 2 若在 ±0.5 rad/s 内来回、3 在 ±0.8 附近跳 → 就是零速刹车循环；
      //       通道 4 为 1 表示该轮已进入"放开"状态（输出 0，自由滑停）。
      float target[4];
      float actual[4];
      float out[4];
      float released[4];
      chassis_diag(target, actual, out, released);
      plotter.plot(target[0], actual[0], out[0], released[0]);
    }
    else if (control_constants::FOLLOW_PLOT_MODE == 1U) {
      const auto state = trajectory_follow::read();
      if (control_constants::FOLLOW_SINGLE_YAW) {
        if (control_constants::FOLLOW_YAW_DIAGNOSTICS) {
          static uint32_t last_yaw_frames = 0, last_tx_frames = 0;
          const auto bus = trajectory_follow::read_transport();
          // TX counts HAL mailbox submissions, not verified motor actuation.
          plotter.plot(state.yaw.target_angle_rad, state.yaw.actual_angle_rad,
                       state.yaw.control_output, state.active ? 1.0f : 0.0f,
                       state.feedback_fresh ? 1.0f : 0.0f, static_cast<float>(state.fault),
                       static_cast<float>(bus.yaw_frames - last_yaw_frames),
                       static_cast<float>(bus.tx_frames - last_tx_frames));
          last_yaw_frames = bus.yaw_frames;
          last_tx_frames = bus.tx_frames;
        }
        else {
          // Both channels use the same trajectory clock and captured zero.
          plotter.plot(state.yaw.target_angle_rad, state.yaw.actual_angle_rad);
        }
      }
      else {
        // 模式 1：按 FOLLOW_PLOT_AXIS 选择画哪根轴（0 = yaw，1 = pitch）
        if (control_constants::FOLLOW_YAW_DIAGNOSTICS) {
          // 双轴反馈诊断（4 通道）：
          //   1 = pitch 实际角   2 = pitch 目标角
          //   3 = pitch 反馈帧数（每 20 ms）  4 = yaw 反馈帧数（每 20 ms）
          // 判读：帧数 ~15-20 = 在线；帧数为 0 = 该电机没有反馈（ID/接线/开关错）
          static uint32_t last_yaw_frames = 0, last_pitch_frames = 0;
          const auto bus = trajectory_follow::read_transport();
          plotter.plot(state.pitch.actual_angle_rad, state.pitch.target_angle_rad,
                       static_cast<float>(bus.pitch_frames - last_pitch_frames),
                       static_cast<float>(bus.yaw_frames - last_yaw_frames));
          last_yaw_frames = bus.yaw_frames;
          last_pitch_frames = bus.pitch_frames;
        }
        else if (control_constants::FOLLOW_PLOT_AXIS == 1U) {
          plotter.plot(state.pitch.target_angle_rad, state.pitch.actual_angle_rad);
        }
        else {
          plotter.plot(state.yaw.target_angle_rad, state.yaw.actual_angle_rad);
        }
      }
    }
    else {
      const auto state = trajectory_follow::read();
      plotter.plot(
        state.yaw.target_angle_rad, state.yaw.actual_angle_rad, state.pitch.target_angle_rad,
        state.pitch.actual_angle_rad);
    }
  }
}
