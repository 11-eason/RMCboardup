#include <limits>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "control_constants.hpp"
#include "grading.hpp"
#include "gimbal_imu.hpp"
#include "homing.hpp"
#include "pitch_start.hpp"
#include "infrared_status.hpp"
#include "shared_data.hpp"
#include "io/plotter/plotter.hpp"
#include "task.h"
#include "trajectory_follow.hpp"
#include "trajectory_task.hpp"
#include "usart.h"

// USART1 / physical 4-pin UART2: AA BB + byte length + little-endian floats.
// Yaw test: target angle (rad), actual IMU angle (rad), score (0..1); 3 floats.
// Infrared mode, when explicitly enabled: distance and reception diagnostics.
// Other plot modes below are inactive while INFRARED_PLOT_TEST is true.

sp::Plotter plotter(&huart1, false);

// 底盘诊断快照（定义在 chassis_task.cpp）：0=FL 1=FR 2=RL 3=RR

extern "C" void plot_task()
{
  uint32_t last_wake = osKernelSysTick();
  for (;;) {
    osDelayUntil(&last_wake, 20);
    if (homing::selected() || pitch_start::selected()) {
      // Homing/IR hardware test: one channel only, IR<=30 latch timer in ms.
      plotter.plot(static_cast<float>(g_data.infrared_timer_ms));
    }
    else if (control_constants::IMU_STARTUP_DIAGNOSTICS) {
      const auto score = grading::read();
      const auto imu = gimbal_imu::read();
      const auto bus = trajectory_follow::read_transport();
      static uint32_t previous_yaw_frames = 0;
      float progress = static_cast<float>(imu.calibration_elapsed_ms) /
                       control_constants::IMU_CALIBRATION_MS;
      const float sample_progress = static_cast<float>(imu.calibration_samples) /
                                    control_constants::IMU_CALIBRATION_MIN_SAMPLES;
      if (sample_progress < progress) progress = sample_progress;
      if (progress > 1.0f) progress = 1.0f;
      if (imu.ready) progress = 1.0f;
      // First three channels are unchanged. Remaining channels diagnose startup.
      plotter.plot(
        score.target_yaw_rad, score.actual_yaw_rad, score.yaw_ratio,
        static_cast<float>(imu.phase), static_cast<float>(imu.fault),
        static_cast<float>(imu.init_error), progress,
        static_cast<float>(bus.yaw_frames - previous_yaw_frames),
        static_cast<float>(score.follow_fault), imu.acceleration_norm_mps2,
        imu.max_raw_gyro_rad_s * 57.2957795f,
        static_cast<float>(imu.calibration_restart_reason));
      previous_yaw_frames = bus.yaw_frames;
    }
    else if (control_constants::INFRARED_PLOT_TEST) {
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
      plotter.plot(
        state.yaw.actual_angle_rad, state.yaw.actual_speed_rad_s, state.yaw.control_output);
    }

    else if (control_constants::FOLLOW_PLOT_MODE == 2U) {
      const TrajectoryPoint target = trajectory_read();
      plotter.plot(target.yaw, target.pitch);
    }

    else if (control_constants::FOLLOW_PLOT_MODE == 1U) {
      const auto state = trajectory_follow::read();
      if (control_constants::FOLLOW_SINGLE_YAW) {
        if (control_constants::FOLLOW_YAW_DIAGNOSTICS) {
          static uint32_t last_yaw_frames = 0, last_tx_frames = 0;
          const auto bus = trajectory_follow::read_transport();
          // TX counts HAL mailbox submissions, not verified motor actuation.
          plotter.plot(
            state.yaw.target_angle_rad, state.yaw.actual_angle_rad, state.yaw.control_output,
            state.active ? 1.0f : 0.0f, state.feedback_fresh ? 1.0f : 0.0f,
            static_cast<float>(state.fault), static_cast<float>(bus.yaw_frames - last_yaw_frames),
            static_cast<float>(bus.tx_frames - last_tx_frames));
          last_yaw_frames = bus.yaw_frames;
          last_tx_frames = bus.tx_frames;
        }
        else {
          // Same aligned IMU values as scoring; NaN while not ready/aligned.
          const auto score = grading::read();
          plotter.plot(
            score.target_yaw_rad, score.actual_yaw_rad, score.yaw_ratio);
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
          plotter.plot(
            state.pitch.actual_angle_rad, state.pitch.target_angle_rad,
            static_cast<float>(bus.pitch_frames - last_pitch_frames),
            static_cast<float>(bus.yaw_frames - last_yaw_frames));
          last_yaw_frames = bus.yaw_frames;
          last_pitch_frames = bus.pitch_frames;
        }
        else if (control_constants::FOLLOW_PLOT_AXIS == 1U) {
          const auto score = grading::read();
          plotter.plot(score.target_pitch_rad, score.actual_pitch_rad);
        }
        else {
          const auto score = grading::read();
          plotter.plot(score.target_yaw_rad, score.actual_yaw_rad);
        }
      }
    }
    else {
      const auto score = grading::read();
      plotter.plot(
        score.target_yaw_rad, score.actual_yaw_rad, score.target_pitch_rad,
        score.actual_pitch_rad);
    }
  }
}
