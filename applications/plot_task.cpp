#include <limits>
#include "cmsis_os.h"
#include "control_constants.hpp"
#include "infrared_status.hpp"
#include "io/plotter/plotter.hpp"
#include "power_meter.hpp"
#include "power_model.hpp"
#include "trajectory_follow.hpp"
#include "trajectory_task.hpp"
#include "usart.h"

// USART1 / physical 4-pin UART2: AA BB + byte length + little-endian floats.
// Infrared test: distance cm, validity, bytes/20ms, valid frames/20ms.
// Other plot modes below are inactive while INFRARED_PLOT_TEST is true.

sp::Plotter plotter(&huart1, false);

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
        plotter.plot(status.valid ? status.distance_cm : missing,
                     status.valid ? 1.0f : 0.0f, bytes, frames,
                     static_cast<float>(status.raw_distance),
                     static_cast<float>(status.payload_byte2),
                     static_cast<float>(status.payload_byte3),
                     static_cast<float>(status.last_payload_length));
      }
      else {
        plotter.plot(status.valid ? status.distance_cm : missing,
                     status.valid ? 1.0f : 0.0f, bytes, frames);
      }
      previous_bytes = status.received_bytes;
      previous_frames = status.valid_frames;
    }
    else if (control_constants::FOLLOW_CONSTANT_TORQUE_TEST) {
      const auto state = trajectory_follow::read();
      // 恒力矩测试：只看实际转速和输出力矩，用来判断静摩擦与转向
      plotter.plot(state.yaw.actual_speed_rad_s, state.yaw.control_output);
    } else if (control_constants::FOLLOW_PLOT_MODE == 0U) {
      plotter.plot(power_meter::snapshot().power_w, chassis_power::g_limit_result.after.total_w);
    } else if (control_constants::FOLLOW_PLOT_MODE == 2U) {
      const TrajectoryPoint target = trajectory_read();
      plotter.plot(target.yaw, target.pitch);
    } else {
      const auto state = trajectory_follow::read();
      plotter.plot(
        state.yaw.target_angle_rad, state.yaw.actual_angle_rad, state.pitch.target_angle_rad,
        state.pitch.actual_angle_rad);
    }
  }
}
