#ifndef TRAJECTORY_FOLLOW_HPP
#define TRAJECTORY_FOLLOW_HPP

#include <cstdint>
#include "trajectory_task.hpp"

namespace trajectory_follow
{
// All angles/speeds are JOINT coordinates, after zero, sign and ratio conversion.
// Bounded, continuous joint angles: no automatic +/-pi wrapping is applied.
struct AxisFeedback
{
  float angle_rad;
  float speed_rad_s;
  uint32_t stamp_ms;
  bool valid;
};

struct Feedback
{
  AxisFeedback yaw;
  AxisFeedback pitch;
};

struct AxisResult
{
  float target_angle_rad;       // Original trajectory.
  float limited_angle_rad;
  float actual_angle_rad;
  float actual_speed_rad_s;
  float angle_error_rad;
  float speed_feedforward_rad_s;
  float speed_command_rad_s;    // Feedforward + angle PID, then limited.
  float control_output;         // Speed PID; driver units must be configured later.
  bool angle_limited;
  bool speed_limited;
  bool output_limited;
};

struct Result
{
  AxisResult yaw;
  AxisResult pitch;
  uint32_t stamp_ms;
  bool configuration_ready;
  bool feedback_fresh;
  bool active;
};

// Task-context APIs only. CAN/motor mapping is intentionally not implemented.
// Call set_feedback from a future motor-adapter task; do NOT fabricate feedback.
void set_feedback(const Feedback & feedback);
Result read();  // Consistent snapshot for plotting / a future motor-output adapter.
// Stop/run request; startup defaults to stopped. Gains/limits must also be configured.
void set_enabled(bool enabled);
}  // namespace trajectory_follow

extern "C" void trajectory_follow_task(void const * argument);

#endif  // TRAJECTORY_FOLLOW_HPP
