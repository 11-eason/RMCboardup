#ifndef POWER_MODEL_HPP
#define POWER_MODEL_HPP

#include <cstddef>

namespace chassis_power
{
constexpr std::size_t MOTOR_COUNT = 4;

struct MotorState
{
  float commanded_torque_nm;  // Post-PID, before any power limiting.
  float measured_speed_rad_s;  // Feedback, NOT target speed; same shaft as torque.
};

struct Prediction
{
  float mechanical_w;
  float torque_loss_w;
  float speed_loss_w;
  float idle_loss_w;
  float total_w;  // Signed model output; negative values are not clamped.
  bool valid;  // Finite inputs/parameters/results AND all feedback fresh.
  bool calibrated;  // Independent of valid: initial coefficients are uncalibrated.
};

// Does not modify motor commands or perform power limiting.
// Negative mechanical power represents braking; regeneration needs validation.
Prediction predict(const MotorState (&motors)[MOTOR_COUNT], bool feedback_valid);

// Latest prediction, updated by chassis_task (also for zero-command mode).
// Intended for debugger inspection; no synchronized cross-task snapshot is provided.
extern Prediction g_prediction;
}  // namespace chassis_power

#endif  // POWER_MODEL_HPP
