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

struct LimitResult
{
  Prediction before;
  Prediction after;
  float torque_scale;  // K in [0, 1]; calculated each cycle, not a tuning constant.
  float limit_w;
  bool limited;
  bool feasible;  // False on invalid input or no feasible uniform scale.
};

// Finds the largest feasible uniform torque scale. Does not alter the inputs.
// Invalid feedback/model: returns scale 0. If no scale can meet the limit,
// returns the scale minimizing model power and reports feasible=false.
LimitResult limit(const MotorState (&motors)[MOTOR_COUNT], bool feedback_valid);
extern LimitResult g_limit_result;

// Latest prediction BEFORE limiting, updated also for zero-command mode.
// Intended for debugger inspection; no synchronized cross-task snapshot is provided.
extern Prediction g_prediction;
}  // namespace chassis_power

#endif  // POWER_MODEL_HPP
