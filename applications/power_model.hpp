#ifndef POWER_MODEL_HPP
#define POWER_MODEL_HPP

#include <cstddef>
#include <cstdint>

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
  float torque_sq_sum;  // Στ²，供标定使用（原始量，未乘 K1）
  float speed_sq_sum;   // Σω²，供标定使用（原始量，未乘 K2）
  float total_w;  // Electrical consumption estimate, clamped to >= 0 W.
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
  float torque_scale;   // K in [0, 1]; calculated each cycle, not a tuning constant.
  float limit_w;              // Measured-power PI reference: hard limit - margin.
  float effective_limit_w;    // Model budget = reference + bounded PI trim.
  float pi_trim_w;
  float pi_integral_w;
  float measured_filtered_w;
  float measured_rise_w_s;
  float measured_forecast_w;
  float trend_scale;          // Single fast guard; independent of PI trim.
  uint32_t meter_interval_ms; // Most recent new-meter-frame interval (not plot period).
  bool fast_over_limit;       // Current fresh raw reading exceeds hard limit.
  bool limited;
  bool feasible;  // False on invalid input or no feasible uniform scale.
  bool meter_used;  // 本次是否用上了实测功率（功率计有效时）
};

// Finds the largest feasible uniform torque scale. Does not alter the inputs.
// Invalid feedback/model: returns scale 0. If no scale can meet the limit,
// returns the scale minimizing model power and reports feasible=false.
// Measured-power PI/fast guard use the frame timestamp and frame counter.
// Missing/stale meter feedback falls back to the uncorrected model budget.
LimitResult limit(
  const MotorState (&motors)[MOTOR_COUNT], bool feedback_valid, float measured_power_w = 0.0f,
  bool power_measured_valid = false, uint32_t meter_stamp_ms = 0,
  uint32_t meter_sequence = 0, uint32_t now_ms = 0);
extern LimitResult g_limit_result;

// Latest prediction BEFORE limiting, updated also for zero-command mode.
// Intended for debugger inspection; no synchronized cross-task snapshot is provided.
extern Prediction g_prediction;
}  // namespace chassis_power

#endif  // POWER_MODEL_HPP
