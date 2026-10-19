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
  float limit_w;        // 名义限幅 = CHASSIS_POWER_LIMIT_W - POWER_LIMIT_MARGIN_W
  float effective_limit_w;  // 叠加实测功率修正后的实际允许值（<= limit_w）
  float meter_bias;         // 修正系数，1.0 = 未修正；范围 [POWER_BIAS_MIN, POWER_BIAS_MAX]
  float peak_correction;    // 峰值修正系数（实测超限时额外打的折），1.0 = 未修正
  float measured_filtered_w;  // 低通后的实测功率（bias 的判据），供绘图观察
  float measured_rise_w_s;  // Filtered slope, updated ONLY on new meter frames.
  float measured_forecast_w;
  float trend_scale;  // Fast measured-power guard scale (independent of model coefficients).
  bool limited;
  bool feasible;  // False on invalid input or no feasible uniform scale.
  bool meter_used;  // 本次是否用上了实测功率（功率计有效时）
};

// Finds the largest feasible uniform torque scale. Does not alter the inputs.
// Invalid feedback/model: returns scale 0. If no scale can meet the limit,
// returns the scale minimizing model power and reports feasible=false.
// measured_power_w / power_measured_valid: 功率计实测值，用于慢速修正模型误差
// （路径 B）。不给这两个参数时行为与原来完全一致。
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
