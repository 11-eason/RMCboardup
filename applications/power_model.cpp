#include "power_model.hpp"

#include <cmath>
#include "control_constants.hpp"

namespace chassis_power
{
Prediction g_prediction = {};
LimitResult g_limit_result = {};

namespace
{
// 实测功率慢速修正系数（路径 B）。
// 目标：让"实测均值"贴住限幅点（小幅超/欠都可以），而不是保守地待在低功率。
// 因此必须先把实测功率低通（看均值）；直接用瞬时值会一遇峰值就降额，功率被压得很低。
float meter_bias = 1.0f;
float filtered_measured_w = 0.0f;
bool filter_primed = false;
uint32_t bias_stamp_ms = 0, bias_sequence = 0;

struct TrendGuard {
  bool primed = false;
  uint32_t stamp = 0, sequence = 0, control_stamp = 0;
  float previous_w = 0.0f, rise_w_s = 0.0f, forecast_w = 0.0f, scale = 1.0f;
};
TrendGuard trend;

void update_trend(float measured_w, bool valid, uint32_t stamp, uint32_t sequence,
                  uint32_t now, float ceiling_w)
{
  if (!valid) {
    // No derivative across missing feedback; retain derating until valid data returns.
    trend.primed = false;
    trend.rise_w_s = 0.0f;
    trend.control_stamp = now;
    return;
  }
  if (!trend.primed || sequence != trend.sequence) {
    if (!trend.primed || stamp - trend.stamp > control_constants::POWER_METER_TIMEOUT_MS) {
      trend.rise_w_s = 0.0f;
    }
    else if (stamp != trend.stamp) {
      const float dt = (stamp - trend.stamp) * 0.001f;
      const float rise = (measured_w - trend.previous_w) / dt;
      const float alpha = dt / (control_constants::POWER_TREND_FILTER_S + dt);
      trend.rise_w_s += alpha * (rise - trend.rise_w_s);
    }
    // Never extrapolate a falling trend below the actual reading.
    const float positive_rise = trend.rise_w_s > 0.0f ? trend.rise_w_s : 0.0f;
    trend.forecast_w = measured_w + control_constants::POWER_TREND_LEAD_S * positive_rise;
    if (ceiling_w > 0.0f && trend.forecast_w > ceiling_w) {
      const float ratio = ceiling_w / trend.forecast_w;
      // Bounded absolute cap: repeated rising frames must NOT compound derating.
      if (ratio < trend.scale) trend.scale = ratio;
    }
    trend.previous_w = measured_w;
    trend.stamp = stamp;
    trend.sequence = sequence;
    trend.primed = true;
  }
  const uint32_t elapsed_ms = now - trend.control_stamp;
  trend.control_stamp = now;
  const float dt = (elapsed_ms < 20U ? elapsed_ms : 20U) * 0.001f;
  // Recover toward the current cap, rather than waiting for the forecast to fall
  // far below the ceiling. Actual power must still have a little headroom.
  const float target_scale = ceiling_w > 0.0f && trend.forecast_w > ceiling_w ?
    ceiling_w / trend.forecast_w : 1.0f;
  if (trend.previous_w < ceiling_w - control_constants::POWER_TREND_RECOVER_BAND_W &&
      trend.scale < target_scale) {
    trend.scale += control_constants::POWER_TREND_RECOVER_PER_S * dt;
    if (trend.scale > target_scale) trend.scale = target_scale;
  }
  if (trend.scale > 1.0f) trend.scale = 1.0f;
  if (trend.scale < 0.0f) trend.scale = 0.0f;
}

float clamp_bias(float value)
{
  const float low = control_constants::POWER_BIAS_MIN;
  const float high = control_constants::POWER_BIAS_MAX;
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

// 峰值修正：实测功率超过允许值时，按超限量对限幅系数再打一折。
// 这是唯一不依赖模型准不准的环节：功率计说超了，就立刻少给力矩。
void apply_peak_correction(LimitResult & result, float measured_power_w)
{
  const float kp = control_constants::POWER_PEAK_KP;
  if (kp <= 0.0f || !result.meter_used || !std::isfinite(measured_power_w)) return;
  if (!std::isfinite(result.effective_limit_w) || result.effective_limit_w <= 0.0f) return;
  const float over = measured_power_w - result.effective_limit_w;
  if (over <= 0.0f) return;
  float k = 1.0f - kp * over / result.effective_limit_w;
  const float lowest = control_constants::POWER_PEAK_MIN;
  if (k < lowest) k = lowest;
  result.torque_scale *= k;
  if (result.torque_scale > 1.0f) result.torque_scale = 1.0f;
  result.peak_correction = k;
}

// 前瞻预留：期望功率上升越快，预留越多（限幅在 0 ~ POWER_RESERVE_MAX_W）。
// 用于抵消加速瞬间"指令已限住、物理功率还没跟上"造成的超调。
float demand_reserve(float demand_w)
{
  constexpr float dt = 0.001f;
  static float previous_demand_w = 0.0f;
  static bool primed = false;
  if (!primed) {
    previous_demand_w = demand_w;
    primed = true;
    return 0.0f;
  }
  const float rate = (demand_w - previous_demand_w) / dt;  // W/s
  previous_demand_w = demand_w;
  if (rate <= 0.0f) return 0.0f;  // 只在上升时预留
  float reserve = rate * control_constants::POWER_RESERVE_LEAD_S;
  const float max_reserve = control_constants::POWER_RESERVE_MAX_W;
  if (reserve > max_reserve) reserve = max_reserve;
  return reserve;
}

// Measurement correction runs only on a new CAN sample, using its actual interval.
void update_meter_bias(float measured_power_w, float limit_w,
                       uint32_t stamp_ms, uint32_t sequence)
{
  if (filter_primed && sequence == bias_sequence) return;
  const uint32_t elapsed_ms = stamp_ms - bias_stamp_ms;
  if (!filter_primed || elapsed_ms > control_constants::POWER_METER_TIMEOUT_MS) {
    filtered_measured_w = measured_power_w;
    filter_primed = true;
    bias_stamp_ms = stamp_ms;
    bias_sequence = sequence;
    return;  // Prime only; do not integrate across startup/dropout.
  }
  bias_stamp_ms = stamp_ms;
  bias_sequence = sequence;
  if (elapsed_ms == 0U) return;
  const float dt = elapsed_ms * 0.001f;
  const float nominal_steps = dt / 0.001f;
  const float tau = control_constants::POWER_BIAS_FILTER_S;
  if (!filter_primed) {
    filtered_measured_w = measured_power_w;
    filter_primed = true;
  }
  else {
    const float alpha = dt / (tau + dt);
    filtered_measured_w += alpha * (measured_power_w - filtered_measured_w);
  }
  // 用均值判断，而不是瞬时值
  const float error = filtered_measured_w - limit_w;
  // Once the actual reading is below target, do not keep reducing the budget
  // solely because the 0.3 s filter still remembers an earlier peak.
  if (error > 0.0f && measured_power_w > limit_w) {
    // 均值超限：降额（约 0.2 s 响应 10 W 误差）
    float change = control_constants::POWER_BIAS_DOWN_PER_W * error * nominal_steps;
    if (change > control_constants::POWER_BIAS_MAX_STEP)
      change = control_constants::POWER_BIAS_MAX_STEP;
    meter_bias = clamp_bias(meter_bias - change);
  }
  else if (error < -control_constants::POWER_BIAS_RECOVER_BAND_W &&
           measured_power_w < limit_w - control_constants::POWER_BIAS_RECOVER_BAND_W) {
    // 均值明显低于限幅：上调，把功率用满（也用于修正模型偏大）
    float change = control_constants::POWER_BIAS_UP_PER_W *
      (-error - control_constants::POWER_BIAS_RECOVER_BAND_W) * nominal_steps;
    if (change > control_constants::POWER_BIAS_MAX_STEP)
      change = control_constants::POWER_BIAS_MAX_STEP;
    meter_bias = clamp_bias(meter_bias + change);
  }
}
}  // namespace

Prediction predict(const MotorState (&motors)[MOTOR_COUNT], bool feedback_valid)
{
  Prediction result = {};
  result.calibrated = control_constants::POWER_MODEL_CALIBRATED;
  const float k1 = control_constants::POWER_K1;
  const float k2 = control_constants::POWER_K2;
  const float k3 = control_constants::POWER_K3;
  if (!std::isfinite(k1) || !std::isfinite(k2) || !std::isfinite(k3) ||
      k1 < 0.0f || k2 < 0.0f || k3 < 0.0f)
    return result;

  float torque_squared_sum = 0.0f;
  float speed_squared_sum = 0.0f;
  for (const auto & motor : motors) {
    const float torque = motor.commanded_torque_nm;
    const float speed = motor.measured_speed_rad_s;
    if (!std::isfinite(torque) || !std::isfinite(speed)) return result;
    result.mechanical_w += torque * speed;
    torque_squared_sum += torque * torque;
    speed_squared_sum += speed * speed;
  }
  result.torque_loss_w = k1 * torque_squared_sum;
  result.speed_loss_w = k2 * speed_squared_sum;
  result.torque_sq_sum = torque_squared_sum;  // 标定用：原始未乘系数的量
  result.speed_sq_sum = speed_squared_sum;
  result.idle_loss_w = k3;
  // Original model: signed mechanical power + torque/speed losses + idle loss.
  result.total_w = result.mechanical_w + result.torque_loss_w +
                   result.speed_loss_w + result.idle_loss_w;
  const bool finite = std::isfinite(result.mechanical_w) &&
                      std::isfinite(result.torque_loss_w) &&
                      std::isfinite(result.speed_loss_w) && std::isfinite(result.total_w);
  if (!finite) {
    result = {};
    result.calibrated = control_constants::POWER_MODEL_CALIBRATED;
    return result;
  }
  // Report nonnegative electrical consumption. Signed mechanical power is
  // retained for braking diagnostics; do not use abs(), which would turn
  // negative model values into artificial positive consumption.
  if (result.total_w < 0.0f) result.total_w = 0.0f;
  result.valid = feedback_valid;
  return result;
}
LimitResult limit(
  const MotorState (&motors)[MOTOR_COUNT], bool feedback_valid, float measured_power_w,
  bool power_measured_valid, uint32_t meter_stamp_ms, uint32_t meter_sequence, uint32_t now_ms)
{
  LimitResult result = {};
  result.limit_w = control_constants::CHASSIS_POWER_LIMIT_W -
                   control_constants::POWER_LIMIT_MARGIN_W;
  result.peak_correction = 1.0f;
  result.meter_used = power_measured_valid && std::isfinite(measured_power_w) && measured_power_w >= 0.0f;
  update_trend(measured_power_w, result.meter_used, meter_stamp_ms, meter_sequence,
               now_ms, result.limit_w);
  result.measured_rise_w_s = trend.rise_w_s;
  result.measured_forecast_w = trend.forecast_w;
  result.trend_scale = trend.scale;
  if (result.meter_used && std::isfinite(result.limit_w) && result.limit_w > 0.0f)
    update_meter_bias(measured_power_w, result.limit_w, meter_stamp_ms, meter_sequence);
  else
    filter_primed = false;  // Re-prime on reconnection; retain bounded bias.
  result.meter_bias = meter_bias;
  result.measured_filtered_w = filter_primed ? filtered_measured_w : 0.0f;
  result.before = predict(motors, feedback_valid);
  // 前瞻预留：期望功率快速上升（即将加速）时提前压低允许值，抑制加速瞬间的超调
  const float reserve = demand_reserve(result.before.total_w);
  result.effective_limit_w = result.limit_w * meter_bias - reserve;
  if (result.effective_limit_w < 0.0f) result.effective_limit_w = 0.0f;
  result.limited = true;
  MotorState scaled[MOTOR_COUNT];
  for (std::size_t i = 0; i < MOTOR_COUNT; ++i)
    scaled[i] = {0.0f, motors[i].measured_speed_rad_s};
  const auto finalize = [&]() {
    // Avoid weakening net braking solely because measured consumption was high.
    if (result.before.mechanical_w >= 0.0f) result.torque_scale *= trend.scale;
    if (result.torque_scale < 0.0f) result.torque_scale = 0.0f;
    if (result.torque_scale > 1.0f) result.torque_scale = 1.0f;
    for (std::size_t i = 0; i < MOTOR_COUNT; ++i)
      scaled[i].commanded_torque_nm = motors[i].commanded_torque_nm * result.torque_scale;
    // Report prediction for the FINAL command, including all extra corrections.
    result.after = predict(scaled, feedback_valid);
    result.limited = result.torque_scale < 1.0f;
    result.feasible = result.after.valid && result.after.total_w <= result.effective_limit_w;
    return result;
  };
  result.after = predict(scaled, feedback_valid);
  if (!result.before.valid || !std::isfinite(result.effective_limit_w) ||
      result.effective_limit_w < 0.0f)
    return finalize();

  if (result.before.total_w <= result.effective_limit_w) {
    result.torque_scale = 1.0f;
    result.after = result.before;
    result.limited = false;
    result.feasible = true;
    // 模型认为不用限，但实测可能已经超了（模型误差）→ 仍做峰值修正
    apply_peak_correction(result, measured_power_w);
    return finalize();
  }

  // Raw P(k) = a*k*k + b*k + c; reported power is max(0, P(k)).
  // Find the constrained minimum first, then search its increasing branch.
  const double a = result.before.torque_loss_w;
  const double b = result.before.mechanical_w;
  float low = 0.0f;
  if (a > 0.0 && b < 0.0) {
    const double minimum = -b / (2.0 * a);
    low = minimum < 1.0 ? static_cast<float>(minimum) : 1.0f;
  }
  const auto evaluate = [&](float scale) {
    for (std::size_t i = 0; i < MOTOR_COUNT; ++i)
      scaled[i].commanded_torque_nm = motors[i].commanded_torque_nm * scale;
    return predict(scaled, feedback_valid);
  };
  if (a == 0.0 && b < 0.0) low = 1.0f;
  result.torque_scale = low;
  result.after = evaluate(low);
  if (!result.after.valid || result.after.total_w > result.effective_limit_w) return finalize();

  result.feasible = true;
  float high = 1.0f;
  for (unsigned i = 0; i < control_constants::POWER_LIMIT_SEARCH_STEPS; ++i) {
    const float middle = low + (high - low) * 0.5f;
    const Prediction candidate = evaluate(middle);
    // Test the actual float model, retaining a known feasible lower endpoint.
    if (candidate.valid && candidate.total_w <= result.effective_limit_w) {
      low = middle;
      result.after = candidate;
    }
    else {
      high = middle;
    }
  }
  result.torque_scale = low;
  // 峰值修正：模型有误差，但"实测超过允许值"是确定的。
  // 按实测超限量立刻再打一折，直接压尖峰（不依赖模型准不准）。
  apply_peak_correction(result, measured_power_w);
  return finalize();
}
}  // namespace chassis_power
