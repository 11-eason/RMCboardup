#include "power_model.hpp"
#include <cmath>
#include "control_constants.hpp"

namespace chassis_power {
Prediction g_prediction = {};
LimitResult g_limit_result = {};
namespace {
float bounded(float x, float low, float high) { return x < low ? low : (x > high ? high : x); }
struct MeterControl {
  bool primed = false;
  uint32_t stamp = 0, sequence = 0, control_stamp = 0, interval_ms = 0;
  float previous_w = 0.0f, filtered_w = 0.0f, rise_w_s = 0.0f, forecast_w = 0.0f;
  float integral_w = 0.0f, trim_w = 0.0f, guard_scale = 1.0f;
};
MeterControl meter_control;

void update_meter(float measured_w, bool valid, uint32_t stamp, uint32_t sequence,
                  uint32_t now, float reference_w, const Prediction & demand)
{
  auto & m = meter_control;
  if (!valid) {
    m.primed = false;
    m.trim_w = m.integral_w = m.rise_w_s = 0.0f;
    m.interval_ms = 0;
    m.control_stamp = now;
    return; // No positive model-budget boost without fresh meter feedback.
  }
  const bool new_frame = !m.primed || sequence != m.sequence;
  if (new_frame) {
    const uint32_t interval = stamp - m.stamp;
    const bool continuous = m.primed && interval > 0U &&
                            interval <= control_constants::POWER_METER_TIMEOUT_MS;
    const float dt = continuous ? interval * 0.001f : 0.0f;
    m.interval_ms = continuous ? interval : 0U;
    if (!continuous) {
      m.filtered_w = measured_w;
      m.rise_w_s = 0.0f;
      m.integral_w = m.trim_w = 0.0f;
    }
    else {
      const float alpha = dt / (control_constants::POWER_PI_FILTER_S + dt);
      m.filtered_w += alpha * (measured_w - m.filtered_w);
      const float slope = (measured_w - m.previous_w) / dt;
      const float slope_alpha = dt / (control_constants::POWER_TREND_FILTER_S + dt);
      m.rise_w_s += slope_alpha * (slope - m.rise_w_s);
    }
    m.forecast_w = measured_w + control_constants::POWER_TREND_LEAD_S *
                                (m.rise_w_s > 0.0f ? m.rise_w_s : 0.0f);
    const float hard_w = control_constants::CHASSIS_POWER_LIMIT_W;
    // Fast guard bypasses PI filtering and executes for every new over-limit frame.
    float cap = 1.0f;
    if (m.forecast_w > hard_w) cap = hard_w / m.forecast_w;
    if (measured_w > hard_w) {
      // Further reduction is allowed only for a NEW, actually over-limit frame.
      const float raw_cap = m.guard_scale * bounded(1.0f - control_constants::POWER_FAST_DROP_KP *
        (measured_w - reference_w) / hard_w,
        0.0f, control_constants::POWER_FAST_OVER_MAX_SCALE);
      if (raw_cap < cap) cap = raw_cap;
    }
    if (cap < m.guard_scale) m.guard_scale = cap;

    // No PI action for released motors or net braking. PI cannot manufacture demand.
    const bool has_demand = demand.valid && demand.torque_sq_sum > 1e-6f &&
                            demand.mechanical_w >= 0.0f;
    if (!has_demand) {
      m.trim_w = m.integral_w = 0.0f;
    }
    else if (continuous) {
      float error = reference_w - m.filtered_w;
      const float band = control_constants::POWER_PI_DEADBAND_W;
      error = error > band ? error - band : (error < -band ? error + band : 0.0f);
      // Allow positive correction only when model-limited and guard is released.
      const bool can_raise = demand.total_w > reference_w + m.trim_w &&
                             m.guard_scale >= 0.999f && measured_w < hard_w;
      const float p = control_constants::POWER_PI_KP * error;
      float increment = control_constants::POWER_PI_KI * error * dt;
      if (increment > 0.0f && !can_raise) increment = 0.0f;
      const float lo = control_constants::POWER_PI_TRIM_MIN_W;
      const float hi = control_constants::POWER_PI_TRIM_MAX_W;
      const float candidate_i = bounded(m.integral_w + increment, lo, hi);
      const float candidate = p + candidate_i;
      // Conditional integration: no windup at either budget-correction limit.
      if (!((candidate > hi && increment > 0.0f) ||
            (candidate < lo && increment < 0.0f))) m.integral_w = candidate_i;
      float requested = bounded(p + m.integral_w, lo, hi);
      if (!can_raise && requested > m.trim_w) requested = m.trim_w;
      const float max_increase = control_constants::POWER_PI_RISE_W_PER_S * dt;
      if (requested > m.trim_w + max_increase) requested = m.trim_w + max_increase;
      m.trim_w = requested;
    }
    m.stamp = stamp; m.sequence = sequence; m.previous_w = measured_w; m.primed = true;
  }
  // Release only below target with forecast headroom. Smooth actuator recovery at 1 kHz.
  const uint32_t elapsed = now - m.control_stamp;
  m.control_stamp = now;
  const float dt = (elapsed < 20U ? elapsed : 20U) * 0.001f;
  if (measured_w < reference_w - control_constants::POWER_TREND_RECOVER_BAND_W &&
      m.forecast_w < control_constants::CHASSIS_POWER_LIMIT_W) {
    m.guard_scale = bounded(m.guard_scale +
      control_constants::POWER_TREND_RECOVER_PER_S * dt, 0.0f, 1.0f);
  }
}
} // namespace

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
LimitResult limit(const MotorState (&motors)[MOTOR_COUNT], bool feedback_valid,
                  float measured_power_w, bool power_measured_valid,
                  uint32_t meter_stamp_ms, uint32_t meter_sequence, uint32_t now_ms)
{
  LimitResult result = {};
  result.limit_w = control_constants::CHASSIS_POWER_LIMIT_W - control_constants::POWER_LIMIT_MARGIN_W;
  result.before = predict(motors, feedback_valid);
  result.meter_used = power_measured_valid && std::isfinite(measured_power_w) && measured_power_w >= 0.0f;
  update_meter(measured_power_w, result.meter_used, meter_stamp_ms, meter_sequence,
               now_ms, result.limit_w, result.before);
  const auto & m = meter_control;
  result.pi_trim_w = m.trim_w;
  result.pi_integral_w = m.integral_w;
  result.measured_filtered_w = m.filtered_w;
  result.measured_rise_w_s = m.rise_w_s;
  result.measured_forecast_w = m.forecast_w;
  result.trend_scale = m.guard_scale;
  result.meter_interval_ms = m.interval_ms;
  result.fast_over_limit = result.meter_used && measured_power_w > control_constants::CHASSIS_POWER_LIMIT_W;
  result.effective_limit_w = result.limit_w + m.trim_w;
  if (result.effective_limit_w < 0.0f) result.effective_limit_w = 0.0f;

  MotorState scaled[MOTOR_COUNT];
  for (std::size_t i = 0; i < MOTOR_COUNT; ++i) scaled[i] = {0.0f, motors[i].measured_speed_rad_s};
  const auto evaluate = [&](float k) {
    for (std::size_t i = 0; i < MOTOR_COUNT; ++i) scaled[i].commanded_torque_nm = motors[i].commanded_torque_nm * k;
    return predict(scaled, feedback_valid);
  };
  const auto finish = [&]() {
    // Guard must not weaken net braking. Normal consuming demand gets fast protection.
    if (result.before.mechanical_w >= 0.0f) result.torque_scale *= m.guard_scale;
    result.torque_scale = bounded(result.torque_scale, 0.0f, 1.0f);
    result.after = evaluate(result.torque_scale);
    result.limited = result.torque_scale < 1.0f;
    result.feasible = result.after.valid && result.after.total_w <= result.effective_limit_w;
    return result;
  };
  if (!result.before.valid || !std::isfinite(result.effective_limit_w)) return finish();
  if (result.before.total_w <= result.effective_limit_w) {
    result.torque_scale = 1.0f;
    return finish();
  }
  // Original model polynomial: a*k^2 + b*k + c, clipped to nonnegative watts.
  const double a = result.before.torque_loss_w, b = result.before.mechanical_w;
  float low = 0.0f;
  if (a > 0.0 && b < 0.0) low = bounded(static_cast<float>(-b / (2.0*a)), 0.0f, 1.0f);
  if (a == 0.0 && b < 0.0) low = 1.0f;
  result.torque_scale = low;
  const Prediction minimum = evaluate(low);
  if (!minimum.valid || minimum.total_w > result.effective_limit_w) return finish();
  float high = 1.0f;
  for (unsigned i = 0; i < control_constants::POWER_LIMIT_SEARCH_STEPS; ++i) {
    const float middle = low + (high-low)*0.5f;
    const Prediction candidate = evaluate(middle);
    if (candidate.valid && candidate.total_w <= result.effective_limit_w) low = middle;
    else high = middle;
  }
  result.torque_scale = low;
  return finish();
}
} // namespace chassis_power
