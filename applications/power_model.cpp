#include "power_model.hpp"

#include <cmath>
#include "control_constants.hpp"

namespace chassis_power
{
Prediction g_prediction = {};
LimitResult g_limit_result = {};

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
  result.idle_loss_w = k3;
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
  result.valid = feedback_valid;
  return result;
}
LimitResult limit(const MotorState (&motors)[MOTOR_COUNT], bool feedback_valid)
{
  LimitResult result = {};
  result.limit_w = control_constants::CHASSIS_POWER_LIMIT_W -
                   control_constants::POWER_LIMIT_MARGIN_W;
  result.before = predict(motors, feedback_valid);
  result.limited = true;
  MotorState scaled[MOTOR_COUNT];
  for (std::size_t i = 0; i < MOTOR_COUNT; ++i)
    scaled[i] = {0.0f, motors[i].measured_speed_rad_s};
  result.after = predict(scaled, feedback_valid);
  if (!result.before.valid || !std::isfinite(result.limit_w) || result.limit_w < 0.0f)
    return result;

  if (result.before.total_w <= result.limit_w) {
    result.torque_scale = 1.0f;
    result.after = result.before;
    result.limited = false;
    result.feasible = true;
    return result;
  }

  // P(k) = a*k*k + b*k + c. a >= 0; braking can make b negative.
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
  if (!result.after.valid || result.after.total_w > result.limit_w) return result;

  result.feasible = true;
  float high = 1.0f;
  for (unsigned i = 0; i < control_constants::POWER_LIMIT_SEARCH_STEPS; ++i) {
    const float middle = low + (high - low) * 0.5f;
    const Prediction candidate = evaluate(middle);
    // Test the actual float model, retaining a known feasible lower endpoint.
    if (candidate.valid && candidate.total_w <= result.limit_w) {
      low = middle;
      result.after = candidate;
    }
    else {
      high = middle;
    }
  }
  result.torque_scale = low;
  return result;
}
}  // namespace chassis_power
