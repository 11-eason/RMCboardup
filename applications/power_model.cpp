#include "power_model.hpp"

#include <cmath>
#include "control_constants.hpp"

namespace chassis_power
{
Prediction g_prediction = {};

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
}  // namespace chassis_power
