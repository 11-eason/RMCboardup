#ifndef CONTROL_CONSTANTS_HPP
#define CONTROL_CONSTANTS_HPP

namespace control_constants
{
// Initial tuning values only; NOT calibrated for this chassis.
// Torque: gearbox output shaft N*m; speed: same shaft rad/s.
// Refit K1/K2 if the shaft convention or actual gear ratio changes.
constexpr float POWER_K1 = 2.0f;   // W / (N*m)^2
constexpr float POWER_K2 = 0.01f;  // W / (rad/s)^2
constexpr float POWER_K3 = 0.0f;   // W, TOTAL chassis idle loss; measure before calibration.
constexpr bool POWER_MODEL_CALIBRATED = false;

constexpr float CHASSIS_POWER_LIMIT_W = 80.0f;
// Optional reserved margin; effective limit = limit - margin.
constexpr float POWER_LIMIT_MARGIN_W = 0.0f;
constexpr unsigned POWER_LIMIT_SEARCH_STEPS = 24;
static_assert(CHASSIS_POWER_LIMIT_W > 0.0f, "Power limit must be positive");
static_assert(POWER_LIMIT_MARGIN_W >= 0.0f &&
              POWER_LIMIT_MARGIN_W < CHASSIS_POWER_LIMIT_W, "Invalid power margin");
static_assert(POWER_LIMIT_SEARCH_STEPS > 0, "Search needs at least one step");
}  // namespace control_constants

#endif  // CONTROL_CONSTANTS_HPP
