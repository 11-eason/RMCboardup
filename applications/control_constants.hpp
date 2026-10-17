#ifndef CONTROL_CONSTANTS_HPP
#define CONTROL_CONSTANTS_HPP

namespace control_constants
{
// Initial tuning values only; NOT calibrated for this chassis.
// Torque: gearbox output shaft N*m; speed: same shaft rad/s.
// Refit K1/K2 if the shaft convention or actual gear ratio changes.
constexpr float POWER_K1 = 1.93f;    // W / (N*m)^2
constexpr float POWER_K2 = 0.0088f;  // W / (rad/s)^2
constexpr float POWER_K3 = 4.0f;     // W, TOTAL chassis idle loss; measure before calibration.
constexpr bool POWER_MODEL_CALIBRATED = false;

constexpr float CHASSIS_POWER_LIMIT_W = 80.0f;
// Optional reserved margin; effective limit = limit - margin.
constexpr float POWER_LIMIT_MARGIN_W = 0.0f;
constexpr unsigned POWER_LIMIT_SEARCH_STEPS = 24;
static_assert(CHASSIS_POWER_LIMIT_W > 0.0f, "Power limit must be positive");
static_assert(
  POWER_LIMIT_MARGIN_W >= 0.0f && POWER_LIMIT_MARGIN_W < CHASSIS_POWER_LIMIT_W,
  "Invalid power margin");
static_assert(POWER_LIMIT_SEARCH_STEPS > 0, "Search needs at least one step");
// ===== Gimbal trajectory following =====
// PID gains and hardware-dependent limits are UNSET placeholders, not tuned values.
// Fill both axis configurations only after motor mapping, units, zero and direction
// are known. Set configured=true, then explicitly call set_enabled(true).
struct FollowPidConfig
{
  float kp;
  float ki;
  float kd;
  float integral_limit;  // Maximum absolute integral contribution, in PID output units.
};

struct FollowAxisConfig
{
  FollowPidConfig angle_pid;  // Output: rad/s; input: joint angle rad.
  FollowPidConfig speed_pid;  // Output: future motor-driver command units.
  float angle_min_rad;        // Mechanical joint limits; NOT trajectory range.
  float angle_max_rad;
  float max_speed_rad_s;
  float max_output;  // Symmetric bound in future motor-driver command units.
  float velocity_feedforward_gain;
  bool configured;
};

constexpr unsigned FOLLOW_PERIOD_MS = 1U;
constexpr unsigned FOLLOW_FEEDBACK_TIMEOUT_MS = 100U;  // Software initial setting.
constexpr unsigned FOLLOW_MAX_STEP_MS = 10U;           // Reset PID after a long scheduling gap.
// Order: {Kp, Ki, Kd, integral limit} for angle loop, then speed loop;
// angle min, angle max, speed limit, output limit, velocity feedforward gain, ready.
constexpr FollowAxisConfig FOLLOW_YAW = {
  {0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, false};
constexpr FollowAxisConfig FOLLOW_PITCH = {
  {0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, false};

}  // namespace control_constants

#endif  // CONTROL_CONSTANTS_HPP
