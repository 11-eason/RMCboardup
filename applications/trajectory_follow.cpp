#include "trajectory_follow.hpp"

#include <cmath>
#include "control_constants.hpp"
#include "FreeRTOS.h"
#include "task.h"

namespace trajectory_follow
{
namespace
{
using control_constants::FollowAxisConfig;
using control_constants::FollowPidConfig;
static_assert(configTICK_RATE_HZ == 1000, "Trajectory follow requires 1 ms RTOS ticks");
static_assert(sizeof(TickType_t) == sizeof(uint32_t), "32-bit ticks required");
static_assert(control_constants::FOLLOW_PERIOD_MS > 0, "Invalid control period");

struct PidState
{
  float integral = 0.0f;
  float previous_feedback = 0.0f;
  bool initialized = false;
};
struct AxisState { PidState angle; PidState speed; };
Feedback feedback = {};
Result latest = {};
bool enabled = false;
AxisState yaw_state, pitch_state;

float clamp(float value, float low, float high)
{
  return value < low ? low : (value > high ? high : value);
}

bool gains_valid(const FollowPidConfig & pid)
{
  return std::isfinite(pid.kp) && pid.kp >= 0.0f &&
         std::isfinite(pid.ki) && pid.ki >= 0.0f &&
         std::isfinite(pid.kd) && pid.kd >= 0.0f &&
         std::isfinite(pid.integral_limit) && pid.integral_limit >= 0.0f;
}

bool configured(const FollowAxisConfig & c)
{
  return c.configured && gains_valid(c.angle_pid) && gains_valid(c.speed_pid) &&
         std::isfinite(c.angle_min_rad) && std::isfinite(c.angle_max_rad) &&
         c.angle_min_rad < c.angle_max_rad &&
         std::isfinite(c.max_speed_rad_s) && c.max_speed_rad_s > 0.0f &&
         std::isfinite(c.max_output) && c.max_output > 0.0f &&
         std::isfinite(c.velocity_feedforward_gain) && c.velocity_feedforward_gain >= 0.0f;
}

bool fresh(const AxisFeedback & f, uint32_t now)
{
  return f.valid && std::isfinite(f.angle_rad) && std::isfinite(f.speed_rad_s) &&
         (now - f.stamp_ms) < control_constants::FOLLOW_FEEDBACK_TIMEOUT_MS;
}

// Derivative on measurement avoids a kick from a target step.
// Conditional integration accounts for feedforward AND final output saturation.
float pid(PidState & state, const FollowPidConfig & config, float target,
          float actual, float dt, float feedforward, float low, float high,
          bool & saturated)
{
  const float error = target - actual;
  const float derivative = state.initialized ? -(actual - state.previous_feedback) / dt : 0.0f;
  state.previous_feedback = actual;
  state.initialized = true;
  const float increment = config.ki * error * dt;
  const float candidate_i = clamp(state.integral + increment,
                                  -config.integral_limit, config.integral_limit);
  const float base = feedforward + config.kp * error + config.kd * derivative;
  const float candidate = base + candidate_i;
  if (!((candidate > high && increment > 0.0f) || (candidate < low && increment < 0.0f)))
    state.integral = candidate_i;
  const float raw = base + state.integral;
  const float output = clamp(raw, low, high);
  saturated = raw < low || raw > high;
  return output;
}

AxisResult update_axis(AxisState & state, const FollowAxisConfig & c,
                       float target_angle, float target_speed,
                       const AxisFeedback & f, float dt)
{
  AxisResult result = {};
  result.target_angle_rad = target_angle;
  result.limited_angle_rad = clamp(target_angle, c.angle_min_rad, c.angle_max_rad);
  result.angle_limited = target_angle != result.limited_angle_rad;
  result.actual_angle_rad = f.angle_rad;
  result.actual_speed_rad_s = f.speed_rad_s;
  result.angle_error_rad = result.limited_angle_rad - f.angle_rad;
  // Remove the original trajectory's velocity when its position was clipped.
  float ff = result.angle_limited ? 0.0f : c.velocity_feedforward_gain * target_speed;
  if ((result.limited_angle_rad <= c.angle_min_rad && ff < 0.0f) ||
      (result.limited_angle_rad >= c.angle_max_rad && ff > 0.0f)) ff = 0.0f;
  result.speed_feedforward_rad_s = ff;
  float min_speed = -c.max_speed_rad_s, max_speed = c.max_speed_rad_s;
  // At the measured joint limit, forbid requesting further outward motion.
  if (f.angle_rad <= c.angle_min_rad) min_speed = 0.0f;
  if (f.angle_rad >= c.angle_max_rad) max_speed = 0.0f;
  result.speed_command_rad_s = pid(state.angle, c.angle_pid, result.limited_angle_rad,
    f.angle_rad, dt, ff, min_speed, max_speed, result.speed_limited);
  float min_output = -c.max_output, max_output = c.max_output;
  if (f.angle_rad <= c.angle_min_rad) min_output = 0.0f;
  if (f.angle_rad >= c.angle_max_rad) max_output = 0.0f;
  result.control_output = pid(state.speed, c.speed_pid, result.speed_command_rad_s,
    f.speed_rad_s, dt, 0.0f, min_output, max_output, result.output_limited);
  return result;
}

bool finite_output(const AxisResult & r)
{
  return std::isfinite(r.control_output) && std::isfinite(r.speed_command_rad_s) &&
         std::isfinite(r.angle_error_rad) && std::isfinite(r.speed_feedforward_rad_s);
}

void step(const TrajectoryPoint & target, const Feedback & f,
          bool run, uint32_t now, float dt)
{
  Result result = {};
  result.stamp_ms = now;
  result.configuration_ready = configured(control_constants::FOLLOW_YAW) &&
                               configured(control_constants::FOLLOW_PITCH);
  result.feedback_fresh = fresh(f.yaw, now) && fresh(f.pitch, now);
  const bool target_valid = std::isfinite(target.yaw) && std::isfinite(target.yaw_vel) &&
                            std::isfinite(target.pitch) && std::isfinite(target.pitch_vel);
  // Preserve references/feedback for plotting even while the controller is disabled.
  result.yaw.target_angle_rad = target.yaw;
  result.pitch.target_angle_rad = target.pitch;
  result.yaw.actual_angle_rad = f.yaw.angle_rad;
  result.pitch.actual_angle_rad = f.pitch.angle_rad;
  result.yaw.actual_speed_rad_s = f.yaw.speed_rad_s;
  result.pitch.actual_speed_rad_s = f.pitch.speed_rad_s;
  if (run && result.configuration_ready && result.feedback_fresh && target_valid &&
      std::isfinite(dt) && dt > 0.0f &&
      dt <= control_constants::FOLLOW_MAX_STEP_MS * 0.001f) {
    result.yaw = update_axis(yaw_state, control_constants::FOLLOW_YAW,
                             target.yaw, target.yaw_vel, f.yaw, dt);
    result.pitch = update_axis(pitch_state, control_constants::FOLLOW_PITCH,
                               target.pitch, target.pitch_vel, f.pitch, dt);
    result.active = finite_output(result.yaw) && finite_output(result.pitch);
  }
  if (!result.active) {
    yaw_state = {};
    pitch_state = {};
    result.yaw.control_output = 0.0f;
    result.pitch.control_output = 0.0f;
    result.yaw.speed_command_rad_s = 0.0f;
    result.pitch.speed_command_rad_s = 0.0f;
  }
  taskENTER_CRITICAL();
  latest = result;
  taskEXIT_CRITICAL();
}
}  // namespace

void set_feedback(const Feedback & value)
{
  taskENTER_CRITICAL();
  feedback = value;
  taskEXIT_CRITICAL();
}

Result read()
{
  taskENTER_CRITICAL();
  const Result value = latest;
  taskEXIT_CRITICAL();
  return value;
}

void set_enabled(bool value)
{
  taskENTER_CRITICAL();
  enabled = value;
  taskEXIT_CRITICAL();
}

void run()
{
  TickType_t wake = xTaskGetTickCount();
  TickType_t previous = wake;
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(control_constants::FOLLOW_PERIOD_MS));
    const TickType_t now = xTaskGetTickCount();
    const TickType_t delta = now - previous;
    previous = now;
    taskENTER_CRITICAL();
    const Feedback current = feedback;
    const bool run_requested = enabled;
    taskEXIT_CRITICAL();
    const TrajectoryPoint target = trajectory_read();
    step(target, current, run_requested, now, static_cast<float>(delta) * 0.001f);
    // After an excessive scheduling delay, skip catch-up iterations.
    if (delta > control_constants::FOLLOW_MAX_STEP_MS) wake = now;
  }
}
}  // namespace trajectory_follow

extern "C" void trajectory_follow_task(void const * argument)
{
  (void)argument;
  trajectory_follow::run();
}
