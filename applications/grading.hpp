#ifndef GRADING_HPP
#define GRADING_HPP

#include <cstdint>

// Angles are radians. The caller owns and initializes *fire_ratio to zero.
void yaw_detect(float gimbal_angle, float plan_angle, float* fire_ratio);
void pitch_detect(float gimbal_angle, float plan_angle, float* fire_ratio);

namespace grading {
struct Result {
  float yaw_ratio;    // 0..1 exponentially filtered hit rate, not percent.
  float pitch_ratio;
  float target_yaw_rad;
  float actual_yaw_rad;    // Fused IMU angle in trajectory coordinates, NOT encoder.
  float target_pitch_rad;
  float actual_pitch_rad;
  bool imu_ready;
  uint8_t follow_fault;
  uint32_t yaw_samples;
  uint32_t pitch_samples;
  uint32_t stamp_ms;
  bool yaw_active;
  bool pitch_active;
};

// Task-context snapshot for plotting/debugging. Pauses retain the last scores.
Result read();
}  // namespace grading

extern "C" void grading_task(void const* argument);

#endif  // GRADING_HPP
