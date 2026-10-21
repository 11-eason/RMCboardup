#ifndef GIMBAL_HOMING_HPP
#define GIMBAL_HOMING_HPP
#include <cstdint>
#include "can.h"
namespace gimbal_homing {
enum class Stage : uint8_t { Idle=0, WaitFeedback=1, SeekPitch=2, MoveM2006=3, Done=4, Fault=5 };
enum class Fault : uint8_t { None=0, Config=1, Feedback=2, Timing=3, SeekTimeout=4,
  SeekTravel=5, MoveTimeout=6, Overspeed=7, MoveOvershoot=8 };
struct Status {
  Stage stage;
  Fault fault;
  float pitch_angle_rad;
  float m2006_relative_rad;
  float m2006_target_relative_rad;
  float pitch_torque_nm;
  float m2006_torque_nm;
  float pitch_zero_rad;  // Final position when sequence completes; RAM only.
  float m2006_zero_rad;
  bool homed;
  bool feedback_fresh;
  uint32_t stamp_ms;
  uint32_t pitch_rx_frames, m2006_rx_frames, tx_errors;
};
bool selected();
// Task-context commands; no automatic retry after a fault.
void start();
void cancel();
Status snapshot();
bool receive_can1(const CAN_RxHeaderTypeDef &, uint8_t *, uint32_t now_ms);
void transmit_can1(uint32_t now_ms);
}
extern "C" void gimbal_homing_task(void const * argument);
#endif
