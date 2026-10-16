#include "cmsis_os.h"
#include "shared_data.hpp"

namespace
{
constexpr uint16_t FOLD_DISTANCE_CM = 30;
constexpr uint16_t UNFOLD_DISTANCE_CM = 50;
constexpr uint32_t FOLD_TIMEOUT_MS = 3000;

enum class FoldState
{
  IDLE,
  FOLDING,
  FOLDED,
  UNFOLDING,
  UNFOLDED,
  TIMEOUT,
};

FoldState state = FoldState::IDLE;
uint32_t state_enter_ms = 0;
}  // namespace

extern "C" void fold_task()
{
  uint32_t last_wake = osKernelSysTick();

  for (;;) {
    osDelayUntil(&last_wake, 50);  // 20Hz

    uint32_t now = osKernelSysTick();

    // 最高优先级：失能逻辑
    if (g_data.sw_r == 0) {
      if (state == FoldState::FOLDING || state == FoldState::UNFOLDING) {
        state = FoldState::IDLE;
        state_enter_ms = now;
      }
      continue;
    }

    switch (state) {
      case FoldState::IDLE:
        if (g_data.infrared_valid && g_data.distance_cm < FOLD_DISTANCE_CM) {
          state = FoldState::FOLDING;
          state_enter_ms = now;
        }
        else if (g_data.infrared_valid && g_data.distance_cm > UNFOLD_DISTANCE_CM) {
          state = FoldState::UNFOLDING;
          state_enter_ms = now;
        }
        break;

      case FoldState::FOLDING:
        if ((now - state_enter_ms) > FOLD_TIMEOUT_MS) {
          state = FoldState::TIMEOUT;
          state_enter_ms = now;
          break;
        }
        // TODO: 云台电机折叠到位检测
        state = FoldState::FOLDED;
        state_enter_ms = now;
        break;

      case FoldState::FOLDED:
        if (g_data.infrared_valid && g_data.distance_cm > UNFOLD_DISTANCE_CM) {
          state = FoldState::UNFOLDING;
          state_enter_ms = now;
        }
        break;

      case FoldState::UNFOLDING:
        if ((now - state_enter_ms) > FOLD_TIMEOUT_MS) {
          state = FoldState::TIMEOUT;
          state_enter_ms = now;
          break;
        }
        // TODO: 云台电机展开到位检测
        state = FoldState::UNFOLDED;
        state_enter_ms = now;
        break;

      case FoldState::UNFOLDED:
        if (g_data.infrared_valid && g_data.distance_cm < FOLD_DISTANCE_CM) {
          state = FoldState::FOLDING;
          state_enter_ms = now;
        }
        break;

      case FoldState::TIMEOUT:
        if ((now - state_enter_ms) > 1000) {
          state = FoldState::IDLE;
          state_enter_ms = now;
        }
        break;
    }
  }
}