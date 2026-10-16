#include "cmsis_os.h"
#include "shared_data.hpp"

namespace
{
// ===== 可调参数 =====
constexpr float FORWARD_SPEED_MPS = 0.5f;     // 前进速度
constexpr uint16_t TUNNEL_ENTER_CM = 40;      // 检测到隧道口的距离阈值
constexpr uint16_t TUNNEL_EXIT_CM = 60;       // 检测到出口的距离阈值
constexpr uint32_t DETECT_DEBOUNCE_MS = 500;  // 连续满足条件才确认
constexpr uint32_t FOLD_TIMEOUT_MS = 3000;    // 折叠超时
constexpr uint32_t UNFOLD_TIMEOUT_MS = 3000;  // 展开超时
constexpr uint32_t DRIVE_TIMEOUT_MS = 30000;  // 前进找隧道超时（暂定 30s）
constexpr uint32_t PASS_TIMEOUT_MS = 20000;   // 通过隧道超时（暂定 20s）

enum AutoState : uint8_t
{
  IDLE = 0,
  DRIVE_FORWARD = 1,
  FOLD_GIMBAL = 2,
  PASS_TUNNEL = 3,
  UNFOLD_GIMBAL = 4,
  FINISHED = 5,
  TIMEOUT = 6,
};

AutoState state = AutoState::IDLE;
uint32_t state_enter_ms = 0;
uint32_t condition_start_ms = 0;  // 条件满足的起始时间
}  // namespace

extern "C" void auto_task()
{
  uint32_t last_wake = osKernelSysTick();

  for (;;) {
    osDelayUntil(&last_wake, 20);  // 50Hz

    uint32_t now = osKernelSysTick();

    // ===== 最高优先级：失能 =====
    if (g_data.sw_r == 0) {
      state = AutoState::IDLE;
      g_data.auto_trigger = 0;
      g_data.auto_vx = 0.0f;
      g_data.auto_vy = 0.0f;
      g_data.auto_omega = 0.0f;
      g_data.auto_state = IDLE;
      condition_start_ms = now;
      continue;
    }

    // ===== 触发条件：上档 =====
    if (state == AutoState::IDLE && g_data.sw_r == 2) {
      state = AutoState::DRIVE_FORWARD;
      state_enter_ms = now;
      condition_start_ms = now;
      g_data.auto_trigger = 1;
    }

    // ===== 状态机 =====
    switch (state) {
      case AutoState::IDLE:
        g_data.auto_trigger = 0;
        g_data.auto_vx = 0.0f;
        g_data.auto_vy = 0.0f;
        g_data.auto_omega = 0.0f;
        break;

      case AutoState::DRIVE_FORWARD:
        // 直行前进
        g_data.auto_vx = FORWARD_SPEED_MPS;
        g_data.auto_vy = 0.0f;
        g_data.auto_omega = 0.0f;

        // 超时保护
        if ((now - state_enter_ms) > DRIVE_TIMEOUT_MS) {
          state = AutoState::TIMEOUT;
          state_enter_ms = now;
          break;
        }

        // 检测隧道口
        if (g_data.infrared_valid && g_data.distance_cm < TUNNEL_ENTER_CM) {
          if ((now - condition_start_ms) > DETECT_DEBOUNCE_MS) {
            state = AutoState::FOLD_GIMBAL;
            state_enter_ms = now;
          }
        }
        else {
          condition_start_ms = now;
        }
        break;

      case AutoState::FOLD_GIMBAL:
        // 停车
        g_data.auto_vx = 0.0f;
        g_data.auto_vy = 0.0f;
        g_data.auto_omega = 0.0f;

        // TODO: 触发云台折叠（等机械参数）
        // 目前暂时直接判定到位
        if ((now - state_enter_ms) > FOLD_TIMEOUT_MS) {
          state = AutoState::TIMEOUT;
          state_enter_ms = now;
          break;
        }

        // 折叠完成 → 进隧道
        state = AutoState::PASS_TUNNEL;
        state_enter_ms = now;
        condition_start_ms = now;
        break;

      case AutoState::PASS_TUNNEL:
        // 继续前进通过隧道
        g_data.auto_vx = FORWARD_SPEED_MPS;
        g_data.auto_vy = 0.0f;
        g_data.auto_omega = 0.0f;

        // 超时保护
        if ((now - state_enter_ms) > PASS_TIMEOUT_MS) {
          state = AutoState::TIMEOUT;
          state_enter_ms = now;
          break;
        }

        // 检测出口
        if (g_data.infrared_valid && g_data.distance_cm > TUNNEL_EXIT_CM) {
          if ((now - condition_start_ms) > DETECT_DEBOUNCE_MS) {
            state = AutoState::UNFOLD_GIMBAL;
            state_enter_ms = now;
          }
        }
        else {
          condition_start_ms = now;
        }
        break;

      case AutoState::UNFOLD_GIMBAL:
        // 停车
        g_data.auto_vx = 0.0f;
        g_data.auto_vy = 0.0f;
        g_data.auto_omega = 0.0f;

        // TODO: 触发云台展开
        if ((now - state_enter_ms) > UNFOLD_TIMEOUT_MS) {
          state = AutoState::TIMEOUT;
          state_enter_ms = now;
          break;
        }

        // 展开完成 → 结束
        state = AutoState::FINISHED;
        state_enter_ms = now;
        break;

      case AutoState::FINISHED:
        // 停车等待
        g_data.auto_vx = 0.0f;
        g_data.auto_vy = 0.0f;
        g_data.auto_omega = 0.0f;
        // 保持 auto_trigger = 1，等操作员回下档或中档
        break;

      case AutoState::TIMEOUT:
        // 停车
        g_data.auto_vx = 0.0f;
        g_data.auto_vy = 0.0f;
        g_data.auto_omega = 0.0f;
        // 1 秒后回 IDLE
        if ((now - state_enter_ms) > 1000) {
          state = AutoState::IDLE;
          state_enter_ms = now;
        }
        break;
    }

    // 输出状态供 plot_task 用
    g_data.auto_state = static_cast<uint8_t>(state);
  }
}