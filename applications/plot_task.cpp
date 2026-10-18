#include <limits>
#include "cmsis_os.h"
#include "control_constants.hpp"
#include "io/plotter/plotter.hpp"
#include "power_meter.hpp"
#include "power_model.hpp"
#include "trajectory_follow.hpp"
#include "trajectory_task.hpp"
#include "usart.h"

// 串口输出方式不变：sp::Plotter 二进制帧（0xAA 0xBB + 长度 + float 数组），
// USART1、每 20 ms 一帧。只画 7 条通道：
//
//   1  yaw 目标角 (rad)   ← 轨迹本身
//   2  yaw 实际角 (rad)   ← 电机自己的曲线，与 1 同量、同画法，直接叠起来比
//   3  yaw 角度误差 (rad) ← 就是 1 和 2 的差，用来看峰谷处差多少
//   4  pitch 目标角 (rad)
//   5  pitch 实际角 (rad)
//   6  pitch 角度误差 (rad)
//   7  状态：0 停止 / 1 正在跟随 / 10+故障码（11 跟踪误差、12 超速、13 反馈丢失、
//      14 时序超限、15 配置无效）
//
// 评价方式：把 1 和 2 叠在一起，两条曲线重合度就是跟踪效果；
// 3 用来量化"差多少度"。若曲线出现高频毛刺，说明内环在振荡（看力矩限幅/增益）。

sp::Plotter plotter(&huart1, false);

extern "C" void plot_task()
{
  uint32_t last_wake = osKernelSysTick();
  for (;;) {
    osDelayUntil(&last_wake, 20);
    if (control_constants::FOLLOW_CONSTANT_TORQUE_TEST) {
      const auto state = trajectory_follow::read();
      // 恒力矩测试：只看实际转速和输出力矩，用来判断静摩擦与转向
      plotter.plot(state.yaw.actual_speed_rad_s, state.yaw.control_output);
    } else if (control_constants::FOLLOW_PLOT_MODE == 0U) {
      plotter.plot(power_meter::snapshot().power_w, chassis_power::g_limit_result.after.total_w);
    } else if (control_constants::FOLLOW_PLOT_MODE == 2U) {
      const TrajectoryPoint target = trajectory_read();
      plotter.plot(target.yaw, target.pitch);
    } else {
      const auto state = trajectory_follow::read();
      const float status = state.active
                             ? 1.0f
                             : (state.fault == trajectory_follow::FAULT_NONE
                                  ? 0.0f
                                  : 10.0f + static_cast<float>(state.fault));
      plotter.plot(
        state.yaw.target_angle_rad, state.yaw.actual_angle_rad, state.yaw.angle_error_rad,
        state.pitch.target_angle_rad, state.pitch.actual_angle_rad, state.pitch.angle_error_rad,
        status);
    }
  }
}
