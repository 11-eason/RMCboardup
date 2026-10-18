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
// USART1 @ 921600、每 20 ms 一帧。这里只画 4 条曲线：
//
//   1  yaw 目标角 (rad)      ← 轨迹给的目标
//   2  yaw 实际角 (rad)      ← 电机实际跟随的曲线
//   3  pitch 目标角 (rad)
//   4  pitch 实际角 (rad)
//
// 评价方式：把 1/2、3/4 各自叠在一起看重合度，差别就是跟踪误差。

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
      plotter.plot(
        state.yaw.target_angle_rad, state.yaw.actual_angle_rad, state.pitch.target_angle_rad,
        state.pitch.actual_angle_rad);
    }
  }
}
