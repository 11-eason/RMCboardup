#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "power_meter.hpp"
#include "power_model.hpp"
#include "shared_data.hpp"
#include "trajectory_task.hpp"
#include "usart.h"

sp::Plotter plotter(&huart1, false);

extern "C" void plot_task()
{
  uint32_t last_wake = osKernelSysTick();

  for (;;) {
    osDelayUntil(&last_wake, 20);  // 50Hz

    plotter.plot(
      static_cast<float>(g_data.distance_cm), static_cast<float>(g_data.infrared_valid),
      static_cast<float>(g_data.sw_r), g_data.ch_lv, g_data.ch_lh);

    // plotter.plot(power_meter::snapshot().power_w, chassis_power::g_limit_result.after.total_w);

    // const TrajectoryPoint point = trajectory_read();
    // plotter.plot(point.yaw, point.pitch);
  }
}