#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "shared_data.hpp"
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
  }
}