#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "shared_data.hpp"
#include "usart.h"

sp::DBus remote(&huart3);

extern "C" void rc_task()
{
  remote.request();

  for (;;) {
    osDelay(10);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart == &huart3) {
    auto stamp_ms = osKernelSysTick();
    remote.update(Size, stamp_ms);

    g_data.sw_r = static_cast<uint8_t>(remote.sw_r);
    g_data.sw_l = static_cast<uint8_t>(remote.sw_l);
    g_data.ch_lv = remote.ch_lv;
    g_data.ch_lh = remote.ch_lh;
    g_data.ch_lu = remote.ch_lu;

    remote.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
}