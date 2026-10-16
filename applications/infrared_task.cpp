#include "cmsis_os.h"
#include "shared_data.hpp"
#include "usart.h"

namespace
{
constexpr uint16_t INFRARED_MIN_CM = 3;
constexpr uint16_t INFRARED_MAX_CM = 200;
constexpr uint32_t INFRARED_TIMEOUT_MS = 100;
constexpr uint32_t INFRARED_RESTART_MS = 500;

uint8_t rx_byte;
uint8_t frame[9];
uint8_t state = 0;
uint8_t index = 0;
}  // namespace

extern "C" void infrared_task()
{
  g_data.infrared_valid = 0;
  g_data.infrared_last_rx_ms = osKernelSysTick();
  HAL_UART_Receive_IT(&huart6, &rx_byte, 1);

  uint32_t last_check_ms = osKernelSysTick();

  for (;;) {
    uint32_t now = osKernelSysTick();

    if ((now - g_data.infrared_last_rx_ms) > INFRARED_TIMEOUT_MS) {
      g_data.infrared_valid = 0;
    }

    if ((now - last_check_ms) > INFRARED_RESTART_MS) {
      last_check_ms = now;
      if ((now - g_data.infrared_last_rx_ms) > INFRARED_RESTART_MS) {
        HAL_UART_AbortReceive(&huart6);
        HAL_UART_Receive_IT(&huart6, &rx_byte, 1);
      }
    }

    osDelay(20);
  }
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart6) {
    g_data.infrared_last_rx_ms = osKernelSysTick();

    switch (state) {
      case 0:
        if (rx_byte == 0x5A) {
          frame[0] = rx_byte;
          state = 1;
        }
        break;
      case 1:
        if (rx_byte == 0x5A) {
          frame[1] = rx_byte;
          state = 2;
        }
        else
          state = 0;
        break;
      case 2:
        if (rx_byte == 0x15) {
          frame[2] = rx_byte;
          state = 3;
        }
        else
          state = 0;
        break;
      case 3:
        if (rx_byte == 0x03) {
          frame[3] = rx_byte;
          index = 4;
          state = 4;
        }
        else
          state = 0;
        break;
      default:
        frame[index++] = rx_byte;
        if (index >= 9) {
          uint8_t checksum = 0;
          for (int i = 0; i < 8; i++) checksum += frame[i];

          if (checksum == frame[8]) {
            uint16_t dist = (frame[4] << 8) | frame[5];
            if (dist >= INFRARED_MIN_CM && dist <= INFRARED_MAX_CM) {
              g_data.distance_cm = dist;
              g_data.infrared_valid = 1;
            }
            else {
              g_data.infrared_valid = 0;
            }
          }
          else {
            g_data.infrared_valid = 0;
          }
          state = 0;
        }
        break;
    }

    HAL_UART_Receive_IT(&huart6, &rx_byte, 1);
  }
}