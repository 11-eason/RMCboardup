#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"
#include "stm32f4xx.h"

sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

namespace {
void beep(int times)
{
  for (int i = 0; i < times; i++) {
    buzzer.start();
    osDelay(100);
    buzzer.stop();
    osDelay(100);
  }
}
volatile bool g_homing_done_pending = false;
}  // namespace

// Called from the homing task when the whole sequence (6020 zero + M2006 final
// rotation) is finished. Only sets a flag; the beep is produced here so the
// homing loop never blocks.
extern "C" void buzzer_signal_homing_done()
{
  g_homing_done_pending = true;
}

extern "C" void buzzer_task()
{
  buzzer.set(4000, 0.5f);

  // 上电时先读复位标志（读 RCC->CSR 后再清），用来判断"运行中手转电机导致复位"的原因。
  const uint32_t csr = RCC->CSR;
  RCC->CSR |= RCC_CSR_RMVF;  // 清除标志，下次复位重新记录

  // 1) 固定的开机三声
  beep(3);

  // 2) 复位原因提示音（间隔 600 ms 后开始，方便和开机音区分）
  osDelay(600);
  if (csr & RCC_CSR_BORRSTF) {
    beep(1);  // 欠压复位：电源被拉低（电机反电势/电流冲击 24V 母线）
  }
  else if (csr & RCC_CSR_PINRSTF) {
    beep(2);  // NRST 引脚被拉低：干扰/ESD/接地回路
  }
  else if (csr & RCC_CSR_IWDGRSTF) {
    beep(3);  // 看门狗复位：某个任务卡死
  }
  else if (csr & RCC_CSR_SFTRSTF) {
    beep(4);  // 软件复位
  }
  else if (csr & RCC_CSR_PORRSTF) {
    beep(5);  // 上电复位：正常冷启动
  }
  // 都没有 → 不加提示音（和原来行为一致）

  for (;;) {
    if (g_homing_done_pending) {
      g_homing_done_pending = false;
      osDelay(500);          // 0.5 s after the M2006 arrives
      buzzer.start();
      osDelay(500);          // one 0.5 s beep
      buzzer.stop();
    }
    osDelay(20);
  }
}
