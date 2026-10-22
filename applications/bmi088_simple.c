#include "bmi088_simple.h"

#include "cmsis_os.h"
#include "spi.h"

#define BMI088_SPI_TIMEOUT_MS 5U

static void select_sensor(int sensor, GPIO_PinState state)
{
  if (sensor == 0) {
    HAL_GPIO_WritePin(CS1_ACC_GPIO_Port, CS1_ACC_Pin, state);
  } else {
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, state);
  }
}

static int read_registers(int sensor, uint8_t address, uint8_t data[], int count)
{
  uint8_t transmit_data[8] = {0};
  uint8_t receive_data[8] = {0};
  int offset;
  int index;
  HAL_StatusTypeDef status;

  if (count < 1 || count > 6) {
    return 0;
  }
  offset = (sensor == 0) ? 2 : 1;
  transmit_data[0] = address | 0x80;
  select_sensor(sensor, GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(&hspi1, transmit_data, receive_data, count + offset, BMI088_SPI_TIMEOUT_MS);
  select_sensor(sensor, GPIO_PIN_SET);
  if (status != HAL_OK) {
    return 0;
  }
  for (index = 0; index < count; index++) {
    data[index] = receive_data[index + offset];
  }
  return 1;
}

static int write_register(int sensor, uint8_t address, uint8_t value)
{
  uint8_t transmit_data[2];
  HAL_StatusTypeDef status;
  transmit_data[0] = address;
  transmit_data[1] = value;
  select_sensor(sensor, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(&hspi1, transmit_data, 2, BMI088_SPI_TIMEOUT_MS);
  select_sensor(sensor, GPIO_PIN_SET);
  osDelay(10);
  return status == HAL_OK;
}

static int configure_register(int sensor, uint8_t address, uint8_t value)
{
  uint8_t result;
  if (!write_register(sensor, address, value)) {
    return 0;
  }
  if (!read_registers(sensor, address, &result, 1)) {
    return 0;
  }
  return result == value;
}

int bmi088_init(void)
{
  uint8_t sensor_id;
  select_sensor(0, GPIO_PIN_SET);
  select_sensor(1, GPIO_PIN_SET);
  osDelay(50);

  if (!read_registers(0, 0x00, &sensor_id, 1)) return 1;
  osDelay(2);
  if (!write_register(0, 0x7E, 0xB6)) return 1;
  if (!write_register(1, 0x14, 0xB6)) return 1;
  osDelay(50);

  if (!read_registers(0, 0x00, &sensor_id, 1)) return 1;
  osDelay(2);
  if (!read_registers(0, 0x00, &sensor_id, 1)) return 1;
  if (sensor_id != 0x1E) return 2;
  if (!read_registers(1, 0x00, &sensor_id, 1)) return 1;
  if (sensor_id != 0x0F) return 3;

  if (!configure_register(0, 0x7C, 0x00)) return 4;
  if (!configure_register(0, 0x7D, 0x04)) return 4;
  if (!configure_register(0, 0x40, 0xAC)) return 4;
  if (!configure_register(0, 0x41, 0x01)) return 4;
  if (!configure_register(1, 0x0F, 0x00)) return 5;
  if (!configure_register(1, 0x10, 0x82)) return 5;
  if (!configure_register(1, 0x11, 0x00)) return 5;
  osDelay(50);
  return 0;
}

int bmi088_read(float acceleration_mps2[3], float angular_velocity_deg_s[3])
{
  uint8_t acc_data[6];
  uint8_t gyro_data[6];
  uint8_t sensor_id;
  int16_t raw;
  int index;

  if (!read_registers(0, 0x00, &sensor_id, 1) || sensor_id != 0x1E) return 0;
  if (!read_registers(1, 0x00, &sensor_id, 1) || sensor_id != 0x0F) return 0;
  if (!read_registers(0, 0x12, acc_data, 6)) return 0;
  if (!read_registers(1, 0x02, gyro_data, 6)) return 0;

  for (index = 0; index < 3; index++) {
    raw = (int16_t)((acc_data[2 * index + 1] << 8) | acc_data[2 * index]);
    acceleration_mps2[index] = raw * (6.0f * 9.80665f / 32768.0f);
    raw = (int16_t)((gyro_data[2 * index + 1] << 8) | gyro_data[2 * index]);
    angular_velocity_deg_s[index] = raw * (2000.0f / 32768.0f);
  }
  return 1;
}
