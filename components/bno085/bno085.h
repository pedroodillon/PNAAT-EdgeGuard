/**
 * @file bno085.h
 * @brief Driver BNO085 usando SH2 + I2C moderno do ESP-IDF 6.x.
 *
 * P-101 / Edge Guard
 */

#ifndef __BNO085_H__
#define __BNO085_H__

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BNO085_I2C_ADDR_DEFAULT 0x4A
#define BNO085_I2C_ADDR_ALT 0x4B

/**
 * @brief Configuração física do BNO085.
 */
typedef struct {
  i2c_master_bus_handle_t i2c_bus;
  uint8_t i2c_addr;

  gpio_num_t reset_gpio;
  gpio_num_t int_gpio;

  uint32_t scl_speed_hz;
} bno085_dev_t;

/**
 * @brief Orientação obtida através do Rotation Vector.
 */
typedef struct {
  float angle_x;
  float angle_y;
  float angle_z;

  float accuracy_rad;

  bool valid;
} bno085_orientation_t;

/**
 * @brief Aceleração linear do BNO085.
 *
 * Unidade fornecida pelo SH2:
 * m/s²
 *
 * A componente gravitacional é removida pelo próprio
 * algoritmo de fusão do BNO085.
 */
typedef struct {
  float x;
  float y;
  float z;

  uint64_t timestamp_us;

  bool valid;
} bno085_linear_accel_t;

/**
 * @brief Inicializa o BNO085.
 */
esp_err_t bno085_init(const bno085_dev_t *dev);

/**
 * @brief Habilita Rotation Vector.
 *
 * @param interval_us intervalo desejado entre relatórios.
 */
esp_err_t bno085_enable_rotation_vector(uint32_t interval_us);

/**
 * @brief Habilita Linear Acceleration.
 *
 * @param interval_us intervalo desejado entre relatórios.
 */
esp_err_t bno085_enable_linear_acceleration(uint32_t interval_us);

/**
 * @brief Processa dados pendentes do SH2.
 *
 * Deve ser chamada continuamente.
 */
void bno085_service(void);

/**
 * @brief Obtém última orientação válida.
 */
bool bno085_get_orientation(bno085_orientation_t *out);

/**
 * @brief Obtém última aceleração linear válida.
 */
bool bno085_get_linear_acceleration(bno085_linear_accel_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __BNO085_H__ */