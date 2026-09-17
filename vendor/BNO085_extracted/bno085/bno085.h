/**
 * @file bno085.h
 * @author Flavian Melquiades (flavian.melquiades@gmail.com)
 * @brief Driver I2C para o BNO085, sobre a lib oficial de protocolo sh2
 *        (CEVA/Hillcrest). Ocupa o mesmo papel que components/mpu6050
 *        ocupava no projeto original.
 * @version 0.1
 * @date 2026-08-19
 *
 * @copyright Copyright (c) 2026
 *
 * Nota: o SH2 é uma lib de sessão única (estado global interno, sem handle
 * por instância) — por isso as funções abaixo não recebem um descriptor por
 * chamada como o mpu6050_*(dev, ...) fazia. O struct bno085_dev_t existe só
 * para deixar a assinatura de bno085_init() explícita sobre o que é
 * necessário configurar (porta/endereço I2C, pinos RESET/INT).
 *
 * Barramento: este driver NÃO inicializa I2C sozinho -- quem chama
 * bno085_init() precisa ter chamado components/i2c_config::initialize_i2c()
 * pra essa porta antes (ver README, seção "Modularidade"). Na Heltec
 * WiFi LoRa 32 V3 isso normalmente é I2C_NUM_1 com os pinos de
 * CONFIG_BNO085_SDA_GPIO/SCL_GPIO (Kconfig deste componente), já que
 * I2C_NUM_0/17/18 são o barramento interno do OLED.
 */
#ifndef __BNO085_H__
#define __BNO085_H__

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BNO085_I2C_ADDR_DEFAULT 0x4A /**< Pino ADR do sensor em GND */
#define BNO085_I2C_ADDR_ALT     0x4B /**< Pino ADR do sensor em 3V3 */

typedef struct {
    i2c_port_t i2c_port; /**< Porta já inicializada por i2c_config::initialize_i2c() */
    uint8_t i2c_addr;
    gpio_num_t reset_gpio; /**< GPIO_NUM_NC se não usar reset por hardware */
    gpio_num_t int_gpio;   /**< GPIO_NUM_NC se não usar pino INT */
} bno085_dev_t;

typedef struct {
    float angle_x; /**< Roll, graus */
    float angle_y; /**< Pitch, graus */
    float angle_z; /**< Yaw, graus */
    float accuracy_rad; /**< Estimativa de erro do rotation vector, radianos */
    bool valid;
} bno085_orientation_t;

/**
 * @brief Configura GPIOs de RESET/INT e abre a sessão SH2 sobre
 *        dev->i2c_port -- essa porta precisa já estar inicializada
 *        (ver nota de barramento acima).
 *
 * Não habilita nenhum relatório sozinho — chame
 * bno085_enable_rotation_vector() depois.
 */
esp_err_t bno085_init(const bno085_dev_t *dev);

/**
 * @brief Habilita o relatório de rotation vector (fusão com magnetômetro).
 *
 * @param interval_us intervalo entre relatórios, em microssegundos.
 */
esp_err_t bno085_enable_rotation_vector(uint32_t interval_us);

/**
 * @brief Deve ser chamada periodicamente para processar dados pendentes do
 *        sensor e despachar callbacks internos do sh2. Sem chamar isso, o
 *        driver nunca atualiza a orientação lida.
 */
void bno085_service(void);

/**
 * @brief Copia a última orientação decodificada para *out.
 *
 * @return true se havia uma leitura válida, false se o sensor ainda não
 *         reportou nada.
 */
bool bno085_get_orientation(bno085_orientation_t *out);

#ifdef __cplusplus
}
#endif

#endif // __BNO085_H__
