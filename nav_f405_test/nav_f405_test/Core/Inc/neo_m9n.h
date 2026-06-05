#ifndef NEO_M9N_H
#define NEO_M9N_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
    uint8_t valid;
    uint8_t fix_quality;
    uint8_t satellites;
    int32_t latitude_e7;
    int32_t longitude_e7;
    int32_t altitude_cm;
    uint32_t last_update_ms;
    char utc_time[11];
} NEO_M9N_Fix_t;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    char sentence[96];
    uint8_t sentence_len;
    uint32_t bytes_rx;
    uint32_t sentences_rx;
} NEO_M9N_Handle_t;

void NEO_M9N_Init(NEO_M9N_Handle_t *dev,
                  SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port,
                  uint16_t cs_pin);

HAL_StatusTypeDef NEO_M9N_Poll(NEO_M9N_Handle_t *dev,
                               NEO_M9N_Fix_t *fix,
                               uint16_t max_bytes);

#endif /* NEO_M9N_H */
