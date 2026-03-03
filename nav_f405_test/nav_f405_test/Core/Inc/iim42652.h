/**
 * @file    iim42652.h
 * @brief   IIM-42652 (TDK InvenSense) 6-axis IMU driver over SPI
 *
 * Register map is compatible with ICM-42652 / ICM-42688-P family.
 * SPI Mode 0 (CPOL=0, CPHA=0), max SPI clock 24 MHz.
 */
#ifndef IIM42652_H
#define IIM42652_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  Register Map — Bank 0 (default)
 * ═══════════════════════════════════════════════════════════════════════════*/
#define IIM42652_REG_DEVICE_CONFIG    0x11
#define IIM42652_REG_INT_CONFIG       0x14
#define IIM42652_REG_FIFO_CONFIG      0x16
#define IIM42652_REG_TEMP_DATA1       0x1D  /* Temperature high byte         */
#define IIM42652_REG_TEMP_DATA0       0x1E  /* Temperature low byte          */
#define IIM42652_REG_ACCEL_DATA_X1    0x1F  /* Accel X high                  */
#define IIM42652_REG_ACCEL_DATA_X0    0x20
#define IIM42652_REG_ACCEL_DATA_Y1    0x21
#define IIM42652_REG_ACCEL_DATA_Y0    0x22
#define IIM42652_REG_ACCEL_DATA_Z1    0x23
#define IIM42652_REG_ACCEL_DATA_Z0    0x24
#define IIM42652_REG_GYRO_DATA_X1     0x25  /* Gyro X high                   */
#define IIM42652_REG_GYRO_DATA_X0     0x26
#define IIM42652_REG_GYRO_DATA_Y1     0x27
#define IIM42652_REG_GYRO_DATA_Y0     0x28
#define IIM42652_REG_GYRO_DATA_Z1     0x29
#define IIM42652_REG_GYRO_DATA_Z0     0x2A
#define IIM42652_REG_INT_STATUS       0x2D
#define IIM42652_REG_INTF_CONFIG0     0x4C
#define IIM42652_REG_PWR_MGMT0       0x4E
#define IIM42652_REG_GYRO_CONFIG0    0x4F
#define IIM42652_REG_ACCEL_CONFIG0   0x50
#define IIM42652_REG_GYRO_CONFIG1    0x51
#define IIM42652_REG_ACCEL_CONFIG1   0x53
#define IIM42652_REG_WHO_AM_I        0x75
#define IIM42652_REG_BANK_SEL        0x76

/* ── Expected WHO_AM_I ──────────────────────────────────────────────────── */
#define IIM42652_WHO_AM_I_VAL        0x6F

/* ── Burst-read length: TEMP(2) + ACCEL(6) + GYRO(6) = 14 bytes ──────── */
#define IIM42652_SENSOR_DATA_LEN     14

/* ═══════════════════════════════════════════════════════════════════════════
 *  Configuration constants
 * ═══════════════════════════════════════════════════════════════════════════*/

/* Gyroscope full-scale select (bits [7:5] of GYRO_CONFIG0) */
#define IIM42652_GYRO_FS_2000DPS     (0x00u << 5)
#define IIM42652_GYRO_FS_1000DPS     (0x01u << 5)
#define IIM42652_GYRO_FS_500DPS      (0x02u << 5)
#define IIM42652_GYRO_FS_250DPS      (0x03u << 5)

/* Accelerometer full-scale select (bits [7:5] of ACCEL_CONFIG0) */
#define IIM42652_ACCEL_FS_16G        (0x00u << 5)
#define IIM42652_ACCEL_FS_8G         (0x01u << 5)
#define IIM42652_ACCEL_FS_4G         (0x02u << 5)
#define IIM42652_ACCEL_FS_2G         (0x03u << 5)

/* Output data rate (bits [3:0] of GYRO/ACCEL_CONFIG0) */
#define IIM42652_ODR_8KHZ            0x03u
#define IIM42652_ODR_4KHZ            0x04u
#define IIM42652_ODR_2KHZ            0x05u
#define IIM42652_ODR_1KHZ            0x06u
#define IIM42652_ODR_200HZ           0x07u
#define IIM42652_ODR_100HZ           0x08u
#define IIM42652_ODR_50HZ            0x09u

/* PWR_MGMT0 fields */
#define IIM42652_PWR_GYRO_LN         (0x03u << 2)  /* Gyro  Low-Noise mode */
#define IIM42652_PWR_ACCEL_LN        (0x03u << 0)  /* Accel Low-Noise mode */

/* ═══════════════════════════════════════════════════════════════════════════
 *  Data types
 * ═══════════════════════════════════════════════════════════════════════════*/

/** Parsed sensor sample (raw 16-bit values). */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temperature;   /* Raw value. °C = raw / 132.48 + 25 */
} IIM42652_Data_t;

/** Driver instance (holds SPI handle + chip-select info). */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
} IIM42652_Handle_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════*/

/**
 * @brief  Initialise the IIM-42652.
 *         Performs soft-reset, verifies WHO_AM_I, configures accel/gyro.
 * @return HAL_OK on success, HAL_ERROR if WHO_AM_I mismatch.
 */
HAL_StatusTypeDef IIM42652_Init(IIM42652_Handle_t *dev,
                                SPI_HandleTypeDef *hspi,
                                GPIO_TypeDef *cs_port,
                                uint16_t cs_pin);

/** @brief  Read WHO_AM_I register (should return 0x6F). */
uint8_t IIM42652_ReadWhoAmI(IIM42652_Handle_t *dev);

/**
 * @brief  Burst-read temperature + accel + gyro (14 bytes) and parse.
 * @return HAL_OK always (SPI errors caught internally).
 */
HAL_StatusTypeDef IIM42652_ReadSensorData(IIM42652_Handle_t *dev,
                                          IIM42652_Data_t *data);

/* Low-level register access (useful for custom configuration) */
uint8_t IIM42652_ReadReg (IIM42652_Handle_t *dev, uint8_t reg);
void    IIM42652_WriteReg(IIM42652_Handle_t *dev, uint8_t reg, uint8_t val);
void    IIM42652_ReadRegs(IIM42652_Handle_t *dev, uint8_t start_reg,
                          uint8_t *buf, uint16_t len);

#endif /* IIM42652_H */
