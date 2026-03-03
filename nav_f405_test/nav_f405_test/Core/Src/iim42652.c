/**
 * @file    iim42652.c
 * @brief   IIM-42652 (TDK InvenSense) 6-axis IMU driver — SPI implementation
 *
 * Default configuration after Init():
 *   - Gyroscope:     ±2000 dps, 1 kHz ODR, Low-Noise mode
 *   - Accelerometer: ±16 g,     1 kHz ODR, Low-Noise mode
 *
 * SPI protocol:  bit 7 of first byte = 1 → read, 0 → write.
 *                Address auto-increments for burst reads.
 */
#include "iim42652.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Internal helpers — chip-select control
 * ═══════════════════════════════════════════════════════════════════════════*/

static inline void CS_Low(IIM42652_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void CS_High(IIM42652_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Low-level register access
 * ═══════════════════════════════════════════════════════════════════════════*/

uint8_t IIM42652_ReadReg(IIM42652_Handle_t *dev, uint8_t reg)
{
    uint8_t tx[2] = { 0x80u | reg, 0x00u };
    uint8_t rx[2];

    CS_Low(dev);
    HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2, HAL_MAX_DELAY);
    CS_High(dev);

    return rx[1];
}

void IIM42652_WriteReg(IIM42652_Handle_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg & 0x7Fu, val };

    CS_Low(dev);
    HAL_SPI_Transmit(dev->hspi, tx, 2, HAL_MAX_DELAY);
    CS_High(dev);
}

void IIM42652_ReadRegs(IIM42652_Handle_t *dev, uint8_t start_reg,
                       uint8_t *buf, uint16_t len)
{
    uint8_t cmd = 0x80u | start_reg;

    CS_Low(dev);
    HAL_SPI_Transmit(dev->hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(dev->hspi, buf, len, HAL_MAX_DELAY);
    CS_High(dev);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════*/

uint8_t IIM42652_ReadWhoAmI(IIM42652_Handle_t *dev)
{
    return IIM42652_ReadReg(dev, IIM42652_REG_WHO_AM_I);
}

HAL_StatusTypeDef IIM42652_Init(IIM42652_Handle_t *dev,
                                SPI_HandleTypeDef *hspi,
                                GPIO_TypeDef *cs_port,
                                uint16_t cs_pin)
{
    dev->hspi    = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin  = cs_pin;

    /* CS idle high */
    CS_High(dev);
    HAL_Delay(10);

    /* ── Soft reset ── */
    IIM42652_WriteReg(dev, IIM42652_REG_DEVICE_CONFIG, 0x01);
    HAL_Delay(10);                        /* Wait for reset to complete   */

    /* ── Verify WHO_AM_I ── */
    uint8_t id = IIM42652_ReadWhoAmI(dev);
    if (id != IIM42652_WHO_AM_I_VAL)
    {
        return HAL_ERROR;                 /* Wrong device or wiring issue */
    }

    /* ── Select Bank 0 (default, but be explicit) ── */
    IIM42652_WriteReg(dev, IIM42652_REG_BANK_SEL, 0x00);
    HAL_Delay(1);

    /* ── Gyro config: ±2000 dps, 1 kHz ODR ── */
    IIM42652_WriteReg(dev, IIM42652_REG_GYRO_CONFIG0,
                      IIM42652_GYRO_FS_2000DPS | IIM42652_ODR_1KHZ);
    HAL_Delay(1);

    /* ── Accel config: ±16 g, 1 kHz ODR ── */
    IIM42652_WriteReg(dev, IIM42652_REG_ACCEL_CONFIG0,
                      IIM42652_ACCEL_FS_16G | IIM42652_ODR_1KHZ);
    HAL_Delay(1);

    /* ── Power on both sensors in Low-Noise mode ── */
    IIM42652_WriteReg(dev, IIM42652_REG_PWR_MGMT0,
                      IIM42652_PWR_GYRO_LN | IIM42652_PWR_ACCEL_LN);
    HAL_Delay(50);                        /* Gyro needs ~45 ms to start   */

    return HAL_OK;
}

HAL_StatusTypeDef IIM42652_ReadSensorData(IIM42652_Handle_t *dev,
                                          IIM42652_Data_t *data)
{
    uint8_t raw[IIM42652_SENSOR_DATA_LEN];

    /* Burst-read 14 bytes starting at TEMP_DATA1 (0x1D):
     *   [0..1]  Temperature  (big-endian int16)
     *   [2..7]  Accel X/Y/Z  (big-endian int16 each)
     *   [8..13] Gyro  X/Y/Z  (big-endian int16 each)
     */
    IIM42652_ReadRegs(dev, IIM42652_REG_TEMP_DATA1, raw, IIM42652_SENSOR_DATA_LEN);

    /* Parse big-endian 16-bit values */
    data->temperature = (int16_t)((uint16_t)raw[0]  << 8 | raw[1]);
    data->accel_x     = (int16_t)((uint16_t)raw[2]  << 8 | raw[3]);
    data->accel_y     = (int16_t)((uint16_t)raw[4]  << 8 | raw[5]);
    data->accel_z     = (int16_t)((uint16_t)raw[6]  << 8 | raw[7]);
    data->gyro_x      = (int16_t)((uint16_t)raw[8]  << 8 | raw[9]);
    data->gyro_y      = (int16_t)((uint16_t)raw[10] << 8 | raw[11]);
    data->gyro_z      = (int16_t)((uint16_t)raw[12] << 8 | raw[13]);

    return HAL_OK;
}
