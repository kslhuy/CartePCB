/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;   /* USART3: NAV link (PD8 TX, PD9 RX) */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <string.h>
#include <stdio.h>

/* ---------- Debug helpers ---------- */
static char debug_buf[256];

static void Debug_Print(const char *msg)
{
  HAL_UART_Transmit(&huart4, (uint8_t *)msg, (uint16_t)strlen(msg), HAL_MAX_DELAY);
}

/* ---------- IMU packet types ---------- */
typedef struct {
  int16_t ax, ay, az;   /* Accel  X/Y/Z  (raw, big-endian on wire) */
  int16_t gx, gy, gz;   /* Gyro   X/Y/Z  */
  int16_t temp;          /* Temperature    */
  uint8_t status;        /* 0x01 = valid   */
} IMU_Data_t;

/* Dummy TX buffer – master clocks out zeros while reading slave MISO */
static const uint8_t spi_tx_dummy[IMU_PKT_LEN] = {0};

/* RX buffer for one SPI packet */
static uint8_t spi_rx_buf[IMU_PKT_LEN];

/* ---------- UART3 (NAV link) RX state machine ---------- */
static uint8_t  uart3_rx_byte;                /* single-byte IT target       */
static uint8_t  uart3_pkt_buf[IMU_PKT_LEN];   /* reassembly buffer           */
static uint8_t  uart3_pkt_idx = 0;            /* next write position         */
static uint8_t  uart3_pkt_ready[IMU_PKT_LEN]; /* latest validated packet     */
static volatile uint8_t uart3_new_pkt = 0;    /* flag: 1 = new packet ready  */

/* ---------- Packet helpers ---------- */

/* Compute XOR checksum over buf[0..len-1] */
static uint8_t IMU_CalcXOR(const uint8_t *buf, uint8_t len)
{
  uint8_t xor_val = 0;
  for (uint8_t i = 0; i < len; i++)
    xor_val ^= buf[i];
  return xor_val;
}

/* Validate header, footer, checksum.  Returns 1 on success. */
static int IMU_ValidatePacket(const uint8_t *buf)
{
  if (buf[0] != IMU_HEADER) return 0;
  if (buf[17] != IMU_FOOTER) return 0;
  if (IMU_CalcXOR(buf, 16) != buf[16]) return 0;   /* bytes 0-15 → chk at 16 */
  return 1;
}

/* Parse validated packet into IMU_Data_t (big-endian int16 on wire) */
static void IMU_ParsePacket(const uint8_t *buf, IMU_Data_t *d)
{
  d->status = buf[1];
  d->ax = (int16_t)((buf[2]  << 8) | buf[3]);
  d->ay = (int16_t)((buf[4]  << 8) | buf[5]);
  d->az = (int16_t)((buf[6]  << 8) | buf[7]);
  d->gx = (int16_t)((buf[8]  << 8) | buf[9]);
  d->gy = (int16_t)((buf[10] << 8) | buf[11]);
  d->gz = (int16_t)((buf[12] << 8) | buf[13]);
  d->temp = (int16_t)((buf[14] << 8) | buf[15]);
}

/* Pretty-print one IMU sample over debug UART */
static void IMU_PrintData(uint32_t seq, const IMU_Data_t *d)
{
  snprintf(debug_buf, sizeof(debug_buf),
    "[%lu] IMU  status=0x%02X\r\n"
    "  Accel  X=%6d  Y=%6d  Z=%6d\r\n"
    "  Gyro   X=%6d  Y=%6d  Z=%6d\r\n"
    "  Temp   %d\r\n",
    (unsigned long)seq, d->status,
    d->ax, d->ay, d->az,
    d->gx, d->gy, d->gz,
    d->temp);
  Debug_Print(debug_buf);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Make sure CS idles HIGH before first transfer */
  SPI3_CS_HIGH();

  /* Startup banner ------------------------------------------------ */
  Debug_Print("\r\n========================================\r\n");
  Debug_Print("  COM H753 - IMU SPI Receiver\r\n");
  Debug_Print("========================================\r\n");
  Debug_Print("[OK] SPI3 Master  (PC10/11/12, PA15 CS)\r\n");
  Debug_Print("[OK] USART3 RX  (PD9) ← NAV USART6_TX\r\n");
  Debug_Print("[OK] UART4 debug console ready\r\n");
  Debug_Print("[OK] Polling NAV with CS-sync\r\n");

  /* --- Wait for NAV to fully boot and arm its SPI slave ----------- */
  Debug_Print("[..] Waiting 500 ms for NAV to boot...\r\n");
  HAL_Delay(500);

  /* Dummy CS cycle: LOW (brief) then HIGH → triggers NAV EXTI
   * so it arms a fresh packet before our first real read.         */
  SPI3_CS_LOW();
  HAL_Delay(1);          /* just long enough for NAV to see the edge */
  SPI3_CS_HIGH();
  HAL_Delay(5);          /* let NAV EXTI handler finish re-arming    */
  Debug_Print("[OK] Sync pulse sent\r\n");
  Debug_Print("----------------------------------------\r\n\r\n");

  /* Start USART3 single-byte IT reception for NAV UART link */
  HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);

  uint32_t pkt_seq    = 0;   /* good-packet counter (SPI)   */
  uint32_t err_count  = 0;   /* validation errors   (SPI)   */
  uint32_t poll_count = 0;   /* total polls         (SPI)   */
  uint32_t last_print_spi  = 0; /* last SPI debug-print tick  */
  uint32_t last_print_uart = 0; /* last UART debug-print tick */
  uint32_t uart_pkt_seq = 0; /* good-packet counter (UART)  */
  IMU_Data_t imu;
  IMU_Data_t imu_uart;       /* separate struct for UART    */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ======== CS-synchronised read ======== */
    /*
     * NAV re-arms its SPI TX buffer on CS rising edge (EXTI),
     * so every CS assertion starts at byte 0 (header = 0xAA).
     * We just clock out all 18 bytes in one burst — no byte-hunt needed.
     */

    SPI3_CS_LOW();
    /* Setup time: let NAV SPI3_NSS hardware latch before first SCK edge.
     * ~1 µs at 480 MHz — covers NAV's APB1 synchroniser (~3 SPI clocks). */
    for (volatile uint16_t dly = 0; dly < 200; dly++) { __NOP(); }
    HAL_SPI_TransmitReceive(&hspi3,
                            (uint8_t *)spi_tx_dummy, spi_rx_buf,
                            IMU_PKT_LEN, 50);
    SPI3_CS_HIGH();

    /* --- Validate ---------------------------------------------------- */
    poll_count++;
    if (!IMU_ValidatePacket(spi_rx_buf))
    {
      err_count++;
      /* Only print errors at most every 200 ms to avoid UART flood */
      uint32_t now = HAL_GetTick();
      if (now - last_print_spi >= 200)
      {
        last_print_spi = now;
        snprintf(debug_buf, sizeof(debug_buf),
                 "[poll=%lu] BAD PKT  hdr=0x%02X ftr=0x%02X xor=0x%02X (exp 0x%02X)  errs=%lu\r\n",
                 (unsigned long)poll_count,
                 spi_rx_buf[0], spi_rx_buf[17],
                 IMU_CalcXOR(spi_rx_buf, 16), spi_rx_buf[16],
                 (unsigned long)err_count);
        Debug_Print(debug_buf);

        /* Hex dump first 10 bad packets so we can see the real alignment */
        if (err_count <= 10)
        {
          Debug_Print("  RAW:");
          for (uint8_t i = 0; i < IMU_PKT_LEN; i++)
          {
            snprintf(debug_buf, sizeof(debug_buf), " %02X", spi_rx_buf[i]);
            Debug_Print(debug_buf);
          }
          Debug_Print("\r\n");
        }
      }
    }
    else
    {
      /* Packet valid — parse */
      pkt_seq++;
      IMU_ParsePacket(spi_rx_buf, &imu);

      /* Print at most every 100 ms (~10 Hz) to not saturate UART */
      uint32_t now = HAL_GetTick();
      if (now - last_print_spi >= 100)
      {
        last_print_spi = now;
        IMU_PrintData(pkt_seq, &imu);
      }
    }

    HAL_Delay(1);   /* poll at ~1 kHz — data always fresh */

    /* ======== UART-based IMU reception (USART3 ← NAV USART6) ======== */
    if (uart3_new_pkt)
    {
      uart3_new_pkt = 0;
      if (IMU_ValidatePacket(uart3_pkt_ready))
      {
        uart_pkt_seq++;
        IMU_ParsePacket(uart3_pkt_ready, &imu_uart);

        uint32_t now2 = HAL_GetTick();
        if (now2 - last_print_uart >= 100)
        {
          last_print_uart = now2;
          snprintf(debug_buf, sizeof(debug_buf),
            "[UART %lu] st=0x%02X  AX=%6d AY=%6d AZ=%6d GX=%6d GY=%6d GZ=%6d T=%d\r\n",
            (unsigned long)uart_pkt_seq, imu_uart.status,
            imu_uart.ax, imu_uart.ay, imu_uart.az,
            imu_uart.gx, imu_uart.gy, imu_uart.gz,
            imu_uart.temp);
          Debug_Print(debug_buf);
        }
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;   /* 192/32 = 6 MHz SPI clock */
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_04CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function (NAV link: PD8=TX, PD9=RX)
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);   /* CS idle HIGH */

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ---------- USART3 RX callback (single-byte IT, reassembles packets) ---------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    uint8_t b = uart3_rx_byte;

    if (uart3_pkt_idx == 0)
    {
      /* Waiting for header */
      if (b == IMU_HEADER)
        uart3_pkt_buf[uart3_pkt_idx++] = b;
      /* else discard */
    }
    else
    {
      uart3_pkt_buf[uart3_pkt_idx++] = b;

      if (uart3_pkt_idx >= IMU_PKT_LEN)
      {
        /* Full packet received — copy to ready buffer */
        memcpy(uart3_pkt_ready, uart3_pkt_buf, IMU_PKT_LEN);
        uart3_new_pkt = 1;
        uart3_pkt_idx = 0;
      }
    }

    /* Re-arm for next byte */
    HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
  }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
