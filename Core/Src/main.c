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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "icm20948_driver.h"
#include "can_mcp2515_odrive.h"
#include "ekf_driver.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ODRV_AXIS_STATE_IDLE                  1U
#define ODRV_AXIS_STATE_CLOSED_LOOP_CONTROL   8U
#define ODRV_CONTROL_MODE_TORQUE_CONTROL      1U
#define ODRV_INPUT_MODE_PASSTHROUGH           1U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
ICM20948_t imu;
ICM20948_PhysSample_t s;
ekf_driver_t g_ekf;
ekf_state_snapshot_t snap;


static uint32_t g_started = 0U;
static uint32_t g_last_diag_ms = 0U;
static uint32_t g_last_cmd_ms  = 0U;
static uint32_t g_test_start_ms = 0U;


static uint32_t t0 = 0;
static uint8_t started = 0;
static float torque = 0.13;



static can_mcp2515_odrive_t g_can_odrive;

//static uint8_t g_started = 0U;
//static char g_diag_line[192];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IMUService(void) {

	(void) ICM20948_Service(&imu);

	if (ICM20948_GetLatestPhys(&imu, &s) == ICM20948_OK) {
		if (imu.gyro_cal_start) {
		  imu.gyro_cal_start = 0;
		  (void) ICM20948_CalGyro(&imu);
		}
	}

	// старт калибровки
	if (imu.cal_start) {
	  imu.cal_start = 0;
      (void) ICM20948_CalReset(&imu);
	}

	// начать сбор для текущего положения
	if (imu.cal_step) {
	  imu.cal_step = 0;
	  (void) ICM20948_CalSetStep(&imu);
	}

}

static void app_debug_write_line(const char *s)
{
    char buf[256];
    int n = snprintf(buf, sizeof(buf), "%s\r\n", s);
    if (n <= 0) {
        return;
    }

    if (n >= (int)sizeof(buf)) {
        n = sizeof(buf) - 1;
    }

    CDC_Transmit_FS((uint8_t *)buf, (uint16_t)n);
}

void app_can_odrive_init(void)
{
    memset(&g_can_odrive, 0, sizeof(g_can_odrive));

    g_can_odrive.hspi = &hspi3;
    g_can_odrive.cs_port = GPIOB;
    g_can_odrive.cs_pin = GPIO_PIN_6;
    g_can_odrive.int_port = GPIOB;
    g_can_odrive.int_pin = GPIO_PIN_7;
    g_can_odrive.mcp2515_osc_hz = 8000000U;
    g_can_odrive.can_bitrate = 250000U;
    g_can_odrive.node_id_1 = 1U;
    g_can_odrive.node_id_2 = 2U;
    g_can_odrive.mode = CAN_MCP2515_ODRIVE_MODE_NORMAL;
    g_can_odrive.poll_pair_rate_hz = 250U;
    g_can_odrive.reply_timeout_us = 3000U;
    g_can_odrive.spi_timeout_us = 2000U;

    if (can_mcp2515_odrive_init(&g_can_odrive) != CAN_MCP2515_ODRIVE_STATUS_OK)
    {
        Error_Handler();
    }

    if (can_mcp2515_odrive_enable_polling(&g_can_odrive, 1U) != CAN_MCP2515_ODRIVE_STATUS_OK)
    {
        Error_Handler();
    }

    g_started = 0U;
    g_last_diag_ms = HAL_GetTick();
}

void app_can_odrive_start_sequence(void)
{
    if (g_started != 0U)
    {
        return;
    }

    // 8 = CLOSED_LOOP_CONTROL в ODrive enum.
    // При необходимости замени на свои значения state/control/input mode.
    (void)odrive_pair_set_axis_state(&g_can_odrive, 8U, 8U);
   // (void)odrive_pair_set_input_torque(&g_can_odrive, 0.0f, 0.0f);
    g_started = 1U;
}

void odrive_full_init(void)
{
	app_can_odrive_init();
	HAL_Delay(20);

	app_can_odrive_start_sequence();
	HAL_Delay(20);

	// Сначала torque mode + passthrough
	(void)odrive_pair_set_controller_mode(&g_can_odrive, ODRV_CONTROL_MODE_TORQUE_CONTROL,ODRV_INPUT_MODE_PASSTHROUGH,ODRV_CONTROL_MODE_TORQUE_CONTROL,ODRV_INPUT_MODE_PASSTHROUGH);
	HAL_Delay(20);

	// Нулевой момент до входа в closed loop
	(void)odrive_pair_set_input_torque(&g_can_odrive, 0.0f, 0.0f);
	HAL_Delay(20);

	// Переводим обе оси в CLOSED_LOOP_CONTROL
	(void)odrive_pair_set_axis_state(&g_can_odrive,ODRV_AXIS_STATE_CLOSED_LOOP_CONTROL, ODRV_AXIS_STATE_CLOSED_LOOP_CONTROL );
	// ODRV_AXIS_STATE_CLOSED_LOOP_CONTROL,
	//ODRV_AXIS_STATE_CLOSED_LOOP_CONTROL
	HAL_Delay(50);

	g_last_diag_ms  = HAL_GetTick();
	g_last_cmd_ms   = HAL_GetTick();
	g_test_start_ms = HAL_GetTick();
}

void imu_init(void)
{
	imu.p.hspi = &hspi1;
	imu.p.hdma_rx = &hdma_spi1_rx;
	imu.p.hdma_tx = &hdma_spi1_tx;
	imu.p.cs_port = GPIOA;
	imu.p.cs_pin  = GPIO_PIN_4; // CS -> PA4
	imu.p.int_port = GPIOA;
	imu.p.int_pin  = GPIO_PIN_3; // INT -> PA3
	imu.p.spi_timeout_ms = ICM20948_SPI_TIMEOUT_MS_DEFAULT;


	ICM20948_Init(&imu);


	while (imu.raw_valid == 0u)
	{
		ICM20948_Service(&imu);
	}


	ICM20948_CalGyro(&imu);

}

void EKF_init(void){
  ekf_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.pitch_gyro_axis = 1u;
  cfg.pitch_acc_forward_axis = 0u;
  cfg.pitch_acc_up_axis = 2u;
  cfg.pitch_gyro_sign = +1;
  cfg.pitch_acc_forward_sign = +1;
  cfg.pitch_acc_up_sign = +1;
  cfg.imu_stale_timeout_us = 5000u;
  cfg.odrv_stale_timeout_us = 12000u;
  cfg.q_pitch = 0.5f;
  cfg.q_gyro_bias = 0.01f;
  cfg.q_wheel_pos = 0.1f;
  cfg.q_wheel_vel = 3.0f;
  cfg.q_diff_pos = 0.1f;
  cfg.q_diff_vel = 3.0f;
  cfg.r_pitch_acc = 0.03f;
  cfg.r_wheel_pos = 0.001f;
  cfg.r_wheel_vel = 0.01f;
  cfg.r_diff_pos = 0.001f;
  cfg.r_diff_vel = 0.01f;
  cfg.p0_pitch = 0.3f;
  cfg.p0_gyro_bias = 0.2f;
  cfg.p0_wheel_pos = 1.0f;
  cfg.p0_wheel_vel = 1.0f;
  cfg.p0_diff_pos = 1.0f;
  cfg.p0_diff_vel = 1.0f;

  cfg.wheel1_sign = +1;
  cfg.wheel2_sign = -1;

  (void)ekf_driver_init(&g_ekf, &cfg);
}



int _write(int file, char *ptr, int len)
{
    // Просто передаем данные в USB
    CDC_Transmit_FS((uint8_t*)ptr, len);
    return len;
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */


  imu_init();

  odrive_full_init();

  HAL_TIM_Base_Start_IT(&htim11);

  EKF_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	ICM20948_Service(&imu);

	can_mcp2515_odrive_process(&g_can_odrive);

	//ICM20948_GetLatestPhys(&imu,&s);

    ekf_driver_update_inputs_from_drivers(&g_ekf, &imu, &g_can_odrive);
    ekf_driver_process_pending(&g_ekf);
    ekf_driver_get_state_snapshot(&g_ekf, &snap);







    if (!started)
    {
        started = 1;
        t0 = HAL_GetTick();

        (void)odrive_pair_set_controller_mode(&g_can_odrive, 1, 1, 1, 1); // torque + passthrough, если у тебя такие enum
        (void)odrive_pair_set_axis_state(&g_can_odrive, 8, 8);             // CLOSED_LOOP_CONTROL
    }

    if ((HAL_GetTick() - t0) < 1500U)
    {
        (void)odrive_pair_set_input_torque(&g_can_odrive, +torque, -torque);
    }
    else
    {
        (void)odrive_pair_set_input_torque(&g_can_odrive, 0, 0);
    }






    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 83;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ICM_CS_Pin */
  GPIO_InitStruct.Pin = ICM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ICM_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_7)
  {
	  can_mcp2515_odrive_exti_callback(&g_can_odrive, GPIO_Pin);

  }

  if (GPIO_Pin == GPIO_PIN_3)
    {
      ICM20948_IrqHandler(&imu, GPIO_Pin);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    can_mcp2515_odrive_spi_txrx_cplt_callback(&g_can_odrive, hspi);
    (void)ICM20948_SpiDmaCpltHandler(&imu, hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    can_mcp2515_odrive_spi_error_callback(&g_can_odrive, hspi);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	  if (htim->Instance == TIM11)
	  {
		  (void)ekf_driver_request_step_isr(&g_ekf, icm_micros64(&imu));
	  }
}

/* USER CODE END 4 */

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
