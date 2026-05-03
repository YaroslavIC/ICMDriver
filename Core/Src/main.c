/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Rover controller main program body
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
#include "rover_drive.h"
#include "hardwareinit.h"
#include "app_serial.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ODRV_AXIS_STATE_IDLE                  1U
#define ODRV_AXIS_STATE_CLOSED_LOOP_CONTROL   8U
#define ODRV_CONTROL_MODE_VELOCITY_CONTROL    2U
#define ODRV_INPUT_MODE_PASSTHROUGH           1U
#define APP_ODRIVE_STATE_CMD_PERIOD_MS        250U
#define APP_ROVER_DRIVE_CMD_ACTIVE_EPS       (0.001f)
#define APP_ODRIVE_LEFT_VEL_GAIN             (0.05f)
#define APP_ODRIVE_LEFT_VEL_INTEGRATOR_GAIN  (0.10f)
#define APP_ODRIVE_RIGHT_VEL_GAIN            (0.05f)
#define APP_ODRIVE_RIGHT_VEL_INTEGRATOR_GAIN (0.10f)
#define APP_ODRIVE_LEFT_VEL_LIMIT_REV_S      (2.0f)
#define APP_ODRIVE_RIGHT_VEL_LIMIT_REV_S     (2.0f)
#define APP_ODRIVE_LEFT_CURRENT_LIMIT_A      (10.0f)
#define APP_ODRIVE_RIGHT_CURRENT_LIMIT_A     (10.0f)
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

static uint8_t g_odrive_closed_loop_requested = 0u;
static uint8_t g_odrive_idle_requested = 0u;
static uint32_t g_last_axis_state_cmd_ms = 0u;
static uint8_t g_last_control_enabled = 0u;

static volatile uint8_t g_drive_step_pending = 0u;

static can_mcp2515_odrive_t g_can_odrive;
static app_runtime_t g_app;
static rover_drive_output_t g_rover_out;
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
static void RoverLedStep(void);
static void DebugTelemetryStep(void);
static void RoverDriveStep(void);
static void AppOdriveRequestIdle(uint32_t now_ms);
static void AppOdriveRequestClosedLoop(uint32_t now_ms);
static void AppOdriveApplyVelocityRuntimeConfig(void);
static void app_debug_printf(const char *fmt, ...);
static uint8_t app_are_axes_ready(void);
static uint8_t app_has_odrive_errors(void);
static app_serial_status_t app_serial_custom_cmd_cb(app_serial_t *serial, void *ctx, const char *line, uint8_t *handled);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MAIN_MODULE_VERSION                 "M10"
#define PROJECT_ROVER_DEBUG_REVISION        MAIN_MODULE_VERSION "-" ROVER_DRIVE_MODULE_VERSION "-" HARDWAREINIT_MODULE_VERSION "-" APP_SERIAL_MODULE_VERSION "-O0.2-I1.1.0"

void IMUService(void)
{
    (void)ICM20948_Service(&imu);

    if (ICM20948_GetLatestPhys(&imu, &s) == ICM20948_OK)
    {
        if (imu.gyro_cal_start != 0u)
        {
            imu.gyro_cal_start = 0u;
            (void)ICM20948_CalGyro(&imu);
        }
    }

    if (imu.cal_start != 0u)
    {
        imu.cal_start = 0u;
        (void)ICM20948_CalReset(&imu);
    }

    if (imu.cal_step != 0u)
    {
        imu.cal_step = 0u;
        (void)ICM20948_CalSetStep(&imu);
    }
}

static void app_debug_printf(const char *fmt, ...)
{
    char buf[384];
    va_list args;
    int n;

    va_start(args, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (n <= 0)
    {
        return;
    }

    if (n >= (int)sizeof(buf))
    {
        n = (int)sizeof(buf) - 1;
    }

    (void)CDC_Transmit_FS((uint8_t *)buf, (uint16_t)n);
}

static uint8_t app_are_axes_ready(void)
{
    if ((g_can_odrive.node1.heartbeat.valid == 0u) || (g_can_odrive.node2.heartbeat.valid == 0u))
    {
        return 0u;
    }

    if ((g_can_odrive.node1.heartbeat.axis_state != ODRV_AXIS_STATE_CLOSED_LOOP_CONTROL) ||
        (g_can_odrive.node2.heartbeat.axis_state != ODRV_AXIS_STATE_CLOSED_LOOP_CONTROL))
    {
        return 0u;
    }

    return 1u;
}

static uint8_t app_has_odrive_errors(void)
{
    if ((g_can_odrive.node1.heartbeat.axis_error != 0u) || (g_can_odrive.node2.heartbeat.axis_error != 0u))
    {
        return 1u;
    }
    if ((g_can_odrive.node1.error.valid != 0u) && (g_can_odrive.node1.error.active_errors != 0u))
    {
        return 1u;
    }
    if ((g_can_odrive.node2.error.valid != 0u) && (g_can_odrive.node2.error.active_errors != 0u))
    {
        return 1u;
    }
    return 0u;
}

static void RoverLedStep(void)
{
    if (app_has_odrive_errors() != 0u)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        return;
    }

    if ((g_app.control_enabled != 0u) && (app_are_axes_ready() != 0u))
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    }
}

static void DebugTelemetryStep(void)
{
    static uint32_t last_dbg_ms = 0u;
    uint32_t now_ms;
    uint32_t dbg_period_ms;
    ICM20948_PhysSample_t imu_sample;
    uint8_t imu_valid;

    now_ms = HAL_GetTick();
    dbg_period_ms = (g_app.control_enabled != 0u) ? 100u : 500u;
    if ((uint32_t)(now_ms - last_dbg_ms) < dbg_period_ms)
    {
        return;
    }
    last_dbg_ms = now_ms;

    imu_valid = (ICM20948_GetLatestPhys(&imu, &imu_sample) == ICM20948_OK) ? 1u : 0u;
    if (imu_valid != 0u)
    {
        s = imu_sample;
    }

    app_debug_printf(
        "DBG[%s] t=%lu en=%u mode=%s ready=%u err=%u tout=%u "
        "fwd=%ld turn=%ld Bv=%ld Bt=%lu Ba=%u Ltar=%ld Rtar=%ld Lout=%ld Rout=%ld "
        "Lvel=%ld Rvel=%ld Lstate=%u Rstate=%u Lerr=%lu Rerr=%lu "
        "gz=%ld ax=%ld ay=%ld az=%ld\r\n",

        PROJECT_ROVER_DEBUG_REVISION,
        (unsigned long)now_ms,
        (unsigned)g_app.control_enabled,
        rover_drive_mode_to_string(g_rover_out.mode),
        (unsigned)app_are_axes_ready(),
        (unsigned)app_has_odrive_errors(),
        (unsigned)g_rover_out.timeout,

        (long)(g_rover_out.forward_cmd * 1000.0f),
        (long)(g_rover_out.turn_cmd * 1000.0f),
        (long)(g_rover_out.boost_vel_rev_s * 1000.0f),
        (unsigned long)g_rover_out.boost_ms,
        (unsigned)g_rover_out.boost_active,
        (long)(g_rover_out.left_target_rev_s * 1000.0f),
        (long)(g_rover_out.right_target_rev_s * 1000.0f),
        (long)(g_rover_out.left_output_rev_s * 1000.0f),
        (long)(g_rover_out.right_output_rev_s * 1000.0f),

        (long)(g_can_odrive.pair.vel1 * 1000.0f),
        (long)(g_can_odrive.pair.vel2 * 1000.0f),
        (unsigned)g_can_odrive.node1.heartbeat.axis_state,
        (unsigned)g_can_odrive.node2.heartbeat.axis_state,
        (unsigned long)g_can_odrive.node1.heartbeat.axis_error,
        (unsigned long)g_can_odrive.node2.heartbeat.axis_error,

        (long)(s.gyro_rads[2] * 1000.0f),
        (long)(s.accel_mps2[0] * 1000.0f),
        (long)(s.accel_mps2[1] * 1000.0f),
        (long)(s.accel_mps2[2] * 1000.0f)
    );
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
    g_can_odrive.node_id_1 = 2U;   // левое заднее ведущее колесо
    g_can_odrive.node_id_2 = 1U;   // правое заднее ведущее колесо
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

    g_odrive_closed_loop_requested = 0u;
    g_odrive_idle_requested = 0u;
    g_last_axis_state_cmd_ms = 0u;
    g_last_control_enabled = 0u;
}

static void AppOdriveApplyVelocityRuntimeConfig(void)
{
    (void)odrive_pair_set_controller_mode(&g_can_odrive,
                                          ODRV_CONTROL_MODE_VELOCITY_CONTROL,
                                          ODRV_INPUT_MODE_PASSTHROUGH,
                                          ODRV_CONTROL_MODE_VELOCITY_CONTROL,
                                          ODRV_INPUT_MODE_PASSTHROUGH);
    (void)odrive_pair_set_vel_gains(&g_can_odrive,
                                    APP_ODRIVE_LEFT_VEL_GAIN,
                                    APP_ODRIVE_LEFT_VEL_INTEGRATOR_GAIN,
                                    APP_ODRIVE_RIGHT_VEL_GAIN,
                                    APP_ODRIVE_RIGHT_VEL_INTEGRATOR_GAIN);
    (void)odrive_pair_set_limits(&g_can_odrive,
                                 APP_ODRIVE_LEFT_VEL_LIMIT_REV_S,
                                 APP_ODRIVE_LEFT_CURRENT_LIMIT_A,
                                 APP_ODRIVE_RIGHT_VEL_LIMIT_REV_S,
                                 APP_ODRIVE_RIGHT_CURRENT_LIMIT_A);
}

static void AppOdriveRequestIdle(uint32_t now_ms)
{
    if ((g_odrive_idle_requested != 0u) &&
        ((uint32_t)(now_ms - g_last_axis_state_cmd_ms) < APP_ODRIVE_STATE_CMD_PERIOD_MS))
    {
        return;
    }

    (void)odrive_pair_set_input_vel(&g_can_odrive, 0.0f, 0.0f, 0.0f, 0.0f);
    (void)odrive_pair_set_axis_state(&g_can_odrive,
                                     ODRV_AXIS_STATE_IDLE,
                                     ODRV_AXIS_STATE_IDLE);
    g_odrive_idle_requested = 1u;
    g_odrive_closed_loop_requested = 0u;
    g_last_axis_state_cmd_ms = now_ms;
}

static void AppOdriveRequestClosedLoop(uint32_t now_ms)
{
    if ((g_odrive_closed_loop_requested != 0u) &&
        ((uint32_t)(now_ms - g_last_axis_state_cmd_ms) < APP_ODRIVE_STATE_CMD_PERIOD_MS))
    {
        return;
    }

    AppOdriveApplyVelocityRuntimeConfig();
    (void)odrive_pair_set_input_vel(&g_can_odrive, 0.0f, 0.0f, 0.0f, 0.0f);
    (void)odrive_pair_set_axis_state(&g_can_odrive,
                                     ODRV_AXIS_STATE_CLOSED_LOOP_CONTROL,
                                     ODRV_AXIS_STATE_CLOSED_LOOP_CONTROL);
    g_odrive_closed_loop_requested = 1u;
    g_odrive_idle_requested = 0u;
    g_last_axis_state_cmd_ms = now_ms;
}

void odrive_full_init(void)
{
    app_can_odrive_init();
    HAL_Delay(20);

    AppOdriveApplyVelocityRuntimeConfig();
    HAL_Delay(20);

    AppOdriveRequestIdle(HAL_GetTick());
    HAL_Delay(50);
}

void imu_init(void)
{
    imu.p.hspi = &hspi1;
    imu.p.hdma_rx = &hdma_spi1_rx;
    imu.p.hdma_tx = &hdma_spi1_tx;
    imu.p.cs_port = GPIOA;
    imu.p.cs_pin  = GPIO_PIN_4;
    imu.p.int_port = GPIOA;
    imu.p.int_pin  = GPIO_PIN_3;
    imu.p.spi_timeout_ms = ICM20948_SPI_TIMEOUT_MS_DEFAULT;

    ICM20948_Init(&imu);

    while (imu.raw_valid == 0u)
    {
        ICM20948_Service(&imu);
    }

    (void)ICM20948_CalGyro(&imu);
    (void)ICM20948_GetLatestPhys(&imu, &s);
}

static void RoverDriveStep(void)
{
    uint32_t now_ms;
    uint8_t fault;
    uint8_t cmd_active;

    now_ms = HAL_GetTick();
    fault = app_has_odrive_errors();

    if ((g_app.control_enabled != 0u) && (g_last_control_enabled == 0u))
    {
        rover_drive_stop(&g_app.command, now_ms);
        rover_drive_reset_outputs(&g_app.drive, now_ms);
        memset(&g_rover_out, 0, sizeof(g_rover_out));
        AppOdriveRequestIdle(now_ms);
    }
    g_last_control_enabled = g_app.control_enabled;

    if (g_app.control_enabled == 0u)
    {
        (void)rover_drive_step(&g_app.drive, 0u, 0u, &g_app.command, now_ms, &g_rover_out);
        AppOdriveRequestIdle(now_ms);
        return;
    }

    if (fault != 0u)
    {
        (void)rover_drive_step(&g_app.drive, 0u, 1u, &g_app.command, now_ms, &g_rover_out);
        AppOdriveRequestIdle(now_ms);
        return;
    }

    if (rover_drive_step(&g_app.drive,
                         1u,
                         0u,
                         &g_app.command,
                         now_ms,
                         &g_rover_out) != ROVER_DRIVE_STATUS_OK)
    {
        AppOdriveRequestIdle(now_ms);
        return;
    }

    if (g_rover_out.timeout != 0u)
    {
        AppOdriveRequestIdle(now_ms);
        return;
    }

    cmd_active = 0u;
    if ((g_rover_out.forward_cmd > APP_ROVER_DRIVE_CMD_ACTIVE_EPS) ||
        (g_rover_out.forward_cmd < -APP_ROVER_DRIVE_CMD_ACTIVE_EPS) ||
        (g_rover_out.turn_cmd > APP_ROVER_DRIVE_CMD_ACTIVE_EPS) ||
        (g_rover_out.turn_cmd < -APP_ROVER_DRIVE_CMD_ACTIVE_EPS))
    {
        cmd_active = 1u;
    }

    if (cmd_active == 0u)
    {
        AppOdriveRequestIdle(now_ms);
        return;
    }

    if (app_are_axes_ready() == 0u)
    {
        AppOdriveRequestClosedLoop(now_ms);
        return;
    }

    g_odrive_closed_loop_requested = 1u;
    g_odrive_idle_requested = 0u;

    (void)odrive_pair_set_input_vel(&g_can_odrive,
                                    g_rover_out.left_output_rev_s,
                                    0.0f,
                                    g_rover_out.right_output_rev_s,
                                    0.0f);
}


static app_serial_status_t app_serial_custom_cmd_cb(app_serial_t *serial, void *ctx, const char *line, uint8_t *handled)
{
    app_runtime_t *app;
    float forward;
    float turn;
    float boost_vel;
    float boost_ms;
    int parsed;

    if ((serial == NULL) || (ctx == NULL) || (line == NULL) || (handled == NULL))
    {
        return APP_SERIAL_STATUS_BAD_ARG;
    }

    *handled = 0u;
    app = (app_runtime_t *)ctx;

    boost_vel = ROVER_DRIVE_DEFAULT_BOOST_VEL_REV_S;
    boost_ms = ROVER_DRIVE_DEFAULT_BOOST_MS;
    parsed = sscanf(line, "drive %f %f %f %f", &forward, &turn, &boost_vel, &boost_ms);
    if (parsed >= 2)
    {
        *handled = 1u;
        if (rover_drive_set_command_boost(&app->command,
                                          forward,
                                          turn,
                                          boost_vel,
                                          boost_ms,
                                          HAL_GetTick()) != ROVER_DRIVE_STATUS_OK)
        {
            return app_serial_printf(serial, "ERR usage drive <forward:-1..1> <turn:-1..1> [boost_vel_rev_s:0..5] [boost_ms:0..3000]\r\n");
        }
        return app_serial_printf(serial,
                                 "RSP drive fwd=%.3f turn=%.3f boost_vel=%.3f boost_ms=%.0f\r\n",
                                 forward,
                                 turn,
                                 boost_vel,
                                 boost_ms);
    }

    if (strcmp(line, "stop") == 0)
    {
        *handled = 1u;
        app->control_enabled = 0u;
        rover_drive_stop(&app->command, HAL_GetTick());
        (void)rover_drive_step(&app->drive, 0u, 0u, &app->command, HAL_GetTick(), &g_rover_out);
        AppOdriveRequestIdle(HAL_GetTick());
        return app_serial_printf(serial, "RSP stop en=0\r\n");
    }

    if (strcmp(line, "get odrive") == 0)
    {
        *handled = 1u;
        return app_serial_printf(serial,
                                 "RSP odrive ready=%u err=%u Lstate=%u Rstate=%u Lerr=%lu Rerr=%lu Lvel=%.4f Rvel=%.4f\r\n",
                                 (unsigned)app_are_axes_ready(),
                                 (unsigned)app_has_odrive_errors(),
                                 (unsigned)g_can_odrive.node1.heartbeat.axis_state,
                                 (unsigned)g_can_odrive.node2.heartbeat.axis_state,
                                 (unsigned long)g_can_odrive.node1.heartbeat.axis_error,
                                 (unsigned long)g_can_odrive.node2.heartbeat.axis_error,
                                 g_can_odrive.pair.vel1,
                                 g_can_odrive.pair.vel2);
    }

    if (strcmp(line, "get imu") == 0)
    {
        *handled = 1u;
        (void)ICM20948_GetLatestPhys(&imu, &s);
        return app_serial_printf(serial,
                                 "RSP imu ax=%ld ay=%ld az=%ld gx=%ld gy=%ld gz=%ld\r\n",
                                 (long)(s.accel_mps2[0] * 1000.0f),
                                 (long)(s.accel_mps2[1] * 1000.0f),
                                 (long)(s.accel_mps2[2] * 1000.0f),
                                 (long)(s.gyro_rads[0] * 1000.0f),
                                 (long)(s.gyro_rads[1] * 1000.0f),
                                 (long)(s.gyro_rads[2] * 1000.0f));
    }

    return APP_SERIAL_STATUS_EMPTY;
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

  if (hardwareinit_runtime_init(&g_app) != HARDWAREINIT_STATUS_OK)
  {
    Error_Handler();
  }

  imu_init();

  odrive_full_init();

  g_app.serial.custom_cmd = app_serial_custom_cmd_cb;
  g_app.serial.custom_ctx = &g_app;

  rover_drive_stop(&g_app.command, HAL_GetTick());
  rover_drive_reset_outputs(&g_app.drive, HAL_GetTick());
  memset(&g_rover_out, 0, sizeof(g_rover_out));

  HAL_TIM_Base_Start_IT(&htim11);
  (void)app_serial_printf(&g_app.serial, "RSP ready rover=%s\r\n", PROJECT_ROVER_DEBUG_REVISION);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
        IMUService();

        can_mcp2515_odrive_process(&g_can_odrive);

        RoverLedStep();
        (void)app_serial_process(&g_app.serial);
        DebugTelemetryStep();

        if (g_drive_step_pending != 0u)
        {
            g_drive_step_pending = 0u;
            RoverDriveStep();
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t drive_div = 0u;
    if (htim->Instance == TIM11)
    {
        drive_div++;
        if (drive_div >= 5u)
        {
            drive_div = 0u;
            g_drive_step_pending = 1u;
        }
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
