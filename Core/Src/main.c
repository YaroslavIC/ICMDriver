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
#include <stdarg.h>
#include "usbd_cdc_if.h"
#include "app_serial.h"
#include "flash_cfg_store.h"
#include "balance.h"
#include "hardwareinit.h"
#include "balance_calibration.h"
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
static uint32_t g_last_cal_encoder_req_ms = 0U;

static volatile uint8_t g_ctrl_step_pending = 0u;
static volatile uint64_t g_ctrl_step_ts_us = 0ull;

static can_mcp2515_odrive_t g_can_odrive;
static app_runtime_t g_app;
static balance_output_t g_balance_out;
static balance_calibration_t g_balance_cal;
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
static void VerticalLedStep(const ekf_state_snapshot_t *state);
static void DebugTelemetryStep(void);
static void ControlTorqueStep(void);
static void app_log_line_cb(void *ctx, const char *line);
static float app_get_left_vel_cb(void *ctx);
static float app_get_right_vel_cb(void *ctx);
static balance_calibration_status_t app_set_pair_torque_cb(void *ctx, float u_left, float u_right);
static uint8_t app_are_axes_ready_cb(void *ctx);
static uint8_t app_has_errors_cb(void *ctx);
static uint8_t app_is_control_enabled_cb(void *ctx);
static balance_calibration_status_t app_set_control_enabled_cb(void *ctx, uint8_t value);
static balance_calibration_status_t app_save_calibration_cb(void *ctx, const balance_calibration_persist_t *persist);
static app_serial_status_t app_serial_custom_cmd_cb(app_serial_t *serial, void *ctx, const char *line, uint8_t *handled);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MAIN_MODULE_VERSION                 "M04"
#define PROJECT_BALANCE_DEBUG_REVISION  MAIN_MODULE_VERSION "-" BALANCE_MODULE_VERSION "-" HARDWAREINIT_MODULE_VERSION "-" APP_SERIAL_MODULE_VERSION "-" EKF_DRIVER_MODULE_VERSION

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
    char buf[320];
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

static void app_log_line_cb(void *ctx, const char *line)
{
    (void)ctx;
    if (line == NULL)
    {
        return;
    }
    app_debug_printf("%s\r\n", line);
}

static float app_get_left_vel_cb(void *ctx)
{
    (void)ctx;

    if (snap.valid != 0u)
    {
        return snap.wheel_vel_left_raw;
    }

    return g_can_odrive.pair.vel1;
}

static float app_get_right_vel_cb(void *ctx)
{
    (void)ctx;

    if (snap.valid != 0u)
    {
        return snap.wheel_vel_right_raw;
    }

    return g_can_odrive.pair.vel2;
}

static balance_calibration_status_t app_set_pair_torque_cb(void *ctx, float u_left, float u_right)
{
    (void)ctx;
    if (odrive_pair_set_input_torque(&g_can_odrive, u_left, u_right) != CAN_MCP2515_ODRIVE_STATUS_OK)
    {
        return BALANCE_CALIBRATION_STATUS_ERROR;
    }
    return BALANCE_CALIBRATION_STATUS_OK;
}

static uint8_t app_are_axes_ready_cb(void *ctx)
{
    (void)ctx;
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

static uint8_t app_has_errors_cb(void *ctx)
{
    (void)ctx;
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

static uint8_t app_is_control_enabled_cb(void *ctx)
{
    app_runtime_t *app;

    app = (app_runtime_t *)ctx;
    if (app == NULL)
    {
        return 0u;
    }
    return app->control_enabled;
}

static balance_calibration_status_t app_set_control_enabled_cb(void *ctx, uint8_t value)
{
    app_runtime_t *app;

    app = (app_runtime_t *)ctx;
    if (app == NULL)
    {
        return BALANCE_CALIBRATION_STATUS_BAD_ARG;
    }

    app->control_enabled = (value != 0u) ? 1u : 0u;
    if (app->control_enabled == 0u)
    {
        app->command.motion_fwd_cmd = 0.0f;
        app->command.motion_turn_cmd = 0.0f;
        (void)balance_reset(&app->balance, snap.wheel_pos_avg, HAL_GetTick());
    }

    return BALANCE_CALIBRATION_STATUS_OK;
}

static balance_calibration_status_t app_save_calibration_cb(void *ctx, const balance_calibration_persist_t *persist)
{
    app_runtime_t *app;

    app = (app_runtime_t *)ctx;
    if ((app == NULL) || (persist == NULL))
    {
        return BALANCE_CALIBRATION_STATUS_BAD_ARG;
    }

    app->calib_data = *persist;
    app->calib_loaded = 1u;
    if (hardwareinit_save_to_flash(app) != HARDWAREINIT_STATUS_OK)
    {
        return BALANCE_CALIBRATION_STATUS_FLASH_ERROR;
    }
    return BALANCE_CALIBRATION_STATUS_OK;
}

static app_serial_status_t app_serial_custom_cmd_cb(app_serial_t *serial, void *ctx, const char *line, uint8_t *handled)
{
    app_runtime_t *app;

    if ((serial == NULL) || (ctx == NULL) || (line == NULL) || (handled == NULL))
    {
        return APP_SERIAL_STATUS_BAD_ARG;
    }

    *handled = 0u;
    app = (app_runtime_t *)ctx;

    if (strcmp(line, "CALIB_START") == 0)
    {
        *handled = 1u;
        if (balance_calibration_is_active(&g_balance_cal) != 0u)
        {
            return app_serial_printf(serial, "ERR calib_busy\r\n");
        }
        if (balance_calibration_start(&g_balance_cal, HAL_GetTick()) != BALANCE_CALIBRATION_STATUS_OK)
        {
            return app_serial_printf(serial, "ERR calib_start_failed\r\n");
        }
        return app_serial_printf(serial, "RSP calib=start\r\n");
    }

    if (strcmp(line, "CALIB_ABORT") == 0)
    {
        *handled = 1u;
        balance_calibration_abort(&g_balance_cal);
        return app_serial_printf(serial, "RSP calib=abort_req\r\n");
    }

    if (strcmp(line, "CALIB_GET") == 0)
    {
        *handled = 1u;
        app_log_line_cb(app, "CALSTAT phase=GET_FLASH state=RUN");
        balance_calibration_emit_persist(&g_balance_cal, &app->calib_data);
        app_log_line_cb(app, "CALSTAT phase=GET_FLASH state=OK");
        return APP_SERIAL_STATUS_OK;
    }

    return APP_SERIAL_STATUS_EMPTY;
}

static void VerticalLedStep(const ekf_state_snapshot_t *state)
{
    int32_t pitch_mrad;
    int32_t rate_mrads;
    int32_t pitch_abs;
    int32_t rate_abs;

    if ((state == NULL) || (state->valid == 0u))
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        return;
    }

    pitch_mrad = (int32_t)(state->pitch_rad * 1000.0f);
    rate_mrads = (int32_t)(state->pitch_rate_rad_s * 1000.0f);
    pitch_abs = (pitch_mrad >= 0) ? pitch_mrad : -pitch_mrad;
    rate_abs = (rate_mrads >= 0) ? rate_mrads : -rate_mrads;

    if ((pitch_abs <= (int32_t)g_app.balance.params.vertical_pitch_thresh_mrad) &&
        (rate_abs <= (int32_t)g_app.balance.params.vertical_rate_thresh_mrads))
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

    now_ms = HAL_GetTick();
    dbg_period_ms = (g_app.control_enabled != 0u) ? 50u : 250u;
    if ((uint32_t)(now_ms - last_dbg_ms) < dbg_period_ms)
    {
        return;
    }
    last_dbg_ms = now_ms;

    app_debug_printf(
        "DBG[%s] t=%lu en=%u valid=%u mode=%s "
        "pitch=%ld pitchc=%ld target=%ld trim=%ld perr=%ld rate=%ld "
        "P=%ld D=%ld V=%ld X=%ld raw=%ld clamp=%ld sat=%u "
        "wv=%ld wd=%ld wp=%ld wpr=%ld "
        "u=%ld usync=%ld uturn=%ld L=%ld R=%ld "
        "fwd=%ld turn=%ld\r\n",

        PROJECT_BALANCE_DEBUG_REVISION,
        (unsigned long)now_ms,
        (unsigned)g_app.control_enabled,
        (unsigned)snap.valid,
        balance_mode_to_string(g_balance_out.mode),

        (long)(snap.pitch_rad * 1000.0f),
        (long)(g_balance_out.pitch_corr_rad * 1000.0f),
        (long)(g_balance_out.target_pitch_rad * 1000.0f),
        (long)(g_app.balance.state.target_trim_rad * 1000.0f),
        (long)(g_balance_out.pitch_error_rad * 1000.0f),
        (long)(snap.pitch_rate_rad_s * 1000.0f),

        (long)(g_balance_out.p_term * 1000.0f),
        (long)(g_balance_out.d_term * 1000.0f),
        (long)(g_balance_out.v_term * 1000.0f),
        (long)(g_balance_out.x_term * 1000.0f),
        (long)(g_balance_out.u_raw * 1000.0f),
        (long)(g_balance_out.u_base * 1000.0f),
        (unsigned)g_balance_out.saturated,

        (long)(snap.wheel_vel_avg * 1000.0f),
        (long)(snap.wheel_vel_diff * 1000.0f),
        (long)(snap.wheel_pos_avg * 1000.0f),
        (long)(g_balance_out.wheel_pos_rel * 1000.0f),

        (long)(g_balance_out.u_base * 1000.0f),
        (long)(g_balance_out.sync_term * 1000.0f),
        (long)(g_balance_out.turn_term * 1000.0f),
        (long)(g_balance_out.u_left * 1000.0f),
        (long)(g_balance_out.u_right * 1000.0f),

        (long)(g_balance_out.motion_fwd_cmd * 1000.0f),
        (long)(g_balance_out.motion_turn_cmd * 1000.0f)
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

    (void)odrive_pair_set_axis_state(&g_can_odrive, 8U, 8U);
    g_started = 1U;
}

void odrive_full_init(void)
{
    app_can_odrive_init();
    HAL_Delay(20);

    app_can_odrive_start_sequence();
    HAL_Delay(20);

    (void)odrive_pair_set_controller_mode(&g_can_odrive,
                                          ODRV_CONTROL_MODE_TORQUE_CONTROL,
                                          ODRV_INPUT_MODE_PASSTHROUGH,
                                          ODRV_CONTROL_MODE_TORQUE_CONTROL,
                                          ODRV_INPUT_MODE_PASSTHROUGH);
    HAL_Delay(20);

    (void)odrive_pair_set_input_torque(&g_can_odrive, 0.0f, 0.0f);
    HAL_Delay(20);

    (void)odrive_pair_set_axis_state(&g_can_odrive,
                                     ODRV_AXIS_STATE_CLOSED_LOOP_CONTROL,
                                     ODRV_AXIS_STATE_CLOSED_LOOP_CONTROL);
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
}

void EKF_init(void)
{
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

    if (ekf_driver_init(&g_ekf, &cfg) != EKF_STATUS_OK)
    {
        Error_Handler();
    }
}


static float apply_deadzone_slope_comp(float u_raw,
                                       float deadzone_pos,
                                       float deadzone_neg,
                                       float slope_pos,
                                       float slope_neg,
                                       float ref_slope_abs,
                                       float u_limit)
{
    float mag;
    float slope_abs;
    float deadzone;
    float slope_scale;

    if ((u_limit <= 0.0f) || (u_raw == 0.0f))
    {
        return 0.0f;
    }

    if (u_raw > 0.0f)
    {
        deadzone = deadzone_pos;
        slope_abs = fabsf(slope_pos);
    }
    else
    {
        deadzone = deadzone_neg;
        slope_abs = fabsf(slope_neg);
    }

    if (slope_abs < 1.0e-6f)
    {
        slope_abs = ref_slope_abs;
    }

    if (ref_slope_abs < 1.0e-6f)
    {
        ref_slope_abs = slope_abs;
    }

    slope_scale = ref_slope_abs / slope_abs;
    mag = fabsf(u_raw) * slope_scale;

    if (mag < 1.0e-4f)
    {
        return 0.0f;
    }

    mag += deadzone;

    if (mag > u_limit)
    {
        mag = u_limit;
    }

    return (u_raw > 0.0f) ? mag : -mag;
}

static void apply_balance_output_compensation(balance_output_t *out,
                                              const balance_calibration_persist_t *cal)
{
    float u_limit;
    float left_ref;
    float right_ref;
    float ref_slope_abs;

    if ((out == NULL) || (cal == NULL))
    {
        return;
    }

    if ((cal->flags & BALANCE_CALIBRATION_FLAG_VALID) == 0u)
    {
        return;
    }

    if (out->mode == BALANCE_MODE_CATCH)
    {
        return;
    }

    if (out->mode != BALANCE_MODE_BALANCE)
    {
        return;
    }

    u_limit = g_app.balance.params.control_u_limit;

    left_ref = 0.5f * (fabsf(cal->l_slope_pos) + fabsf(cal->l_slope_neg));
    right_ref = 0.5f * (fabsf(cal->r_slope_pos) + fabsf(cal->r_slope_neg));
    ref_slope_abs = 0.5f * (left_ref + right_ref);

    out->u_left = apply_deadzone_slope_comp(out->u_left,
                                            cal->l_deadzone_pos,
                                            cal->l_deadzone_neg,
                                            cal->l_slope_pos,
                                            cal->l_slope_neg,
                                            ref_slope_abs,
                                            u_limit);

    out->u_right = apply_deadzone_slope_comp(out->u_right,
                                             cal->r_deadzone_pos,
                                             cal->r_deadzone_neg,
                                             cal->r_slope_pos,
                                             cal->r_slope_neg,
                                             ref_slope_abs,
                                             u_limit);
}

static void ControlTorqueStep(void)
{
    balance_input_t in;

    memset(&in, 0, sizeof(in));
    in.pitch_rad = snap.pitch_rad;
    in.pitch_rate_rad_s = snap.pitch_rate_rad_s;
    in.wheel_vel_avg = snap.wheel_vel_avg;
    in.wheel_pos_avg = snap.wheel_pos_avg;
    in.wheel_vel_diff = snap.wheel_vel_diff;
    in.valid = snap.valid;
    in.control_enabled = g_app.control_enabled;
    in.now_ms = HAL_GetTick();

    if (balance_calibration_is_active(&g_balance_cal) != 0u)
    {
        return;
    }

    if (balance_step(&g_app.balance, &in, &g_app.command, &g_balance_out) != BALANCE_STATUS_OK)
    {
        (void)odrive_pair_set_input_torque(&g_can_odrive, 0.0f, 0.0f);
        return;
    }

    if ((g_balance_out.mode == BALANCE_MODE_IDLE) || (g_balance_out.valid == 0u))
    {
        (void)odrive_pair_set_input_torque(&g_can_odrive, 0.0f, 0.0f);
        return;
    }

    apply_balance_output_compensation(&g_balance_out, &g_app.calib_data);

    (void)odrive_pair_set_input_torque(&g_can_odrive, g_balance_out.u_left, g_balance_out.u_right);
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

  (void)balance_reset(&g_app.balance, 0.0f, HAL_GetTick());

  imu_init();

  odrive_full_init();

  EKF_init();

  g_app.serial.custom_cmd = app_serial_custom_cmd_cb;
  g_app.serial.custom_ctx = &g_app;

  balance_calibration_init(&g_balance_cal, NULL);
  g_balance_cal.io.user_ctx = &g_app;
  g_balance_cal.io.get_left_vel = app_get_left_vel_cb;
  g_balance_cal.io.get_right_vel = app_get_right_vel_cb;
  g_balance_cal.io.set_torque = app_set_pair_torque_cb;
  g_balance_cal.io.are_axes_ready = app_are_axes_ready_cb;
  g_balance_cal.io.has_errors = app_has_errors_cb;
  g_balance_cal.io.is_control_enabled = app_is_control_enabled_cb;
  g_balance_cal.io.set_control_enabled = app_set_control_enabled_cb;
  g_balance_cal.io.log_line = app_log_line_cb;
  g_balance_cal.io.save_persist = app_save_calibration_cb;
  balance_calibration_set_persist(&g_balance_cal, &g_app.calib_data);

  HAL_TIM_Base_Start_IT(&htim11);
  (void)app_serial_printf(&g_app.serial, "RSP ready\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    ICM20948_Service(&imu);

	    can_mcp2515_odrive_process(&g_can_odrive);

	    if (balance_calibration_is_active(&g_balance_cal) != 0u)
	    {
	        uint32_t now_ms = HAL_GetTick();
	        if ((uint32_t)(now_ms - g_last_cal_encoder_req_ms) >= 20u)
	        {
	            can_mcp2515_odrive_status_t enc_st = odrive_pair_request_encoder_estimates(&g_can_odrive);
	            if ((enc_st == CAN_MCP2515_ODRIVE_STATUS_OK) || (enc_st == CAN_MCP2515_ODRIVE_STATUS_BUSY))
	            {
	                g_last_cal_encoder_req_ms = now_ms;
	            }
	        }
	    }

	    ekf_driver_update_inputs_from_drivers(&g_ekf, &imu, &g_can_odrive);
	    ekf_driver_process_pending(&g_ekf);
	    ekf_driver_get_state_snapshot(&g_ekf, &snap);
	    VerticalLedStep(&snap);
	    app_serial_process(&g_app.serial);

	    balance_calibration_process(&g_balance_cal, HAL_GetTick());


	    DebugTelemetryStep();

	    if (g_ctrl_step_pending != 0u)
	    {
	        g_ctrl_step_pending = 0u;
	        ControlTorqueStep();
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
	static uint8_t ctrl_div = 0u;
	uint64_t t_us;

	if (htim->Instance == TIM11)
	{
		t_us = icm_micros64(&imu);

		(void)ekf_driver_request_step_isr(&g_ekf, t_us);

		ctrl_div++;
		if (ctrl_div >= 5u)
		{
			ctrl_div = 0u;
			g_ctrl_step_pending = 1u;
			g_ctrl_step_ts_us = t_us;
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
