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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    CONTROL_MODE_IDLE = 0,
    CONTROL_MODE_CATCH,
    CONTROL_MODE_BALANCE
} control_mode_t;

typedef struct
{
    float control_u_limit;
    float control_k_pitch;
    float control_k_pitch_rate;
    float control_k_sync;
    float control_u_sync_limit;
    float vertical_pitch_thresh_mrad;
    float vertical_rate_thresh_mrads;
    float control_k_wheel_vel;
    float control_pitch_trim_rad;
    float catch2bal_pitch_th_rad;
    float catch2bal_rate_th_rads;
    float bal2catch_pitch_th_rad;
    float catch_hold_ms;
    float catch_u_limit;
    float catch_k_pitch;
    float catch_k_pitch_rate;
    float catch_k_wheel_vel;
    float fall_pitch_pos_th_rad;
    float fall_pitch_neg_th_rad;
} balance_runtime_cfg_t;

typedef struct
{
    balance_runtime_cfg_t balance;
    uint8_t control_enabled;
    uint8_t flash_cfg_loaded;
    uint8_t reserved0;
    uint8_t reserved1;
    flash_cfg_store_t flash_store;
    app_serial_t serial;
} app_runtime_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ODRV_AXIS_STATE_IDLE                  1U
#define ODRV_AXIS_STATE_CLOSED_LOOP_CONTROL   8U
#define ODRV_CONTROL_MODE_TORQUE_CONTROL      1U
#define ODRV_INPUT_MODE_PASSTHROUGH           1U

#define BAL_CFG_CONTROL_U_LIMIT_DEFAULT             0.40f
#define BAL_CFG_CONTROL_K_PITCH_DEFAULT             0.80f
#define BAL_CFG_CONTROL_K_PITCH_RATE_DEFAULT        0.45f
#define BAL_CFG_CONTROL_K_SYNC_DEFAULT              0.06f
#define BAL_CFG_CONTROL_U_SYNC_LIMIT_DEFAULT        0.03f
#define BAL_CFG_VERTICAL_PITCH_THRESH_MRAD_DEFAULT  80.0f
#define BAL_CFG_VERTICAL_RATE_THRESH_MRADS_DEFAULT  200.0f
#define BAL_CFG_CONTROL_K_WHEEL_VEL_DEFAULT         0.20f
#define BAL_CFG_CONTROL_PITCH_TRIM_RAD_DEFAULT      0.20f
#define BAL_CFG_CATCH2BAL_PITCH_TH_RAD_DEFAULT      0.05f
#define BAL_CFG_CATCH2BAL_RATE_TH_RADS_DEFAULT      0.50f
#define BAL_CFG_BAL2CATCH_PITCH_TH_RAD_DEFAULT      0.16f
#define BAL_CFG_CATCH_HOLD_MS_DEFAULT               300.0f
#define BAL_CFG_CATCH_U_LIMIT_DEFAULT               0.45f
#define BAL_CFG_CATCH_K_PITCH_DEFAULT               0.52f
#define BAL_CFG_CATCH_K_PITCH_RATE_DEFAULT          0.45f
#define BAL_CFG_CATCH_K_WHEEL_VEL_DEFAULT           0.22f
#define BAL_CFG_FALL_PITCH_POS_TH_RAD_DEFAULT       (750.0f / 1000.0f)
#define BAL_CFG_FALL_PITCH_NEG_TH_RAD_DEFAULT       (-1200.0f / 1000.0f)
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

static volatile uint8_t g_ctrl_step_pending = 0u;
static volatile uint64_t g_ctrl_step_ts_us = 0ull;

static float g_torque_cmd = 0.0f;
static float g_left_cmd = 0.0f;
static float g_right_cmd = 0.0f;
static float g_u_sync = 0.0f;
static float g_u_wheel_vel = 0.0f;
static float g_vel1_raw = 0.0f;
static float g_vel2_raw = 0.0f;
static float g_vel1_norm = 0.0f;
static float g_vel2_norm = 0.0f;
static float g_wheel_vel_diff_raw = 0.0f;
static control_mode_t g_control_mode = CONTROL_MODE_IDLE;
static uint32_t g_control_mode_enter_ms = 0u;
static uint32_t g_catch_in_band_since_ms = 0u;
static uint8_t g_catch_in_band_active = 0u;
static float g_wheel_pos_ref = 0.0f;

static can_mcp2515_odrive_t g_can_odrive;
static app_runtime_t g_app;

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
static const char *ControlModeToString(control_mode_t mode);
static void ControlSetMode(control_mode_t mode, uint32_t now_ms);
static void ControlUpdateMode(uint32_t now_ms, float pitch_err, float pitch_rate);
static void ControlResetOutputs(void);
static void AppRuntimeInit(void);
static void AppRuntimeToFlashData(const app_runtime_t *app, flash_cfg_balance_data_t *data);
static void AppRuntimeFromFlashData(app_runtime_t *app, const flash_cfg_balance_data_t *data);
static app_serial_status_t AppRuntimeSaveToFlash(app_runtime_t *app);
static app_serial_status_t AppSerialGetParam(void *ctx, app_serial_param_id_t id, float *value);
static app_serial_status_t AppSerialSetParam(void *ctx, app_serial_param_id_t id, float value);
static app_serial_status_t AppSerialGetEnable(void *ctx, uint8_t *enabled);
static app_serial_status_t AppSerialSetEnable(void *ctx, uint8_t enabled);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static const char *ControlModeToString(control_mode_t mode)
{
    switch (mode)
    {
    case CONTROL_MODE_IDLE:
        return "IDLE";

    case CONTROL_MODE_CATCH:
        return "CATCH";

    case CONTROL_MODE_BALANCE:
        return "BAL";

    default:
        return "UNK";
    }
}

static void ControlSetMode(control_mode_t mode, uint32_t now_ms)
{
    if (g_control_mode == mode)
    {
        return;
    }

    g_control_mode = mode;
    g_control_mode_enter_ms = now_ms;

    if (mode != CONTROL_MODE_CATCH)
    {
        g_catch_in_band_since_ms = 0u;
        g_catch_in_band_active = 0u;
    }
}

static void ControlUpdateMode(uint32_t now_ms, float pitch_err, float pitch_rate)
{
    if ((g_app.control_enabled == 0u) || (snap.valid == 0u))
    {
        ControlSetMode(CONTROL_MODE_IDLE, now_ms);
        return;
    }

    switch (g_control_mode)
    {
    case CONTROL_MODE_IDLE:
        ControlSetMode(CONTROL_MODE_CATCH, now_ms);
        break;

    case CONTROL_MODE_CATCH:
    {
        uint8_t in_band;

        in_band = 0u;
        if ((fabsf(pitch_err) < g_app.balance.catch2bal_pitch_th_rad) &&
            (fabsf(pitch_rate) < g_app.balance.catch2bal_rate_th_rads))
        {
            in_band = 1u;
        }

        if (in_band != 0u)
        {
            if (g_catch_in_band_active == 0u)
            {
                g_catch_in_band_active = 1u;
                g_catch_in_band_since_ms = now_ms;
            }

            if ((float)((uint32_t)(now_ms - g_catch_in_band_since_ms)) >= g_app.balance.catch_hold_ms)
            {
                ControlSetMode(CONTROL_MODE_BALANCE, now_ms);
            }
        }
        else
        {
            g_catch_in_band_active = 0u;
            g_catch_in_band_since_ms = 0u;
        }
        break;
    }

    case CONTROL_MODE_BALANCE:
        if (fabsf(pitch_err) > g_app.balance.bal2catch_pitch_th_rad)
        {
            ControlSetMode(CONTROL_MODE_CATCH, now_ms);
        }
        break;

    default:
        ControlSetMode(CONTROL_MODE_IDLE, now_ms);
        break;
    }
}

static void ControlResetOutputs(void)
{
    g_torque_cmd = 0.0f;
    g_u_sync = 0.0f;
    g_left_cmd = 0.0f;
    g_right_cmd = 0.0f;
    g_u_wheel_vel = 0.0f;
    g_vel1_raw = 0.0f;
    g_vel2_raw = 0.0f;
    g_vel1_norm = 0.0f;
    g_vel2_norm = 0.0f;
    g_wheel_vel_diff_raw = 0.0f;
}

static void AppRuntimeToFlashData(const app_runtime_t *app, flash_cfg_balance_data_t *data)
{
    if ((app == NULL) || (data == NULL))
    {
        return;
    }

    data->control_u_limit = app->balance.control_u_limit;
    data->control_k_pitch = app->balance.control_k_pitch;
    data->control_k_pitch_rate = app->balance.control_k_pitch_rate;
    data->control_k_sync = app->balance.control_k_sync;
    data->control_u_sync_limit = app->balance.control_u_sync_limit;
    data->vertical_pitch_thresh_mrad = app->balance.vertical_pitch_thresh_mrad;
    data->vertical_rate_thresh_mrads = app->balance.vertical_rate_thresh_mrads;
    data->control_k_wheel_vel = app->balance.control_k_wheel_vel;
    data->control_pitch_trim_rad = app->balance.control_pitch_trim_rad;
    data->catch2bal_pitch_th_rad = app->balance.catch2bal_pitch_th_rad;
    data->catch2bal_rate_th_rads = app->balance.catch2bal_rate_th_rads;
    data->bal2catch_pitch_th_rad = app->balance.bal2catch_pitch_th_rad;
    data->catch_hold_ms = app->balance.catch_hold_ms;
    data->catch_u_limit = app->balance.catch_u_limit;
    data->catch_k_pitch = app->balance.catch_k_pitch;
    data->catch_k_pitch_rate = app->balance.catch_k_pitch_rate;
    data->catch_k_wheel_vel = app->balance.catch_k_wheel_vel;
    data->fall_pitch_pos_th_rad = app->balance.fall_pitch_pos_th_rad;
    data->fall_pitch_neg_th_rad = app->balance.fall_pitch_neg_th_rad;
}

static void AppRuntimeFromFlashData(app_runtime_t *app, const flash_cfg_balance_data_t *data)
{
    if ((app == NULL) || (data == NULL))
    {
        return;
    }

    app->balance.control_u_limit = data->control_u_limit;
    app->balance.control_k_pitch = data->control_k_pitch;
    app->balance.control_k_pitch_rate = data->control_k_pitch_rate;
    app->balance.control_k_sync = data->control_k_sync;
    app->balance.control_u_sync_limit = data->control_u_sync_limit;
    app->balance.vertical_pitch_thresh_mrad = data->vertical_pitch_thresh_mrad;
    app->balance.vertical_rate_thresh_mrads = data->vertical_rate_thresh_mrads;
    app->balance.control_k_wheel_vel = data->control_k_wheel_vel;
    app->balance.control_pitch_trim_rad = data->control_pitch_trim_rad;
    app->balance.catch2bal_pitch_th_rad = data->catch2bal_pitch_th_rad;
    app->balance.catch2bal_rate_th_rads = data->catch2bal_rate_th_rads;
    app->balance.bal2catch_pitch_th_rad = data->bal2catch_pitch_th_rad;
    app->balance.catch_hold_ms = data->catch_hold_ms;
    app->balance.catch_u_limit = data->catch_u_limit;
    app->balance.catch_k_pitch = data->catch_k_pitch;
    app->balance.catch_k_pitch_rate = data->catch_k_pitch_rate;
    app->balance.catch_k_wheel_vel = data->catch_k_wheel_vel;
    app->balance.fall_pitch_pos_th_rad = data->fall_pitch_pos_th_rad;
    app->balance.fall_pitch_neg_th_rad = data->fall_pitch_neg_th_rad;
}

static app_serial_status_t AppRuntimeSaveToFlash(app_runtime_t *app)
{
    flash_cfg_balance_data_t data;

    if (app == NULL)
    {
        return APP_SERIAL_STATUS_BAD_ARG;
    }

    AppRuntimeToFlashData(app, &data);
    if (flash_cfg_store_save(&app->flash_store, &data) != FLASH_CFG_STORE_STATUS_OK)
    {
        return APP_SERIAL_STATUS_ERROR;
    }

    return APP_SERIAL_STATUS_OK;
}

static void AppRuntimeInit(void)
{
    flash_cfg_balance_data_t flash_data;

    memset(&g_app, 0, sizeof(g_app));

    g_app.balance.control_u_limit = BAL_CFG_CONTROL_U_LIMIT_DEFAULT;
    g_app.balance.control_k_pitch = BAL_CFG_CONTROL_K_PITCH_DEFAULT;
    g_app.balance.control_k_pitch_rate = BAL_CFG_CONTROL_K_PITCH_RATE_DEFAULT;
    g_app.balance.control_k_sync = BAL_CFG_CONTROL_K_SYNC_DEFAULT;
    g_app.balance.control_u_sync_limit = BAL_CFG_CONTROL_U_SYNC_LIMIT_DEFAULT;
    g_app.balance.vertical_pitch_thresh_mrad = BAL_CFG_VERTICAL_PITCH_THRESH_MRAD_DEFAULT;
    g_app.balance.vertical_rate_thresh_mrads = BAL_CFG_VERTICAL_RATE_THRESH_MRADS_DEFAULT;
    g_app.balance.control_k_wheel_vel = BAL_CFG_CONTROL_K_WHEEL_VEL_DEFAULT;
    g_app.balance.control_pitch_trim_rad = BAL_CFG_CONTROL_PITCH_TRIM_RAD_DEFAULT;
    g_app.balance.catch2bal_pitch_th_rad = BAL_CFG_CATCH2BAL_PITCH_TH_RAD_DEFAULT;
    g_app.balance.catch2bal_rate_th_rads = BAL_CFG_CATCH2BAL_RATE_TH_RADS_DEFAULT;
    g_app.balance.bal2catch_pitch_th_rad = BAL_CFG_BAL2CATCH_PITCH_TH_RAD_DEFAULT;
    g_app.balance.catch_hold_ms = BAL_CFG_CATCH_HOLD_MS_DEFAULT;
    g_app.balance.catch_u_limit = BAL_CFG_CATCH_U_LIMIT_DEFAULT;
    g_app.balance.catch_k_pitch = BAL_CFG_CATCH_K_PITCH_DEFAULT;
    g_app.balance.catch_k_pitch_rate = BAL_CFG_CATCH_K_PITCH_RATE_DEFAULT;
    g_app.balance.catch_k_wheel_vel = BAL_CFG_CATCH_K_WHEEL_VEL_DEFAULT;
    g_app.balance.fall_pitch_pos_th_rad = BAL_CFG_FALL_PITCH_POS_TH_RAD_DEFAULT;
    g_app.balance.fall_pitch_neg_th_rad = BAL_CFG_FALL_PITCH_NEG_TH_RAD_DEFAULT;
    g_app.control_enabled = 0u;

    (void)flash_cfg_store_init(&g_app.flash_store);
    if (flash_cfg_store_load(&g_app.flash_store, &flash_data) == FLASH_CFG_STORE_STATUS_OK)
    {
        AppRuntimeFromFlashData(&g_app, &flash_data);
        g_app.flash_cfg_loaded = 1u;
    }

    g_app.serial.get_param = AppSerialGetParam;
    g_app.serial.set_param = AppSerialSetParam;
    g_app.serial.get_enable = AppSerialGetEnable;
    g_app.serial.set_enable = AppSerialSetEnable;
    g_app.serial.user_ctx = &g_app;
    (void)app_serial_init(&g_app.serial);
}

static app_serial_status_t AppSerialGetParam(void *ctx, app_serial_param_id_t id, float *value)
{
    app_runtime_t *app;

    if ((ctx == NULL) || (value == NULL))
    {
        return APP_SERIAL_STATUS_BAD_ARG;
    }

    app = (app_runtime_t *)ctx;

    switch (id)
    {
    case APP_SERIAL_PARAM_CONTROL_U_LIMIT: *value = app->balance.control_u_limit; break;
    case APP_SERIAL_PARAM_CONTROL_K_PITCH: *value = app->balance.control_k_pitch; break;
    case APP_SERIAL_PARAM_CONTROL_K_PITCH_RATE: *value = app->balance.control_k_pitch_rate; break;
    case APP_SERIAL_PARAM_CONTROL_K_SYNC: *value = app->balance.control_k_sync; break;
    case APP_SERIAL_PARAM_CONTROL_U_SYNC_LIMIT: *value = app->balance.control_u_sync_limit; break;
    case APP_SERIAL_PARAM_VERTICAL_PITCH_THRESH_MRAD: *value = app->balance.vertical_pitch_thresh_mrad; break;
    case APP_SERIAL_PARAM_VERTICAL_RATE_THRESH_MRADS: *value = app->balance.vertical_rate_thresh_mrads; break;
    case APP_SERIAL_PARAM_CONTROL_K_WHEEL_VEL: *value = app->balance.control_k_wheel_vel; break;
    case APP_SERIAL_PARAM_CONTROL_PITCH_TRIM_RAD: *value = app->balance.control_pitch_trim_rad; break;
    case APP_SERIAL_PARAM_CATCH2BAL_PITCH_TH_RAD: *value = app->balance.catch2bal_pitch_th_rad; break;
    case APP_SERIAL_PARAM_CATCH2BAL_RATE_TH_RADS: *value = app->balance.catch2bal_rate_th_rads; break;
    case APP_SERIAL_PARAM_BAL2CATCH_PITCH_TH_RAD: *value = app->balance.bal2catch_pitch_th_rad; break;
    case APP_SERIAL_PARAM_CATCH_HOLD_MS: *value = app->balance.catch_hold_ms; break;
    case APP_SERIAL_PARAM_CATCH_U_LIMIT: *value = app->balance.catch_u_limit; break;
    case APP_SERIAL_PARAM_CATCH_K_PITCH: *value = app->balance.catch_k_pitch; break;
    case APP_SERIAL_PARAM_CATCH_K_PITCH_RATE: *value = app->balance.catch_k_pitch_rate; break;
    case APP_SERIAL_PARAM_CATCH_K_WHEEL_VEL: *value = app->balance.catch_k_wheel_vel; break;
    case APP_SERIAL_PARAM_FALL_PITCH_POS_TH_RAD: *value = app->balance.fall_pitch_pos_th_rad; break;
    case APP_SERIAL_PARAM_FALL_PITCH_NEG_TH_RAD: *value = app->balance.fall_pitch_neg_th_rad; break;
    default:
        return APP_SERIAL_STATUS_BAD_PARAM;
    }

    return APP_SERIAL_STATUS_OK;
}

static app_serial_status_t AppSerialSetParam(void *ctx, app_serial_param_id_t id, float value)
{
    app_runtime_t *app;
    balance_runtime_cfg_t prev_cfg;

    if (ctx == NULL)
    {
        return APP_SERIAL_STATUS_BAD_ARG;
    }

    app = (app_runtime_t *)ctx;
    prev_cfg = app->balance;

    switch (id)
    {
    case APP_SERIAL_PARAM_CONTROL_U_LIMIT: app->balance.control_u_limit = value; break;
    case APP_SERIAL_PARAM_CONTROL_K_PITCH: app->balance.control_k_pitch = value; break;
    case APP_SERIAL_PARAM_CONTROL_K_PITCH_RATE: app->balance.control_k_pitch_rate = value; break;
    case APP_SERIAL_PARAM_CONTROL_K_SYNC: app->balance.control_k_sync = value; break;
    case APP_SERIAL_PARAM_CONTROL_U_SYNC_LIMIT: app->balance.control_u_sync_limit = value; break;
    case APP_SERIAL_PARAM_VERTICAL_PITCH_THRESH_MRAD: app->balance.vertical_pitch_thresh_mrad = value; break;
    case APP_SERIAL_PARAM_VERTICAL_RATE_THRESH_MRADS: app->balance.vertical_rate_thresh_mrads = value; break;
    case APP_SERIAL_PARAM_CONTROL_K_WHEEL_VEL: app->balance.control_k_wheel_vel = value; break;
    case APP_SERIAL_PARAM_CONTROL_PITCH_TRIM_RAD: app->balance.control_pitch_trim_rad = value; break;
    case APP_SERIAL_PARAM_CATCH2BAL_PITCH_TH_RAD: app->balance.catch2bal_pitch_th_rad = value; break;
    case APP_SERIAL_PARAM_CATCH2BAL_RATE_TH_RADS: app->balance.catch2bal_rate_th_rads = value; break;
    case APP_SERIAL_PARAM_BAL2CATCH_PITCH_TH_RAD: app->balance.bal2catch_pitch_th_rad = value; break;
    case APP_SERIAL_PARAM_CATCH_HOLD_MS:
        if (value < 0.0f)
        {
            return APP_SERIAL_STATUS_BAD_VALUE;
        }
        app->balance.catch_hold_ms = value;
        break;
    case APP_SERIAL_PARAM_CATCH_U_LIMIT: app->balance.catch_u_limit = value; break;
    case APP_SERIAL_PARAM_CATCH_K_PITCH: app->balance.catch_k_pitch = value; break;
    case APP_SERIAL_PARAM_CATCH_K_PITCH_RATE: app->balance.catch_k_pitch_rate = value; break;
    case APP_SERIAL_PARAM_CATCH_K_WHEEL_VEL: app->balance.catch_k_wheel_vel = value; break;
    case APP_SERIAL_PARAM_FALL_PITCH_POS_TH_RAD: app->balance.fall_pitch_pos_th_rad = value; break;
    case APP_SERIAL_PARAM_FALL_PITCH_NEG_TH_RAD: app->balance.fall_pitch_neg_th_rad = value; break;
    default:
        return APP_SERIAL_STATUS_BAD_PARAM;
    }

    if (AppRuntimeSaveToFlash(app) != APP_SERIAL_STATUS_OK)
    {
        app->balance = prev_cfg;
        return APP_SERIAL_STATUS_ERROR;
    }

    return APP_SERIAL_STATUS_OK;
}

static app_serial_status_t AppSerialGetEnable(void *ctx, uint8_t *enabled)
{
    app_runtime_t *app;

    if ((ctx == NULL) || (enabled == NULL))
    {
        return APP_SERIAL_STATUS_BAD_ARG;
    }

    app = (app_runtime_t *)ctx;
    *enabled = app->control_enabled;
    return APP_SERIAL_STATUS_OK;
}

static app_serial_status_t AppSerialSetEnable(void *ctx, uint8_t enabled)
{
    app_runtime_t *app;

    if (ctx == NULL)
    {
        return APP_SERIAL_STATUS_BAD_ARG;
    }

    app = (app_runtime_t *)ctx;
    app->control_enabled = (enabled != 0u) ? 1u : 0u;
    return APP_SERIAL_STATUS_OK;
}

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

    (void)CDC_Transmit_FS((uint8_t *)buf, (uint16_t)n);
}

static void app_debug_printf(const char *fmt, ...)
{
    char buf[256];
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

static float control_clamp(float v, float vmin, float vmax)
{
    if (v < vmin)
    {
        return vmin;
    }
    if (v > vmax)
    {
        return vmax;
    }
    return v;
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

    if ((pitch_abs <= g_app.balance.vertical_pitch_thresh_mrad) &&
        (rate_abs <= g_app.balance.vertical_rate_thresh_mrads))
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
    float pitch_err;
    float p_term;
    float d_term;
    float v_term;
    float u_raw;
    float u_clamped;
    float kp_dbg;
    float kd_dbg;
    float ks_dbg;
    float wp;
    float wpr;

    int32_t pitch_mrad;
    int32_t rate_mrads;
    int32_t wv_x1000;
    int32_t wd_x1000;
    int32_t wp_x1000;
    int32_t wpr_x1000;

    int32_t p_x1000;
    int32_t d_x1000;
    int32_t u_pd_raw_x1000;
    int32_t u_pd_clamped_x1000;
    int32_t vterm_x1000;

    int32_t u_x1000;
    int32_t usync_x1000;
    int32_t l_x1000;
    int32_t r_x1000;

    int32_t v1raw_x1000;
    int32_t v2raw_x1000;
    int32_t v1n_x1000;
    int32_t v2n_x1000;
    int32_t wdraw_x1000;

    uint8_t sat_flag = 0u;

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - last_dbg_ms) < 100u)
    {
        return;
    }
    last_dbg_ms = now_ms;

    pitch_mrad = (int32_t)(snap.pitch_rad * 1000.0f);
    rate_mrads = (int32_t)(snap.pitch_rate_rad_s * 1000.0f);
    wv_x1000 = (int32_t)(snap.wheel_vel_avg * 1000.0f);
    wd_x1000 = (int32_t)(snap.wheel_vel_diff * 1000.0f);
    wp = snap.wheel_pos_avg;
    wpr = snap.wheel_pos_avg - g_wheel_pos_ref;
    wp_x1000 = (int32_t)(wp * 1000.0f);
    wpr_x1000 = (int32_t)(wpr * 1000.0f);

    pitch_err = snap.pitch_rad + g_app.balance.control_pitch_trim_rad;

    if (g_control_mode == CONTROL_MODE_CATCH)
    {
        p_term = g_app.balance.catch_k_pitch * pitch_err;
        d_term = g_app.balance.catch_k_pitch_rate * snap.pitch_rate_rad_s;
        v_term = -(g_app.balance.catch_k_wheel_vel * snap.wheel_vel_avg);
        u_raw = p_term + d_term + v_term;
        u_clamped = control_clamp(u_raw, -g_app.balance.catch_u_limit, +g_app.balance.catch_u_limit);
        kp_dbg = g_app.balance.catch_k_pitch;
        kd_dbg = g_app.balance.catch_k_pitch_rate;
        ks_dbg = 0.0f;
    }
    else if (g_control_mode == CONTROL_MODE_BALANCE)
    {
        p_term = g_app.balance.control_k_pitch * pitch_err;
        d_term = g_app.balance.control_k_pitch_rate * snap.pitch_rate_rad_s;
        v_term = -(g_app.balance.control_k_wheel_vel * snap.wheel_vel_avg);
        u_raw = p_term + d_term + v_term;
        u_clamped = control_clamp(u_raw,
                                  -(g_app.balance.control_u_limit - g_app.balance.control_u_sync_limit),
                                  +(g_app.balance.control_u_limit - g_app.balance.control_u_sync_limit));
        kp_dbg = g_app.balance.control_k_pitch;
        kd_dbg = g_app.balance.control_k_pitch_rate;
        ks_dbg = g_app.balance.control_k_sync;
    }
    else
    {
        p_term = 0.0f;
        d_term = 0.0f;
        v_term = 0.0f;
        u_raw = 0.0f;
        u_clamped = 0.0f;
        kp_dbg = 0.0f;
        kd_dbg = 0.0f;
        ks_dbg = 0.0f;
    }

    p_x1000 = (int32_t)(p_term * 1000.0f);
    d_x1000 = (int32_t)(d_term * 1000.0f);
    u_pd_raw_x1000 = (int32_t)(u_raw * 1000.0f);
    u_pd_clamped_x1000 = (int32_t)(u_clamped * 1000.0f);
    vterm_x1000 = (int32_t)(v_term * 1000.0f);

    if ((u_raw > u_clamped + 1e-6f) ||
        (u_raw < u_clamped - 1e-6f))
    {
        sat_flag = 1u;
    }

    u_x1000 = (int32_t)(g_torque_cmd * 1000.0f);
    usync_x1000 = (int32_t)(g_u_sync * 1000.0f);
    l_x1000 = (int32_t)(g_left_cmd * 1000.0f);
    r_x1000 = (int32_t)(g_right_cmd * 1000.0f);

    v1raw_x1000 = (int32_t)(g_vel1_raw * 1000.0f);
    v2raw_x1000 = (int32_t)(g_vel2_raw * 1000.0f);
    v1n_x1000 = (int32_t)(g_vel1_norm * 1000.0f);
    v2n_x1000 = (int32_t)(g_vel2_norm * 1000.0f);
    wdraw_x1000 = (int32_t)(g_wheel_vel_diff_raw * 1000.0f);

    app_debug_printf(
        "DBG t=%lu en=%u valid=%u mode=%s "
        "pitch=%ld rate=%ld "
        "P=%ld D=%ld V=%ld raw=%ld clamp=%ld sat=%u "
        "wv=%ld wd=%ld wp=%ld wpr=%ld "
        "v1=%ld v2=%ld v1n=%ld v2n=%ld wdr=%ld "
        "u=%ld usync=%ld L=%ld R=%ld "
        "Kp=%ld Kd=%ld Ks=%ld\r\n",

        (unsigned long)now_ms,
        (unsigned)g_app.control_enabled,
        (unsigned)snap.valid,
        ControlModeToString(g_control_mode),

        (long)pitch_mrad,
        (long)rate_mrads,

        (long)p_x1000,
        (long)d_x1000,
        (long)vterm_x1000,
        (long)u_pd_raw_x1000,
        (long)u_pd_clamped_x1000,
        (unsigned)sat_flag,

        (long)wv_x1000,
        (long)wd_x1000,
        (long)wp_x1000,
        (long)wpr_x1000,

        (long)v1raw_x1000,
        (long)v2raw_x1000,
        (long)v1n_x1000,
        (long)v2n_x1000,
        (long)wdraw_x1000,

        (long)u_x1000,
        (long)usync_x1000,
        (long)l_x1000,
        (long)r_x1000,

        (long)(kp_dbg * 1000.0f),
        (long)(kd_dbg * 1000.0f),
        (long)(ks_dbg * 1000.0f)
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


static void ControlTorqueStep(void)
{
    uint32_t now_ms;
    float pitch_err;
    float pitch_rate;
    float u_cmd;
    float u_sync;
    float left_cmd;
    float right_cmd;
    float vel1;
    float vel2;
    float wheel_vel_diff;

    now_ms = HAL_GetTick();

    if ((snap.pitch_rad > g_app.balance.fall_pitch_pos_th_rad) ||
        (snap.pitch_rad < g_app.balance.fall_pitch_neg_th_rad))
    {
        ControlSetMode(CONTROL_MODE_IDLE, now_ms);
        g_wheel_pos_ref = snap.wheel_pos_avg;
        ControlResetOutputs();
        (void)odrive_pair_set_input_torque(&g_can_odrive, 0.0f, 0.0f);
        return;
    }

    if ((g_app.control_enabled == 0u) || (snap.valid == 0u))
    {
        ControlSetMode(CONTROL_MODE_IDLE, now_ms);
        g_wheel_pos_ref = snap.wheel_pos_avg;
        ControlResetOutputs();
        (void)odrive_pair_set_input_torque(&g_can_odrive, 0.0f, 0.0f);
        return;
    }

    g_vel1_raw = g_can_odrive.pair.vel1;
    g_vel2_raw = g_can_odrive.pair.vel2;

    vel1 = g_vel1_raw;
    vel2 = -g_vel2_raw;

    vel1 = control_clamp(vel1, -1500.0f, 1500.0f);
    vel2 = control_clamp(vel2, -1500.0f, 1500.0f);

    wheel_vel_diff = snap.wheel_vel_diff;

    g_vel1_norm = vel1;
    g_vel2_norm = vel2;
    g_wheel_vel_diff_raw = wheel_vel_diff;

    pitch_err = snap.pitch_rad + g_app.balance.control_pitch_trim_rad;
    pitch_rate = snap.pitch_rate_rad_s;

    ControlUpdateMode(now_ms, pitch_err, pitch_rate);

    if (g_control_mode == CONTROL_MODE_IDLE)
    {
        g_wheel_pos_ref = snap.wheel_pos_avg;
        ControlResetOutputs();
        (void)odrive_pair_set_input_torque(&g_can_odrive, 0.0f, 0.0f);
        return;
    }

    if (g_control_mode == CONTROL_MODE_CATCH)
    {
        g_u_wheel_vel = -(g_app.balance.catch_k_wheel_vel * snap.wheel_vel_avg);

        u_cmd = g_app.balance.catch_k_pitch * pitch_err
              + g_app.balance.catch_k_pitch_rate * pitch_rate
              + g_u_wheel_vel;

        u_cmd = control_clamp(u_cmd, -g_app.balance.catch_u_limit, +g_app.balance.catch_u_limit);
        u_sync = 0.0f;

        left_cmd = u_cmd;
        right_cmd = -u_cmd;
    }
    else
    {
        g_u_wheel_vel = -(g_app.balance.control_k_wheel_vel * snap.wheel_vel_avg);

        u_cmd = (g_app.balance.control_k_pitch * pitch_err) +
                (g_app.balance.control_k_pitch_rate * pitch_rate) +
                g_u_wheel_vel;

        u_cmd = control_clamp(u_cmd,
                              -(g_app.balance.control_u_limit - g_app.balance.control_u_sync_limit),
                              +(g_app.balance.control_u_limit - g_app.balance.control_u_sync_limit));

        u_sync = -(g_app.balance.control_k_sync * wheel_vel_diff);
        u_sync = control_clamp(u_sync, -g_app.balance.control_u_sync_limit, g_app.balance.control_u_sync_limit);

        left_cmd = u_cmd + u_sync;
        right_cmd = -u_cmd - u_sync;

        left_cmd = control_clamp(left_cmd, -g_app.balance.control_u_limit, g_app.balance.control_u_limit);
        right_cmd = control_clamp(right_cmd, -g_app.balance.control_u_limit, g_app.balance.control_u_limit);
    }

    g_torque_cmd = u_cmd;
    g_u_sync = u_sync;
    g_left_cmd = left_cmd;
    g_right_cmd = right_cmd;

    (void)odrive_pair_set_input_torque(&g_can_odrive, left_cmd, right_cmd);
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

  AppRuntimeInit();

  imu_init();

  odrive_full_init();

  EKF_init();

  HAL_TIM_Base_Start_IT(&htim11);
  (void)app_serial_printf(&g_app.serial, "RSP ready\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    ICM20948_Service(&imu);

	    can_mcp2515_odrive_process(&g_can_odrive);

	    ekf_driver_update_inputs_from_drivers(&g_ekf, &imu, &g_can_odrive);
	    ekf_driver_process_pending(&g_ekf);
	    ekf_driver_get_state_snapshot(&g_ekf, &snap);
	    VerticalLedStep(&snap);
	    app_serial_process(&g_app.serial);
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
