#ifndef APP_SERIAL_H
#define APP_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdarg.h>

// Версия: 01.01 29.03.26 13:10:00
// Назначение:
// Простой USB CDC интерфейс команд для настройки коэффициентов балансировки
// и команд движения. Модуль принимает ASCII строки, разбирает команды и
// отправляет ответы.
//
// Пример инициализации:
// app_runtime_t app;
// memset(&app, 0, sizeof(app));
// app.serial.get_param = hardwareinit_serial_get_param;
// app.serial.set_param = hardwareinit_serial_set_param;
// app.serial.get_enable = hardwareinit_serial_get_enable;
// app.serial.set_enable = hardwareinit_serial_set_enable;
// app.serial.user_ctx = &app;
// app_serial_init(&app.serial);
//
// Пример цикла main:
// while (1)
// {
//     app_serial_process(&app.serial);
// }
//
// Команды:
// help
// get all
// get balance_target_pitch_rad
// set control_k_wheel_pos 0.060
// set motion_fwd_cmd 0.200
// set motion_turn_cmd -0.150
// en 1

typedef enum
{
    APP_SERIAL_STATUS_OK = 0,
    APP_SERIAL_STATUS_BAD_ARG,
    APP_SERIAL_STATUS_BAD_CMD,
    APP_SERIAL_STATUS_BAD_PARAM,
    APP_SERIAL_STATUS_BAD_VALUE,
    APP_SERIAL_STATUS_FULL,
    APP_SERIAL_STATUS_EMPTY,
    APP_SERIAL_STATUS_BUSY,
    APP_SERIAL_STATUS_ERROR
} app_serial_status_t;

typedef enum
{
    APP_SERIAL_PARAM_CONTROL_U_LIMIT = 0,
    APP_SERIAL_PARAM_CONTROL_K_PITCH,
    APP_SERIAL_PARAM_CONTROL_K_PITCH_RATE,
    APP_SERIAL_PARAM_CONTROL_K_WHEEL_VEL,
    APP_SERIAL_PARAM_CONTROL_K_WHEEL_POS,
    APP_SERIAL_PARAM_CONTROL_K_SYNC,
    APP_SERIAL_PARAM_CONTROL_U_SYNC_LIMIT,
    APP_SERIAL_PARAM_VERTICAL_PITCH_THRESH_MRAD,
    APP_SERIAL_PARAM_VERTICAL_RATE_THRESH_MRADS,
    APP_SERIAL_PARAM_IMU_PITCH_ZERO_OFFSET_RAD,
    APP_SERIAL_PARAM_BALANCE_TARGET_PITCH_RAD,
    APP_SERIAL_PARAM_CATCH2BAL_PITCH_TH_RAD,
    APP_SERIAL_PARAM_CATCH2BAL_RATE_TH_RADS,
    APP_SERIAL_PARAM_BAL2CATCH_PITCH_TH_RAD,
    APP_SERIAL_PARAM_CATCH_HOLD_MS,
    APP_SERIAL_PARAM_CATCH_U_LIMIT,
    APP_SERIAL_PARAM_CATCH_K_PITCH,
    APP_SERIAL_PARAM_CATCH_K_PITCH_RATE,
    APP_SERIAL_PARAM_CATCH_K_WHEEL_VEL,
    APP_SERIAL_PARAM_CATCH_K_WHEEL_POS,
    APP_SERIAL_PARAM_FALL_PITCH_POS_TH_RAD,
    APP_SERIAL_PARAM_FALL_PITCH_NEG_TH_RAD,
    APP_SERIAL_PARAM_MOTION_PITCH_BIAS_PER_CMD_RAD,
    APP_SERIAL_PARAM_MOTION_CMD_RATE_PER_S,
    APP_SERIAL_PARAM_MOTION_TURN_U_LIMIT,
    APP_SERIAL_PARAM_MOTION_FWD_CMD,
    APP_SERIAL_PARAM_MOTION_TURN_CMD,
    APP_SERIAL_PARAM_COUNT
} app_serial_param_id_t;

typedef struct app_serial_s app_serial_t;

typedef app_serial_status_t (*app_serial_get_param_fn)(void *ctx, app_serial_param_id_t id, float *value);
typedef app_serial_status_t (*app_serial_set_param_fn)(void *ctx, app_serial_param_id_t id, float value);
typedef app_serial_status_t (*app_serial_get_enable_fn)(void *ctx, uint8_t *enabled);
typedef app_serial_status_t (*app_serial_set_enable_fn)(void *ctx, uint8_t enabled);

#define APP_SERIAL_RX_FIFO_SIZE          256u
#define APP_SERIAL_LINE_SIZE             128u
#define APP_SERIAL_TX_LINE_SIZE          160u
#define APP_SERIAL_TOKEN_NAME_SIZE       48u

typedef struct app_serial_s
{
    app_serial_get_param_fn get_param;
    app_serial_set_param_fn set_param;
    app_serial_get_enable_fn get_enable;
    app_serial_set_enable_fn set_enable;
    void *user_ctx;

    volatile uint16_t rx_wr;
    volatile uint16_t rx_rd;
    uint8_t rx_fifo[APP_SERIAL_RX_FIFO_SIZE];

    uint16_t line_len;
    char line_buf[APP_SERIAL_LINE_SIZE];

    uint8_t dump_active;
    uint8_t tx_busy;
    uint8_t reserved0;
    uint8_t reserved1;
    uint32_t dump_index;

    char tx_line[APP_SERIAL_TX_LINE_SIZE];
} app_serial_t;

app_serial_status_t app_serial_init(app_serial_t *serial);
app_serial_status_t app_serial_rx_bytes(app_serial_t *serial, const uint8_t *data, uint32_t len);
app_serial_status_t app_serial_process(app_serial_t *serial);
app_serial_status_t app_serial_printf(app_serial_t *serial, const char *fmt, ...);
void app_serial_cdc_set_target(app_serial_t *serial);
void app_serial_cdc_rx_callback(uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif
