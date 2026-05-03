#ifndef APP_SERIAL_H
#define APP_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#define APP_SERIAL_MODULE_VERSION        "S06"

#include "main.h"
#include <stdint.h>
#include <stdarg.h>

// Версия: S06 03.05.26
// Назначение:
// USB CDC ASCII интерфейс для rover controller.
// Поддерживает параметры rover_drive и пользовательские команды drive/stop/get imu/get odrive.
// Оставлены только параметры rover controller.
//
// Команды:
// help
// get all
// get <name>
// set <name> <value>
// en 0|1
// drive <forward> <turn>
// stop

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
    APP_SERIAL_PARAM_MAX_VEL_REV_S = 0,
    APP_SERIAL_PARAM_MAX_TURN_REV_S,
    APP_SERIAL_PARAM_ACCEL_REV_S2,
    APP_SERIAL_PARAM_LEFT_DIR,
    APP_SERIAL_PARAM_RIGHT_DIR,
    APP_SERIAL_PARAM_CMD_TIMEOUT_MS,
    APP_SERIAL_PARAM_FORWARD_CMD,
    APP_SERIAL_PARAM_TURN_CMD,
    APP_SERIAL_PARAM_COUNT
} app_serial_param_id_t;

typedef struct app_serial_s app_serial_t;

typedef app_serial_status_t (*app_serial_get_param_fn)(void *ctx, app_serial_param_id_t id, float *value);
typedef app_serial_status_t (*app_serial_set_param_fn)(void *ctx, app_serial_param_id_t id, float value);
typedef app_serial_status_t (*app_serial_get_enable_fn)(void *ctx, uint8_t *enabled);
typedef app_serial_status_t (*app_serial_set_enable_fn)(void *ctx, uint8_t enabled);
typedef app_serial_status_t (*app_serial_custom_cmd_fn)(app_serial_t *serial, void *ctx, const char *line, uint8_t *handled);

#define APP_SERIAL_RX_FIFO_SIZE          256u
#define APP_SERIAL_LINE_SIZE             128u
#define APP_SERIAL_TX_LINE_SIZE          192u
#define APP_SERIAL_TOKEN_NAME_SIZE       48u

typedef struct app_serial_s
{
    app_serial_get_param_fn get_param;
    app_serial_set_param_fn set_param;
    app_serial_get_enable_fn get_enable;
    app_serial_set_enable_fn set_enable;
    void *user_ctx;
    app_serial_custom_cmd_fn custom_cmd;
    void *custom_ctx;

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
