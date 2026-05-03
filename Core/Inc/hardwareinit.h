#ifndef HARDWAREINIT_H
#define HARDWAREINIT_H

#ifdef __cplusplus
extern "C" {
#endif

#define HARDWAREINIT_MODULE_VERSION      "H06"

#include "main.h"
#include "rover_drive.h"
#include "flash_cfg_store.h"
#include "app_serial.h"
#include <stdint.h>

// Версия: H06 03.05.26
// Назначение:
// Runtime-структура rover controller, параметры rover_drive, Flash load/save,
// callbacks для USB serial.

typedef enum
{
    HARDWAREINIT_STATUS_OK = 0,
    HARDWAREINIT_STATUS_BAD_ARG,
    HARDWAREINIT_STATUS_FLASH_ERROR,
    HARDWAREINIT_STATUS_ROVER_ERROR,
    HARDWAREINIT_STATUS_SERIAL_ERROR
} hardwareinit_status_t;

typedef struct
{
    rover_drive_t drive;
    rover_drive_command_t command;
    uint8_t control_enabled;
    uint8_t flash_cfg_loaded;
    uint8_t reserved0;
    uint8_t reserved1;
    flash_cfg_store_t flash_store;
    app_serial_t serial;
} app_runtime_t;

hardwareinit_status_t hardwareinit_runtime_init(app_runtime_t *app);
hardwareinit_status_t hardwareinit_save_to_flash(app_runtime_t *app);
app_serial_status_t hardwareinit_serial_get_param(void *ctx, app_serial_param_id_t id, float *value);
app_serial_status_t hardwareinit_serial_set_param(void *ctx, app_serial_param_id_t id, float value);
app_serial_status_t hardwareinit_serial_get_enable(void *ctx, uint8_t *enabled);
app_serial_status_t hardwareinit_serial_set_enable(void *ctx, uint8_t enabled);

#ifdef __cplusplus
}
#endif

#endif
