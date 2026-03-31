#ifndef HARDWAREINIT_H
#define HARDWAREINIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "balance.h"
#include "balance_calibration.h"
#include "flash_cfg_store.h"
#include "app_serial.h"
#include <stdint.h>

// Версия: 01.01 31.03.26 19:35:00
// Назначение:
// Модуль инициализации приложения. Хранит рабочую структуру проекта,
// заполняет параметры по умолчанию, загружает коэффициенты из Flash,
// результаты калибровки и подключает serial callbacks.

typedef enum
{
    HARDWAREINIT_STATUS_OK = 0,
    HARDWAREINIT_STATUS_BAD_ARG,
    HARDWAREINIT_STATUS_FLASH_ERROR,
    HARDWAREINIT_STATUS_BALANCE_ERROR,
    HARDWAREINIT_STATUS_SERIAL_ERROR
} hardwareinit_status_t;

typedef struct
{
    balance_controller_t balance;
    balance_command_t command;
    balance_calibration_persist_t calib_data;
    uint8_t control_enabled;
    uint8_t flash_cfg_loaded;
    uint8_t calib_loaded;
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
