#ifndef FLASH_CFG_STORE_H
#define FLASH_CFG_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

// Версия: 01.00 29.03.26 12:20:00
// Назначение:
// Драйвер хранения коэффициентов балансировки во Flash STM32F401CCU6.
// Модуль хранит одну структуру с заголовком, счетчиком записи и CRC32.
// После старта можно восстановить последние сохраненные коэффициенты.
// После изменения коэффициентов можно сразу выполнить запись во Flash.
//
// Пример инициализации:
// flash_cfg_store_t cfg_store;
// flash_cfg_balance_data_t flash_data;
// memset(&cfg_store, 0, sizeof(cfg_store));
// if (flash_cfg_store_init(&cfg_store) != FLASH_CFG_STORE_STATUS_OK)
// {
//     Error_Handler();
// }
// if (flash_cfg_store_load(&cfg_store, &flash_data) == FLASH_CFG_STORE_STATUS_OK)
// {
//     // перенести flash_data в рабочую структуру коэффициентов
// }
//
// Пример сохранения после изменения параметра:
// flash_cfg_balance_data_t flash_data;
// заполнить flash_data из runtime структуры
// if (flash_cfg_store_save(&cfg_store, &flash_data) != FLASH_CFG_STORE_STATUS_OK)
// {
//     // обработка ошибки записи
// }
//
// Пример для main loop не нужен, драйвер вызывается по событию.

// Адрес последнего сектора для STM32F401CCU6 с Flash 256 Кбайт.
// Пример: 0x08020000u = начало сектора 5, размер 128 Кбайт.
#define FLASH_CFG_STORE_BASE_ADDRESS            0x08020000u
#define FLASH_CFG_STORE_FLASH_SECTOR            FLASH_SECTOR_5
#define FLASH_CFG_STORE_FLASH_BANK              FLASH_BANK_1
#define FLASH_CFG_STORE_MAGIC                   0x31474643u
#define FLASH_CFG_STORE_FORMAT_VERSION          0x0001u
#define FLASH_CFG_STORE_MAX_WRITE_RETRY         1u

// Параметры балансировки, сохраняемые во Flash.
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
} flash_cfg_balance_data_t;

typedef enum
{
    FLASH_CFG_STORE_STATUS_OK = 0,
    FLASH_CFG_STORE_STATUS_BAD_ARG,
    FLASH_CFG_STORE_STATUS_EMPTY,
    FLASH_CFG_STORE_STATUS_BAD_MAGIC,
    FLASH_CFG_STORE_STATUS_BAD_VERSION,
    FLASH_CFG_STORE_STATUS_BAD_SIZE,
    FLASH_CFG_STORE_STATUS_BAD_CRC,
    FLASH_CFG_STORE_STATUS_FLASH_ERROR,
    FLASH_CFG_STORE_STATUS_VERIFY_ERROR
} flash_cfg_store_status_t;

typedef struct
{
    uint32_t last_counter;

    struct
    {
        uint32_t init_ok;
        uint32_t load_ok;
        uint32_t save_ok;
        uint32_t save_count;
        uint32_t load_count;
        uint32_t last_error;
        uint32_t last_crc;
        uint32_t last_size;
        uint32_t last_magic;
        uint32_t last_version;
        uint32_t last_flash_error;
    } dbg;
} flash_cfg_store_t;

/// Инициализирует структуру драйвера хранения.
flash_cfg_store_status_t flash_cfg_store_init(flash_cfg_store_t *store);

/// Загружает коэффициенты из Flash в выходную структуру.
flash_cfg_store_status_t flash_cfg_store_load(flash_cfg_store_t *store, flash_cfg_balance_data_t *data);

/// Сохраняет коэффициенты во Flash с проверкой записанного содержимого.
flash_cfg_store_status_t flash_cfg_store_save(flash_cfg_store_t *store, const flash_cfg_balance_data_t *data);

#ifdef __cplusplus
}
#endif

#endif
