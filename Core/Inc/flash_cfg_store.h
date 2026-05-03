#ifndef FLASH_CFG_STORE_H
#define FLASH_CFG_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "rover_drive.h"
#include <stdint.h>

// Версия: F01 03.05.26
// Назначение:
// Хранение persistent параметров rover_drive во Flash STM32F401CCU6.
// Используется новый формат rover controller.

#define FLASH_CFG_STORE_BASE_ADDRESS            0x08020000u
#define FLASH_CFG_STORE_FLASH_SECTOR            FLASH_SECTOR_5
#define FLASH_CFG_STORE_FLASH_BANK              FLASH_BANK_1
#define FLASH_CFG_STORE_MAGIC                   0x32564643u
#define FLASH_CFG_STORE_FORMAT_VERSION          0x0100u
#define FLASH_CFG_STORE_MAX_WRITE_RETRY         1u

typedef struct
{
    rover_drive_params_t rover_params;
} flash_cfg_data_t;

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

flash_cfg_store_status_t flash_cfg_store_init(flash_cfg_store_t *store);
flash_cfg_store_status_t flash_cfg_store_load(flash_cfg_store_t *store, flash_cfg_data_t *data);
flash_cfg_store_status_t flash_cfg_store_save(flash_cfg_store_t *store, const flash_cfg_data_t *data);

#ifdef __cplusplus
}
#endif

#endif
