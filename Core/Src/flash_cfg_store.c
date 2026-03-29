#include "flash_cfg_store.h"
#include <string.h>

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t size;
    uint32_t counter;
    flash_cfg_balance_data_t data;
    uint32_t crc32;
} flash_cfg_store_image_t;

static uint32_t flash_cfg_store_crc32_calc(const uint8_t *data, uint32_t len)
{
    uint32_t crc;
    uint32_t i;
    uint32_t j;

    crc = 0xFFFFFFFFu;
    for (i = 0u; i < len; i++)
    {
        crc ^= (uint32_t)data[i];
        for (j = 0u; j < 8u; j++)
        {
            if ((crc & 1u) != 0u)
            {
                crc = (crc >> 1u) ^ 0xEDB88320u;
            }
            else
            {
                crc >>= 1u;
            }
        }
    }

    return (crc ^ 0xFFFFFFFFu);
}

static flash_cfg_store_status_t flash_cfg_store_validate_image(flash_cfg_store_t *store,
                                                               const flash_cfg_store_image_t *image)
{
    uint32_t crc_calc;

    if ((store == NULL) || (image == NULL))
    {
        return FLASH_CFG_STORE_STATUS_BAD_ARG;
    }

    store->dbg.last_magic = image->magic;
    store->dbg.last_version = image->version;
    store->dbg.last_size = image->size;
    store->dbg.last_crc = image->crc32;

    if (image->magic == 0xFFFFFFFFu)
    {
        return FLASH_CFG_STORE_STATUS_EMPTY;
    }

    if (image->magic != FLASH_CFG_STORE_MAGIC)
    {
        return FLASH_CFG_STORE_STATUS_BAD_MAGIC;
    }

    if (image->version != FLASH_CFG_STORE_FORMAT_VERSION)
    {
        return FLASH_CFG_STORE_STATUS_BAD_VERSION;
    }

    if (image->size != (uint32_t)sizeof(flash_cfg_balance_data_t))
    {
        return FLASH_CFG_STORE_STATUS_BAD_SIZE;
    }

    crc_calc = flash_cfg_store_crc32_calc((const uint8_t *)image,
                                          (uint32_t)(sizeof(flash_cfg_store_image_t) - sizeof(uint32_t)));
    if (crc_calc != image->crc32)
    {
        return FLASH_CFG_STORE_STATUS_BAD_CRC;
    }

    return FLASH_CFG_STORE_STATUS_OK;
}

flash_cfg_store_status_t flash_cfg_store_init(flash_cfg_store_t *store)
{
    if (store == NULL)
    {
        return FLASH_CFG_STORE_STATUS_BAD_ARG;
    }

    memset(store, 0, sizeof(*store));
    store->dbg.init_ok = 1u;
    return FLASH_CFG_STORE_STATUS_OK;
}

flash_cfg_store_status_t flash_cfg_store_load(flash_cfg_store_t *store, flash_cfg_balance_data_t *data)
{
    const flash_cfg_store_image_t *image;
    flash_cfg_store_status_t status;

    if ((store == NULL) || (data == NULL))
    {
        return FLASH_CFG_STORE_STATUS_BAD_ARG;
    }

    image = (const flash_cfg_store_image_t *)FLASH_CFG_STORE_BASE_ADDRESS;
    status = flash_cfg_store_validate_image(store, image);
    store->dbg.last_error = (uint32_t)status;
    if (status != FLASH_CFG_STORE_STATUS_OK)
    {
        return status;
    }

    memcpy(data, &image->data, sizeof(*data));
    store->last_counter = image->counter;
    store->dbg.load_ok = 1u;
    store->dbg.load_count++;
    return FLASH_CFG_STORE_STATUS_OK;
}

flash_cfg_store_status_t flash_cfg_store_save(flash_cfg_store_t *store, const flash_cfg_balance_data_t *data)
{
    flash_cfg_store_image_t image;
    const flash_cfg_store_image_t *image_rd;
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error;
    uint32_t address;
    uint32_t word_count;
    uint32_t i;
    HAL_StatusTypeDef hal_status;
    flash_cfg_store_status_t status;

    if ((store == NULL) || (data == NULL))
    {
        return FLASH_CFG_STORE_STATUS_BAD_ARG;
    }

    memset(&image, 0, sizeof(image));
    image.magic = FLASH_CFG_STORE_MAGIC;
    image.version = FLASH_CFG_STORE_FORMAT_VERSION;
    image.size = (uint32_t)sizeof(image.data);
    image.counter = store->last_counter + 1u;
    memcpy(&image.data, data, sizeof(image.data));
    image.crc32 = flash_cfg_store_crc32_calc((const uint8_t *)&image,
                                             (uint32_t)(sizeof(image) - sizeof(uint32_t)));

    HAL_FLASH_Unlock();

    memset(&erase, 0, sizeof(erase));
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase.Sector = FLASH_CFG_STORE_FLASH_SECTOR;
    erase.NbSectors = 1u;
#if defined(FLASH_BANK_1)
    erase.Banks = FLASH_CFG_STORE_FLASH_BANK;
#endif

    sector_error = 0u;
    hal_status = HAL_FLASHEx_Erase(&erase, &sector_error);
    if (hal_status != HAL_OK)
    {
        store->dbg.last_flash_error = HAL_FLASH_GetError();
        store->dbg.last_error = (uint32_t)FLASH_CFG_STORE_STATUS_FLASH_ERROR;
        (void)HAL_FLASH_Lock();
        return FLASH_CFG_STORE_STATUS_FLASH_ERROR;
    }

    address = FLASH_CFG_STORE_BASE_ADDRESS;
    word_count = (uint32_t)(sizeof(image) / sizeof(uint32_t));
    for (i = 0u; i < word_count; i++)
    {
        hal_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                       address,
                                       ((const uint32_t *)&image)[i]);
        if (hal_status != HAL_OK)
        {
            store->dbg.last_flash_error = HAL_FLASH_GetError();
            store->dbg.last_error = (uint32_t)FLASH_CFG_STORE_STATUS_FLASH_ERROR;
            (void)HAL_FLASH_Lock();
            return FLASH_CFG_STORE_STATUS_FLASH_ERROR;
        }
        address += sizeof(uint32_t);
    }

    (void)HAL_FLASH_Lock();

    image_rd = (const flash_cfg_store_image_t *)FLASH_CFG_STORE_BASE_ADDRESS;
    status = flash_cfg_store_validate_image(store, image_rd);
    if (status != FLASH_CFG_STORE_STATUS_OK)
    {
        store->dbg.last_error = (uint32_t)FLASH_CFG_STORE_STATUS_VERIFY_ERROR;
        return FLASH_CFG_STORE_STATUS_VERIFY_ERROR;
    }

    if (memcmp(&image, image_rd, sizeof(image)) != 0)
    {
        store->dbg.last_error = (uint32_t)FLASH_CFG_STORE_STATUS_VERIFY_ERROR;
        return FLASH_CFG_STORE_STATUS_VERIFY_ERROR;
    }

    store->last_counter = image.counter;
    store->dbg.save_ok = 1u;
    store->dbg.save_count++;
    store->dbg.last_error = (uint32_t)FLASH_CFG_STORE_STATUS_OK;
    return FLASH_CFG_STORE_STATUS_OK;
}
