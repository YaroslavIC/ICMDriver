// icm20948_driver.c
// Драйвер ICM-20948 (только Accel+Gyro) по SPI + INT + DMA для STM32F401CCU6 (HAL)
// Версия драйвера: 1.1.0
//
// Все комментарии только через //, без /* и */.
// Кодировка: Windows-1251.

#include "icm20948_driver.h"

static float icm_abs_f(float v)
{
    return (v < 0.0f) ? -v : v;
}

static void icm_cs_low(ICM20948_t* dev)
{
    HAL_GPIO_WritePin(dev->cfg.cs_port, dev->cfg.cs_pin, GPIO_PIN_RESET);
}

static void icm_cs_high(ICM20948_t* dev)
{
    HAL_GPIO_WritePin(dev->cfg.cs_port, dev->cfg.cs_pin, GPIO_PIN_SET);
}

static ICM20948_Status_t icm_spi_txrx_blocking(ICM20948_t* dev, uint8_t* tx, uint8_t* rx, uint16_t len)
{
    HAL_StatusTypeDef hs;

    icm_cs_low(dev);
    hs = HAL_SPI_TransmitReceive(dev->cfg.hspi, tx, rx, len, dev->cfg.spi_timeout_ms);
    icm_cs_high(dev);

    if (hs != HAL_OK)
    {
        return ICM20948_ERR_SPI;
    }

    return ICM20948_OK;
}

static ICM20948_Status_t icm_write_reg(ICM20948_t* dev, uint8_t reg, uint8_t val)
{
    ICM20948_Status_t st;

    dev->tx_buf[0] = (uint8_t)(reg & 0x7Fu);
    dev->tx_buf[1] = val;

    st = icm_spi_txrx_blocking(dev, dev->tx_buf, dev->rx_buf, 2u);
    return st;
}

static ICM20948_Status_t icm_read_regs(ICM20948_t* dev, uint8_t reg, uint8_t* out, uint16_t n)
{
    ICM20948_Status_t st;
    uint16_t i;

    if (out == 0)
    {
        return ICM20948_ERR_NULL_PTR;
    }
    if ((n == 0u) || (n > 16u))
    {
        return ICM20948_ERR_PARAM;
    }

    dev->tx_buf[0] = (uint8_t)(reg | ICM20948_SPI_READ_BIT);
    for (i = 1u; i < (uint16_t)(n + 1u); i++)
    {
        dev->tx_buf[i] = ICM20948_SPI_DUMMY_BYTE_DEFAULT;
    }

    st = icm_spi_txrx_blocking(dev, dev->tx_buf, dev->rx_buf, (uint16_t)(n + 1u));
    if (st != ICM20948_OK)
    {
        return st;
    }

    for (i = 0u; i < n; i++)
    {
        out[i] = dev->rx_buf[i + 1u];
    }

    return ICM20948_OK;
}

static ICM20948_Status_t icm_set_bank(ICM20948_t* dev, uint8_t bank)
{
    // Банк выбираем всегда явно, без кеширования, чтобы упростить первую версию.
    return icm_write_reg(dev, ICM20948_REG_BANK_SEL, bank);
}

static uint8_t icm_calc_gyro_cfg1(uint32_t fs_dps)
{
    uint8_t cfg;

    // GYRO_CONFIG_1:
    // bits[5:3] = GYRO_DLPFCFG
    // bits[2:1] = FS_SEL
    // bit[0]    = GYRO_FCHOICE (1 = enable DLPF)
    cfg = (uint8_t)(((uint8_t)(ICM20948_GYRO_DLPFCFG_DEFAULT & 0x07u)) << 3);

    if (fs_dps == 250u)      { cfg |= (uint8_t)(0u << 1); }
    else if (fs_dps == 500u) { cfg |= (uint8_t)(1u << 1); }
    else if (fs_dps == 1000u){ cfg |= (uint8_t)(2u << 1); }
    else                     { cfg |= (uint8_t)(3u << 1); }

    cfg |= 0x01u;

    return cfg;
}

static uint8_t icm_calc_accel_cfg(uint32_t fs_g)
{
    uint8_t cfg;

    // ACCEL_CONFIG:
    // bits[5:3] = ACCEL_DLPFCFG
    // bits[2:1] = FS_SEL
    // bit[0]    = ACCEL_FCHOICE (1 = enable DLPF)
    cfg = (uint8_t)(((uint8_t)(ICM20948_ACCEL_DLPFCFG_DEFAULT & 0x07u)) << 3);

    if (fs_g == 2u)       { cfg |= (uint8_t)(0u << 1); }
    else if (fs_g == 4u)  { cfg |= (uint8_t)(1u << 1); }
    else if (fs_g == 8u)  { cfg |= (uint8_t)(2u << 1); }
    else                  { cfg |= (uint8_t)(3u << 1); }

    cfg |= 0x01u;

    return cfg;
}

static void icm_update_scale_constants(ICM20948_t* dev)
{
    // LSB per g
    if (ICM20948_ACCEL_FS_G_DEFAULT == 2u)      { dev->accel_lsb_per_g = 16384.0f; }
    else if (ICM20948_ACCEL_FS_G_DEFAULT == 4u) { dev->accel_lsb_per_g = 8192.0f; }
    else if (ICM20948_ACCEL_FS_G_DEFAULT == 8u) { dev->accel_lsb_per_g = 4096.0f; }
    else                                       { dev->accel_lsb_per_g = 2048.0f; }

    // LSB per dps
    if (ICM20948_GYRO_FS_DPS_DEFAULT == 250u)       { dev->gyro_lsb_per_dps = 131.0f; }
    else if (ICM20948_GYRO_FS_DPS_DEFAULT == 500u)  { dev->gyro_lsb_per_dps = 65.5f; }
    else if (ICM20948_GYRO_FS_DPS_DEFAULT == 1000u) { dev->gyro_lsb_per_dps = 32.8f; }
    else                                           { dev->gyro_lsb_per_dps = 16.4f; }
}

static ICM20948_Status_t icm_dwt_ensure_running(ICM20948_t* dev)
{
    uint32_t a;
    uint32_t b;

    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0u)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u)
    {
        DWT->CYCCNT = 0u;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    a = DWT->CYCCNT;
    __NOP();
    __NOP();
    __NOP();
    b = DWT->CYCCNT;

    if (b == a)
    {
        return ICM20948_ERR_DWT_NOT_RUNNING;
    }

    dev->dwt_hi = 0u;
    dev->dwt_last = b;

    return ICM20948_OK;
}

static uint64_t icm_cycles64(ICM20948_t* dev)
{
    uint32_t now;
    uint32_t last;
    uint32_t hi;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    now = DWT->CYCCNT;
    last = dev->dwt_last;
    hi = dev->dwt_hi;

    if (now < last)
    {
        hi++;
        dev->dwt_hi = hi;
    }
    dev->dwt_last = now;

    if (primask == 0u)
    {
        __enable_irq();
    }

    return (((uint64_t)hi) << 32) | (uint64_t)now;
}

uint64_t icm_micros64(ICM20948_t* dev)
{
    uint32_t div;

    div = (uint32_t)(SystemCoreClock / 1000000u);
    if (div == 0u)
    {
        return 0ull;
    }

    return icm_cycles64(dev) / (uint64_t)div;
}

static ICM20948_CalStep_t icm_autostep_from_accel_g(float ax, float ay, float az)
{
    float abx;
    float aby;
    float abz;

    abx = icm_abs_f(ax);
    aby = icm_abs_f(ay);
    abz = icm_abs_f(az);

    if ((abx >= aby) && (abx >= abz))
    {
        return (ax >= 0.0f) ? ICM20948_CAL_STEP_POS_X : ICM20948_CAL_STEP_NEG_X;
    }
    if ((aby >= abx) && (aby >= abz))
    {
        return (ay >= 0.0f) ? ICM20948_CAL_STEP_POS_Y : ICM20948_CAL_STEP_NEG_Y;
    }
    return (az >= 0.0f) ? ICM20948_CAL_STEP_POS_Z : ICM20948_CAL_STEP_NEG_Z;
}

static void icm_cal_reset_accum(ICM20948_t* dev, ICM20948_CalResetReason_t reason)
{
    // Сбрасываем только накопление текущего шага акселерометра.
    // Поля гирокалибровки здесь НЕ трогаем.
    dev->cal6.progress = 0u;
    dev->cal6.sum_ax_g = 0.0;
    dev->cal6.sum_ay_g = 0.0;
    dev->cal6.sum_az_g = 0.0;
    dev->cal6.last_reset_reason = reason;
    dev->cal6.last_accel_valid = 0u;
}


static ICM20948_Status_t icm_cal_try_finish_step(ICM20948_t* dev, ICM20948_CalStep_t step)
{
    float mean[3];
    float dom_abs;

    if (dev->cal6.progress < (uint16_t)ICM20948_CAL_SAMPLES_PER_STEP_DEFAULT)
    {
        return ICM20948_BUSY;
    }

    mean[0] = (float)(dev->cal6.sum_ax_g / (double)ICM20948_CAL_SAMPLES_PER_STEP_DEFAULT);
    mean[1] = (float)(dev->cal6.sum_ay_g / (double)ICM20948_CAL_SAMPLES_PER_STEP_DEFAULT);
    mean[2] = (float)(dev->cal6.sum_az_g / (double)ICM20948_CAL_SAMPLES_PER_STEP_DEFAULT);

    // Доминирующая ось по max(|a|)
    dom_abs = icm_abs_f(mean[0]);
    if (icm_abs_f(mean[1]) > dom_abs) { dom_abs = icm_abs_f(mean[1]); }
    if (icm_abs_f(mean[2]) > dom_abs) { dom_abs = icm_abs_f(mean[2]); }

    // Проверка близости к 1g по доминирующей оси
    if ((dom_abs < ICM20948_CAL_DOM_AXIS_G_MIN) || (dom_abs > ICM20948_CAL_DOM_AXIS_G_MAX))
    {
        if (dev->cal6.retry_count < (uint8_t)ICM20948_CAL_MAX_RETRY_PER_STEP)
        {
            dev->cal6.retry_count++;
            icm_cal_reset_accum(dev, ICM20948_CAL_RESET_BAD_GRAVITY);
            return ICM20948_BUSY;
        }
        // Вторая неудача: принимаем данные
    }

    dev->cal6.mean_step_g[(uint8_t)step][0] = mean[0];
    dev->cal6.mean_step_g[(uint8_t)step][1] = mean[1];
    dev->cal6.mean_step_g[(uint8_t)step][2] = mean[2];

    dev->cal6.done_mask |= (uint8_t)(1u << (uint8_t)step);
    dev->cal6.active_step = ICM20948_CAL_STEP_NONE;
    dev->cal6.state = ICM20948_CAL_STATE_IDLE;

    return ICM20948_OK;
}

static ICM20948_Status_t icm_cal_compute_result(ICM20948_t* dev)
{
    float px;
    float nx;
    float py;
    float ny;
    float pz;
    float nz;

    float dx;
    float dy;
    float dz;

    // Используем компоненту соответствующей оси
    px = dev->cal6.mean_step_g[(uint8_t)ICM20948_CAL_STEP_POS_X][0];
    nx = dev->cal6.mean_step_g[(uint8_t)ICM20948_CAL_STEP_NEG_X][0];
    py = dev->cal6.mean_step_g[(uint8_t)ICM20948_CAL_STEP_POS_Y][1];
    ny = dev->cal6.mean_step_g[(uint8_t)ICM20948_CAL_STEP_NEG_Y][1];
    pz = dev->cal6.mean_step_g[(uint8_t)ICM20948_CAL_STEP_POS_Z][2];
    nz = dev->cal6.mean_step_g[(uint8_t)ICM20948_CAL_STEP_NEG_Z][2];

    dx = (px - nx);
    dy = (py - ny);
    dz = (pz - nz);

    if ((icm_abs_f(dx) < 0.1f) || (icm_abs_f(dy) < 0.1f) || (icm_abs_f(dz) < 0.1f))
    {
        return ICM20948_ERR_PARAM;
    }

    dev->cal.acc_bias_g[0] = (px + nx) * 0.5f;
    dev->cal.acc_bias_g[1] = (py + ny) * 0.5f;
    dev->cal.acc_bias_g[2] = (pz + nz) * 0.5f;

    dev->cal.acc_scale[0] = 2.0f / dx;
    dev->cal.acc_scale[1] = 2.0f / dy;
    dev->cal.acc_scale[2] = 2.0f / dz;

    return ICM20948_OK;
}

ICM20948_Status_t ICM20948_Init(ICM20948_t* dev)
{
    ICM20948_Status_t st;
    uint8_t who;
    uint8_t tmp[1];
    uint8_t cfg;
    uint8_t status1;

    if ((dev == 0))
    {
        return ICM20948_ERR_NULL_PTR;
    }
    if ((dev->p.hspi == 0) || (dev->p.hdma_rx == 0) || (dev->p.hdma_tx == 0))
    {
        return ICM20948_ERR_NULL_PTR;
    }
    if ((dev->p.cs_port == 0) || (dev->p.int_port == 0))
    {
        return ICM20948_ERR_NULL_PTR;
    }
    if ((dev->p.spi_timeout_ms == 0u) || (dev->p.spi_timeout_ms > 1000u))
    {
        return ICM20948_ERR_PARAM;
    }

    dev->cfg.hspi = dev->p.hspi;
    dev->cfg.hdma_rx = dev->p.hdma_rx;
    dev->cfg.hdma_tx = dev->p.hdma_tx;
    dev->cfg.cs_port = dev->p.cs_port;
    dev->cfg.cs_pin = dev->p.cs_pin;
    dev->cfg.int_port = dev->p.int_port;
    dev->cfg.int_pin = dev->p.int_pin;
    dev->cfg.spi_timeout_ms = dev->p.spi_timeout_ms;

    dev->state = ICM20948_STATE_UNINIT;
    dev->spi_busy = 0u;
    dev->new_sample = 0u;
    dev->raw_valid = 0u;
    dev->real_samples_prev_timestamp_us = 0ull;
    dev->real_samples_hz = 0.0f;
    dev->real_samples_period_us = 0.0f;

    dev->dbg.reg_bank_sel = 0u;
    dev->dbg.user_ctrl = 0u;
    dev->dbg.int_pin_cfg = 0u;
    dev->dbg.int_enable_1 = 0u;
    dev->dbg.int_status_1 = 0u;
    dev->dbg.gyro_smplrt_div = 0u;
    dev->dbg.gyro_config_1 = 0u;
    dev->dbg.odr_align_en = 0u;
    dev->dbg.accel_smplrt_div_1 = 0u;
    dev->dbg.accel_smplrt_div_2 = 0u;
    dev->dbg.accel_config = 0u;

    dev->cal.acc_bias_g[0] = ICM20948_ACC_BIAS_X_G_DEFAULT;
    dev->cal.acc_bias_g[1] = ICM20948_ACC_BIAS_Y_G_DEFAULT;
    dev->cal.acc_bias_g[2] = ICM20948_ACC_BIAS_Z_G_DEFAULT;

    dev->cal.acc_scale[0] = ICM20948_ACC_SCALE_X_DEFAULT;
    dev->cal.acc_scale[1] = ICM20948_ACC_SCALE_Y_DEFAULT;
    dev->cal.acc_scale[2] = ICM20948_ACC_SCALE_Z_DEFAULT;

    dev->cal.gyro_bias_rads[0] = ICM20948_GYRO_BIAS_X_RADS_DEFAULT;
    dev->cal.gyro_bias_rads[1] = ICM20948_GYRO_BIAS_Y_RADS_DEFAULT;
    dev->cal.gyro_bias_rads[2] = ICM20948_GYRO_BIAS_Z_RADS_DEFAULT;

    dev->cal6.state = ICM20948_CAL_STATE_IDLE;
    dev->cal6.done_mask = 0u;
    dev->cal6.retry_count = 0u;
    dev->cal6.active_step = ICM20948_CAL_STEP_NONE;
    dev->cal6.last_auto_step = ICM20948_CAL_STEP_NONE;
    dev->cal6.last_reset_reason = ICM20948_CAL_RESET_NONE;
    dev->cal6.progress = 0u;
    dev->cal6.sum_ax_g = 0.0;
    dev->cal6.sum_ay_g = 0.0;
    dev->cal6.sum_az_g = 0.0;
    dev->cal6.last_accel_valid = 0u;

    st = icm_dwt_ensure_running(dev);
    if (st != ICM20948_OK)
    {
        return st;
    }

    icm_update_scale_constants(dev);

    // Тактирование SPI3 и DMA1
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // CS как output, держим 1
    {
        GPIO_InitTypeDef gi;
        gi.Pin = dev->cfg.cs_pin;
        gi.Mode = GPIO_MODE_OUTPUT_PP;
        gi.Pull = GPIO_NOPULL;
        gi.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(dev->cfg.cs_port, &gi);
        HAL_GPIO_WritePin(dev->cfg.cs_port, dev->cfg.cs_pin, GPIO_PIN_SET);
    }

    // SPI и DMA должны быть уже настроены снаружи (например CubeMX).
    // Драйвер использует переданные handle как есть.
    if ((dev->cfg.hspi->Instance == 0) ||
        (dev->cfg.hdma_rx->Instance == 0) ||
        (dev->cfg.hdma_tx->Instance == 0))
    {
        return ICM20948_ERR_PARAM;
    }

    __HAL_LINKDMA(dev->cfg.hspi, hdmarx, *dev->cfg.hdma_rx);
    __HAL_LINKDMA(dev->cfg.hspi, hdmatx, *dev->cfg.hdma_tx);

    HAL_NVIC_SetPriority(ICM20948_DMA_RX_IRQn, ICM20948_DMA_RX_IRQ_PRIO, 0u);
    HAL_NVIC_EnableIRQ(ICM20948_DMA_RX_IRQn);
    HAL_NVIC_SetPriority(ICM20948_DMA_TX_IRQn, ICM20948_DMA_TX_IRQ_PRIO, 0u);
    HAL_NVIC_EnableIRQ(ICM20948_DMA_TX_IRQn);

    // Проверка WHO_AM_I
    st = icm_set_bank(dev, ICM20948_BANK_0);
    if (st != ICM20948_OK)
    {
        return st;
    }
    st = icm_read_regs(dev, ICM20948_REG_WHO_AM_I, tmp, 1u);
    if (st != ICM20948_OK)
    {
        return st;
    }
    who = tmp[0];
    if (who != (uint8_t)ICM20948_WHO_AM_I_VALUE)
    {
        return ICM20948_ERR_DEVICE_ID;
    }

    // Reset
    st = icm_write_reg(dev, ICM20948_REG_PWR_MGMT_1, ICM20948_PWR1_DEVICE_RESET);
    if (st != ICM20948_OK)
    {
        return st;
    }
    HAL_Delay(10u);

    // Выходим из sleep, выбираем auto clock
    st = icm_write_reg(dev, ICM20948_REG_PWR_MGMT_1, ICM20948_PWR1_CLKSEL_AUTO);
    if (st != ICM20948_OK)
    {
        return st;
    }

    // Включаем accel+gyro (не выключаем оси)
    st = icm_write_reg(dev, ICM20948_REG_PWR_MGMT_2, 0x00u);
    if (st != ICM20948_OK)
    {
        return st;
    }

    // В SPI режиме по документации нужно отключить I2C slave interface
    st = icm_write_reg(dev, ICM20948_REG_USER_CTRL, ICM20948_USER_CTRL_I2C_IF_DIS);
    if (st != ICM20948_OK)
    {
        return st;
    }

    // INT pin cfg: push-pull, active high, pulse
    // BIT4 INT_ANYRD_2CLEAR = 0
    // BIT5 LATCH_INT_EN = 0 для pulse режима
    // BIT7 ACTL = 0 для active high
    // BIT6 OPEN = 0 для push-pull
    cfg = 0x00u;
    if (ICM20948_INT_LATCHED_DEFAULT != 0u)
    {
        cfg |= 0x20u;
    }
    st = icm_write_reg(dev, ICM20948_REG_INT_PIN_CFG, cfg);
    if (st != ICM20948_OK)
    {
        return st;
    }

    // Разрешаем RAW_DATA_0_RDY на INT1 через INT_ENABLE_1
    st = icm_write_reg(dev, ICM20948_REG_INT_ENABLE_1, ICM20948_INT1_RAW_RDY_EN);
    if (st != ICM20948_OK)
    {
        return st;
    }

    // Считываем INT_STATUS_1 один раз, чтобы очистить стартовый статус прерывания
    st = icm_read_regs(dev, ICM20948_REG_INT_STATUS_1, &status1, 1u);
    if (st != ICM20948_OK)
    {
        return st;
    }

    // Настройки частоты и диапазонов в Bank 2
    st = icm_set_bank(dev, ICM20948_BANK_2);
    if (st != ICM20948_OK)
    {
        return st;
    }

    st = icm_write_reg(dev, ICM20948_REG_GYRO_SMPLRT_DIV, (uint8_t)ICM20948_RATE_GYRO_SMPLRT_DIV_DEFAULT);
    if (st != ICM20948_OK)
    {
        return st;
    }

    // Синхронизируем старт ODR для gyro и accel
    st = icm_write_reg(dev, ICM20948_REG_ODR_ALIGN_EN,
                       (uint8_t)(ICM20948_ODR_ALIGN_EN_DEFAULT ? 0x01u : 0x00u));
    if (st != ICM20948_OK)
    {
        return st;
    }

    // GYRO_CONFIG_1: включаем DLPF и задаем FS
    cfg = icm_calc_gyro_cfg1(ICM20948_GYRO_FS_DPS_DEFAULT);
    st = icm_write_reg(dev, ICM20948_REG_GYRO_CONFIG_1, cfg);
    if (st != ICM20948_OK)
    {
        return st;
    }

    // ACCEL_SMPLRT_DIV берется из ICM20948_RATE_ACCEL_SMPLRT_DIV_DEFAULT
    st = icm_write_reg(dev, ICM20948_REG_ACCEL_SMPLRT_DIV_1,
                       (uint8_t)((ICM20948_RATE_ACCEL_SMPLRT_DIV_DEFAULT >> 8) & 0xFFu));
    if (st != ICM20948_OK)
    {
        return st;
    }
    st = icm_write_reg(dev, ICM20948_REG_ACCEL_SMPLRT_DIV_2,
                       (uint8_t)(ICM20948_RATE_ACCEL_SMPLRT_DIV_DEFAULT & 0xFFu));
    if (st != ICM20948_OK)
    {
        return st;
    }

    // ACCEL_CONFIG: включаем DLPF и задаем FS
    cfg = icm_calc_accel_cfg(ICM20948_ACCEL_FS_G_DEFAULT);
    st = icm_write_reg(dev, ICM20948_REG_ACCEL_CONFIG, cfg);
    if (st != ICM20948_OK)
    {
        return st;
    }

    st = icm_set_bank(dev, ICM20948_BANK_0);
    if (st != ICM20948_OK)
    {
        return st;
    }

    st = ICM20948_ReadDebugRegs(dev);
    if (st != ICM20948_OK)
    {
        return st;
    }

    dev->state = ICM20948_STATE_IDLE;

    dev->gyro_cal_start = 1;

    return ICM20948_OK;
}

ICM20948_Status_t ICM20948_IrqHandler(ICM20948_t* dev, uint16_t gpio_pin)
{
    HAL_StatusTypeDef hs;
    uint16_t i;

    if (dev == 0)
    {
        return ICM20948_ERR_NULL_PTR;
    }

    if (gpio_pin != dev->cfg.int_pin)
    {
        return ICM20948_OK;
    }

    if (dev->spi_busy != 0u)
    {
        return ICM20948_OK;
    }

    dev->raw.timestamp_us = icm_micros64(dev);

    dev->tx_buf[0] = (uint8_t)(ICM20948_REG_ACCEL_XOUT_H | ICM20948_SPI_READ_BIT);
    for (i = 1u; i < (uint16_t)ICM20948_BURST_TOTAL_BYTES; i++)
    {
        dev->tx_buf[i] = ICM20948_SPI_DUMMY_BYTE_DEFAULT;
    }

    dev->spi_busy = 1u;
    dev->state = ICM20948_STATE_DMA_ACTIVE;

    icm_cs_low(dev);
    hs = HAL_SPI_TransmitReceive_DMA(dev->cfg.hspi, dev->tx_buf, dev->rx_buf, (uint16_t)ICM20948_BURST_TOTAL_BYTES);
    if (hs != HAL_OK)
    {
        icm_cs_high(dev);
        dev->spi_busy = 0u;
        dev->state = ICM20948_STATE_IDLE;
        return ICM20948_ERR_DMA;
    }

    return ICM20948_OK;
}

ICM20948_Status_t ICM20948_SpiDmaCpltHandler(ICM20948_t* dev, SPI_HandleTypeDef* hspi)
{
    uint8_t* r;
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;

    float ax_g;
    float ay_g;
    float az_g;

    float da_abs;
    uint8_t status1;

    if ((dev == 0) || (hspi == 0))
    {
        return ICM20948_ERR_NULL_PTR;
    }
    if (hspi != dev->cfg.hspi)
    {
        return ICM20948_OK;
    }

    icm_cs_high(dev);

    r = dev->rx_buf;

    ax = (int16_t)((((uint16_t)r[1]) << 8) | (uint16_t)r[2]);
    ay = (int16_t)((((uint16_t)r[3]) << 8) | (uint16_t)r[4]);
    az = (int16_t)((((uint16_t)r[5]) << 8) | (uint16_t)r[6]);

    gx = (int16_t)((((uint16_t)r[9]) << 8) | (uint16_t)r[10]);
    gy = (int16_t)((((uint16_t)r[11]) << 8) | (uint16_t)r[12]);
    gz = (int16_t)((((uint16_t)r[13]) << 8) | (uint16_t)r[14]);

    // Сохраняем raw
    dev->raw.accel_raw[0] = ax;
    dev->raw.accel_raw[1] = ay;
    dev->raw.accel_raw[2] = az;

    dev->raw.gyro_raw[0] = gx;
    dev->raw.gyro_raw[1] = gy;
    dev->raw.gyro_raw[2] = gz;

    dev->raw_valid = 1u;
    dev->new_sample = 1u;

    // Обновляем оценку реальной частоты прихода данных акселерометра
    (void)ICM20948_RealSamples(dev);

    // Для RAW_DATA_0_RDY источник прерывания находится в INT_STATUS_1 (R/C).
    // Чтение sensor data его не очищает, поэтому читаем INT_STATUS_1 после приема пакета.
    (void)icm_read_regs(dev, ICM20948_REG_INT_STATUS_1, &status1, 1u);

    // Калибровка: накопление в g + детекция движения (gyro+accel delta)
    if (dev->cal6.state == ICM20948_CAL_STATE_ACCUM)
    {
        ax_g = ((float)ax) / dev->accel_lsb_per_g;
        ay_g = ((float)ay) / dev->accel_lsb_per_g;
        az_g = ((float)az) / dev->accel_lsb_per_g;

        // В версии 1.0.7 контроль движения по гироскопу отключен.
        // Для калибровки используем только контроль по акселерометру.
        // В версии 1.0.19 автоматическая калибровка гироскопа из CalSetStep отключена.

        da_abs = 0.0f;
        if (dev->cal6.last_accel_valid != 0u)
        {
            float dx;
            float dy;
            float dz;

            dx = icm_abs_f(ax_g - dev->cal6.last_accel_g[0]);
            dy = icm_abs_f(ay_g - dev->cal6.last_accel_g[1]);
            dz = icm_abs_f(az_g - dev->cal6.last_accel_g[2]);

            da_abs = dx;
            if (dy > da_abs) { da_abs = dy; }
            if (dz > da_abs) { da_abs = dz; }
        }

        dev->cal6.last_accel_g[0] = ax_g;
        dev->cal6.last_accel_g[1] = ay_g;
        dev->cal6.last_accel_g[2] = az_g;
        dev->cal6.last_accel_valid = 1u;

        if (da_abs > ICM20948_CAL_ACCEL_DELTA_THR_G)
        {
            icm_cal_reset_accum(dev, ICM20948_CAL_RESET_ACCEL_DELTA);
        }
        else
        {
            if (dev->cal6.progress < (uint16_t)ICM20948_CAL_SAMPLES_PER_STEP_DEFAULT)
            {
                dev->cal6.sum_ax_g += (double)ax_g;
                dev->cal6.sum_ay_g += (double)ay_g;
                dev->cal6.sum_az_g += (double)az_g;
                dev->cal6.progress++;
            }
        }
    }

    dev->spi_busy = 0u;
    dev->state = ICM20948_STATE_IDLE;

    return ICM20948_OK;
}

ICM20948_Status_t ICM20948_Service(ICM20948_t* dev)
{
    ICM20948_Status_t st;

    if (dev == 0)
    {
        return ICM20948_ERR_NULL_PTR;
    }

    if (dev->cal6.state == ICM20948_CAL_STATE_ACCUM)
    {
        st = icm_cal_try_finish_step(dev, dev->cal6.active_step);
        if (st == ICM20948_OK)
        {
            if (dev->cal6.done_mask == 0x3Fu)
            {
                st = icm_cal_compute_result(dev);
                if (st == ICM20948_OK)
                {
                    dev->cal6.state = ICM20948_CAL_STATE_DONE;
                }
                return st;
            }
        }
    }

    return ICM20948_OK;
}

ICM20948_Status_t ICM20948_GetLatestRaw(ICM20948_t* dev, ICM20948_RawSample_t* out)
{
    uint32_t primask;

    if ((dev == 0) || (out == 0))
    {
        return ICM20948_ERR_NULL_PTR;
    }
    if (dev->raw_valid == 0u)
    {
        return ICM20948_NO_NEW_SAMPLE;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (dev->new_sample == 0u)
    {
        if (primask == 0u) { __enable_irq(); }
        return ICM20948_NO_NEW_SAMPLE;
    }

    *out = dev->raw;
    dev->new_sample = 0u;

    if (primask == 0u)
    {
        __enable_irq();
    }

    return ICM20948_OK;
}

ICM20948_Status_t ICM20948_GetLatestPhys(ICM20948_t* dev, ICM20948_PhysSample_t* out)
{
    uint32_t primask;
    ICM20948_RawSample_t s;

    float ax_g;
    float ay_g;
    float az_g;

    float gx_rads;
    float gy_rads;
    float gz_rads;

    if ((dev == 0) || (out == 0))
    {
        return ICM20948_ERR_NULL_PTR;
    }
    if (dev->raw_valid == 0u)
    {
        return ICM20948_NO_NEW_SAMPLE;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (dev->new_sample == 0u)
    {
        if (primask == 0u) { __enable_irq(); }
        return ICM20948_NO_NEW_SAMPLE;
    }

    s = dev->raw;
    dev->new_sample = 0u;

    if (primask == 0u)
    {
        __enable_irq();
    }

    ax_g = ((float)s.accel_raw[0]) / dev->accel_lsb_per_g;
    ay_g = ((float)s.accel_raw[1]) / dev->accel_lsb_per_g;
    az_g = ((float)s.accel_raw[2]) / dev->accel_lsb_per_g;

    ax_g = (ax_g - dev->cal.acc_bias_g[0]) * dev->cal.acc_scale[0];
    ay_g = (ay_g - dev->cal.acc_bias_g[1]) * dev->cal.acc_scale[1];
    az_g = (az_g - dev->cal.acc_bias_g[2]) * dev->cal.acc_scale[2];

    gx_rads = (((float)s.gyro_raw[0]) / dev->gyro_lsb_per_dps) * ICM20948_DEG_TO_RAD;
    gy_rads = (((float)s.gyro_raw[1]) / dev->gyro_lsb_per_dps) * ICM20948_DEG_TO_RAD;
    gz_rads = (((float)s.gyro_raw[2]) / dev->gyro_lsb_per_dps) * ICM20948_DEG_TO_RAD;

    gx_rads = gx_rads - dev->cal.gyro_bias_rads[0];
    gy_rads = gy_rads - dev->cal.gyro_bias_rads[1];
    gz_rads = gz_rads - dev->cal.gyro_bias_rads[2];

    out->timestamp_us = s.timestamp_us;

    out->accel_mps2[0] = ax_g * ICM20948_G_TO_MPS2;
    out->accel_mps2[1] = ay_g * ICM20948_G_TO_MPS2;
    out->accel_mps2[2] = az_g * ICM20948_G_TO_MPS2;

    out->gyro_rads[0] = gx_rads;
    out->gyro_rads[1] = gy_rads;
    out->gyro_rads[2] = gz_rads;

    return ICM20948_OK;
}


ICM20948_Status_t ICM20948_RealSamples(ICM20948_t* dev)
{
    uint32_t primask;
    uint64_t now_us;
    uint64_t prev_us;
    uint64_t dt_us;

    if (dev == 0)
    {
        return ICM20948_ERR_NULL_PTR;
    }
    if (dev->raw_valid == 0u)
    {
        return ICM20948_NO_NEW_SAMPLE;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    now_us = dev->raw.timestamp_us;
    prev_us = dev->real_samples_prev_timestamp_us;

    if (primask == 0u)
    {
        __enable_irq();
    }

    if (prev_us == 0ull)
    {
        dev->real_samples_prev_timestamp_us = now_us;
        dev->real_samples_hz = 0.0f;
        dev->real_samples_period_us = 0.0f;
        return ICM20948_BUSY;
    }

    if (now_us <= prev_us)
    {
        return ICM20948_BUSY;
    }

    dt_us = (uint64_t)(now_us - prev_us);
    dev->real_samples_prev_timestamp_us = now_us;

    dev->real_samples_period_us = (float)dt_us;
    dev->real_samples_hz = 1000000.0f / (float)dt_us;

    return ICM20948_OK;
}



ICM20948_Status_t ICM20948_ReadDebugRegs(ICM20948_t* dev)
{
    ICM20948_Status_t st;
    uint8_t tmp[1];

    if (dev == 0)
    {
        return ICM20948_ERR_NULL_PTR;
    }

    st = icm_set_bank(dev, ICM20948_BANK_0);
    if (st != ICM20948_OK)
    {
        return st;
    }

    st = icm_read_regs(dev, ICM20948_REG_BANK_SEL, tmp, 1u);
    if (st != ICM20948_OK) { return st; }
    dev->dbg.reg_bank_sel = tmp[0];

    st = icm_read_regs(dev, ICM20948_REG_USER_CTRL, tmp, 1u);
    if (st != ICM20948_OK) { return st; }
    dev->dbg.user_ctrl = tmp[0];

    st = icm_read_regs(dev, ICM20948_REG_INT_PIN_CFG, tmp, 1u);
    if (st != ICM20948_OK) { return st; }
    dev->dbg.int_pin_cfg = tmp[0];

    st = icm_read_regs(dev, ICM20948_REG_INT_ENABLE_1, tmp, 1u);
    if (st != ICM20948_OK) { return st; }
    dev->dbg.int_enable_1 = tmp[0];

    st = icm_read_regs(dev, ICM20948_REG_INT_STATUS_1, tmp, 1u);
    if (st != ICM20948_OK) { return st; }
    dev->dbg.int_status_1 = tmp[0];

    st = icm_set_bank(dev, ICM20948_BANK_2);
    if (st != ICM20948_OK)
    {
        return st;
    }

    st = icm_read_regs(dev, ICM20948_REG_GYRO_SMPLRT_DIV, tmp, 1u);
    if (st != ICM20948_OK) { return st; }
    dev->dbg.gyro_smplrt_div = tmp[0];

    st = icm_read_regs(dev, ICM20948_REG_GYRO_CONFIG_1, tmp, 1u);
    if (st != ICM20948_OK) { return st; }
    dev->dbg.gyro_config_1 = tmp[0];

    st = icm_read_regs(dev, ICM20948_REG_ODR_ALIGN_EN, tmp, 1u);
    if (st != ICM20948_OK) { return st; }
    dev->dbg.odr_align_en = tmp[0];

    st = icm_read_regs(dev, ICM20948_REG_ACCEL_SMPLRT_DIV_1, tmp, 1u);
    if (st != ICM20948_OK) { return st; }
    dev->dbg.accel_smplrt_div_1 = tmp[0];

    st = icm_read_regs(dev, ICM20948_REG_ACCEL_SMPLRT_DIV_2, tmp, 1u);
    if (st != ICM20948_OK) { return st; }
    dev->dbg.accel_smplrt_div_2 = tmp[0];

    st = icm_read_regs(dev, ICM20948_REG_ACCEL_CONFIG, tmp, 1u);
    if (st != ICM20948_OK) { return st; }
    dev->dbg.accel_config = tmp[0];

    st = icm_set_bank(dev, ICM20948_BANK_0);
    if (st != ICM20948_OK)
    {
        return st;
    }

    return ICM20948_OK;
}

ICM20948_Status_t ICM20948_CalGyro(ICM20948_t* dev)
{
    uint32_t i;
    uint32_t wait_guard;
    int64_t sum_x;
    int64_t sum_y;
    int64_t sum_z;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    uint64_t t0;

    if (dev == 0)
    {
        return ICM20948_ERR_NULL_PTR;
    }
    if (dev->raw_valid == 0u)
    {
        return ICM20948_NO_NEW_SAMPLE;
    }

    dev->cal6.gyro_cal_started = 1u;
    dev->cal6.gyro_cal_active = 1u;
    dev->cal6.gyro_cal_done = 0u;
    dev->cal6.gyro_cal_progress = 0u;
    dev->cal6.gyro_sum_rads[0] = 0.0;
    dev->cal6.gyro_sum_rads[1] = 0.0;
    dev->cal6.gyro_sum_rads[2] = 0.0;
    dev->cal6.gyro_cal_step = ICM20948_CAL_STEP_NONE;

    sum_x = 0;
    sum_y = 0;
    sum_z = 0;

    for (i = 0u; i < (uint32_t)ICM20948_GYRO_CAL_SAMPLES_DEFAULT; i++)
    {
        t0 = dev->raw.timestamp_us;
        wait_guard = 0u;

        while (dev->raw.timestamp_us == t0)
        {
            wait_guard++;
            if (wait_guard > 2000000u)
            {
                dev->cal6.gyro_cal_active = 0u;
                return ICM20948_BUSY;
            }
        }

        gx = dev->raw.gyro_raw[0];
        gy = dev->raw.gyro_raw[1];
        gz = dev->raw.gyro_raw[2];

        sum_x += (int64_t)gx;
        sum_y += (int64_t)gy;
        sum_z += (int64_t)gz;

        dev->cal6.gyro_cal_progress = (uint16_t)(i + 1u);
    }

    dev->cal.gyro_bias_rads[0] = ((((float)sum_x) / (float)ICM20948_GYRO_CAL_SAMPLES_DEFAULT) / dev->gyro_lsb_per_dps) * ICM20948_DEG_TO_RAD;
    dev->cal.gyro_bias_rads[1] = ((((float)sum_y) / (float)ICM20948_GYRO_CAL_SAMPLES_DEFAULT) / dev->gyro_lsb_per_dps) * ICM20948_DEG_TO_RAD;
    dev->cal.gyro_bias_rads[2] = ((((float)sum_z) / (float)ICM20948_GYRO_CAL_SAMPLES_DEFAULT) / dev->gyro_lsb_per_dps) * ICM20948_DEG_TO_RAD;

    dev->cal6.gyro_cal_done = 1u;
    dev->cal6.gyro_cal_active = 0u;

    return ICM20948_OK;
}

ICM20948_Status_t ICM20948_CalReset(ICM20948_t* dev)
{
    if (dev == 0)
    {
        return ICM20948_ERR_NULL_PTR;
    }

    dev->cal6.state = ICM20948_CAL_STATE_IDLE;
    dev->cal6.done_mask = 0u;
    dev->cal6.retry_count = 0u;
    dev->cal6.active_step = ICM20948_CAL_STEP_NONE;
    dev->cal6.last_auto_step = ICM20948_CAL_STEP_NONE;
    dev->cal6.last_reset_reason = ICM20948_CAL_RESET_NONE;
    dev->cal6.gyro_cal_done = 0u;
    dev->cal6.gyro_cal_started = 0u;
    dev->cal6.gyro_cal_active = 0u;
    dev->cal6.gyro_cal_step = ICM20948_CAL_STEP_NONE;
    dev->cal6.gyro_cal_progress = 0u;
    dev->cal6.gyro_sum_rads[0] = 0.0;
    dev->cal6.gyro_sum_rads[1] = 0.0;
    dev->cal6.gyro_sum_rads[2] = 0.0;
    icm_cal_reset_accum(dev, ICM20948_CAL_RESET_NONE);

    return ICM20948_OK;
}

ICM20948_Status_t ICM20948_CalSetStep(ICM20948_t* dev)
{
    float ax_g;
    float ay_g;
    float az_g;
    ICM20948_CalStep_t step;

    if (dev == 0)
    {
        return ICM20948_ERR_NULL_PTR;
    }
    if (dev->raw_valid == 0u)
    {
        return ICM20948_NO_NEW_SAMPLE;
    }
    if ((dev->cal6.state == ICM20948_CAL_STATE_DONE) && (ICM20948_CAL_SETSTEP_LOCK_WHEN_DONE != 0u))
    {
        return ICM20948_OK;
    }

    ax_g = ((float)dev->raw.accel_raw[0]) / dev->accel_lsb_per_g;
    ay_g = ((float)dev->raw.accel_raw[1]) / dev->accel_lsb_per_g;
    az_g = ((float)dev->raw.accel_raw[2]) / dev->accel_lsb_per_g;

    step = icm_autostep_from_accel_g(ax_g, ay_g, az_g);
    dev->cal6.last_auto_step = step;

    if ((dev->cal6.done_mask & (uint8_t)(1u << (uint8_t)step)) != 0u)
    {
        if (ICM20948_CAL_OVERWRITE_EXISTING_STEP == 0u)
        {
            return ICM20948_OK;
        }
        dev->cal6.done_mask &= (uint8_t)(~(uint8_t)(1u << (uint8_t)step));
    }

    dev->cal6.active_step = step;
    dev->cal6.state = ICM20948_CAL_STATE_ACCUM;
    dev->cal6.retry_count = 0u;
    // В версии 1.0.14 CalSetStep не запускает калибровку гироскопа автоматически.

    icm_cal_reset_accum(dev, ICM20948_CAL_RESET_NONE);

    return ICM20948_OK;
}

ICM20948_Status_t ICM20948_CalGetStatus(ICM20948_t* dev, ICM20948_CalStatus_t* out)
{
    if ((dev == 0) || (out == 0))
    {
        return ICM20948_ERR_NULL_PTR;
    }

    out->done_mask = dev->cal6.done_mask;
    out->cal_done = (uint8_t)((dev->cal6.state == ICM20948_CAL_STATE_DONE) ? 1u : 0u);

    out->active_step = (uint8_t)dev->cal6.active_step;
    out->progress = dev->cal6.progress;

    out->retry_count = dev->cal6.retry_count;
    out->last_auto_step = (uint8_t)dev->cal6.last_auto_step;
    out->last_reset_reason = (uint8_t)dev->cal6.last_reset_reason;

    return ICM20948_OK;
}

ICM20948_Status_t ICM20948_CalGetResult(ICM20948_t* dev, ICM20948_CalResult_t* out)
{
    if ((dev == 0) || (out == 0))
    {
        return ICM20948_ERR_NULL_PTR;
    }

    out->valid = (uint8_t)((dev->cal6.state == ICM20948_CAL_STATE_DONE) ? 1u : 0u);

    out->acc_bias_g[0] = dev->cal.acc_bias_g[0];
    out->acc_bias_g[1] = dev->cal.acc_bias_g[1];
    out->acc_bias_g[2] = dev->cal.acc_bias_g[2];

    out->acc_scale[0] = dev->cal.acc_scale[0];
    out->acc_scale[1] = dev->cal.acc_scale[1];
    out->acc_scale[2] = dev->cal.acc_scale[2];

    return ICM20948_OK;
}
