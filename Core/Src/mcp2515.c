#include "mcp2515.h"
#include <string.h>

static uint64_t mcp2515_tick64_get(mcp2515_t *dev)
{
    uint32_t tick32;
    uint64_t tick64;

    if (dev == NULL)
    {
        return 0ULL;
    }

    __disable_irq();
    tick32 = DWT->CYCCNT;

    if (tick32 < dev->tick_last32)
    {
        dev->tick_hi32++;
    }

    dev->tick_last32 = tick32;
    tick64 = (((uint64_t)dev->tick_hi32) << 32) | tick32;
    __enable_irq();

    return tick64;
}

static HAL_StatusTypeDef mcp2515_spi_write_bytes(mcp2515_t *dev, const uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef hal_st;

    if ((dev == NULL) || (buf == NULL) || (len == 0U))
    {
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    hal_st = HAL_SPI_Transmit(dev->hspi, (uint8_t *)buf, len, MCP2515_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    return hal_st;
}

static HAL_StatusTypeDef mcp2515_spi_read_regs(mcp2515_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef hal_st;
    uint8_t hdr[2];

    if ((dev == NULL) || (data == NULL) || (len == 0U))
    {
        return HAL_ERROR;
    }

    hdr[0] = MCP2515_CMD_READ;
    hdr[1] = reg;

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);

    hal_st = HAL_SPI_Transmit(dev->hspi, hdr, 2U, MCP2515_SPI_TIMEOUT_MS);
    if (hal_st == HAL_OK)
    {
        hal_st = HAL_SPI_Receive(dev->hspi, data, len, MCP2515_SPI_TIMEOUT_MS);
    }

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    return hal_st;
}

static HAL_StatusTypeDef mcp2515_write_reg(mcp2515_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t tx[3];

    tx[0] = MCP2515_CMD_WRITE;
    tx[1] = reg;
    tx[2] = value;

    return mcp2515_spi_write_bytes(dev, tx, 3U);
}

static HAL_StatusTypeDef mcp2515_read_reg(mcp2515_t *dev, uint8_t reg, uint8_t *value)
{
    if ((dev == NULL) || (value == NULL))
    {
        return HAL_ERROR;
    }

    return mcp2515_spi_read_regs(dev, reg, value, 1U);
}

static HAL_StatusTypeDef mcp2515_bit_modify(mcp2515_t *dev, uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t tx[4];

    tx[0] = MCP2515_CMD_BIT_MODIFY;
    tx[1] = reg;
    tx[2] = mask;
    tx[3] = value;

    return mcp2515_spi_write_bytes(dev, tx, 4U);
}

static HAL_StatusTypeDef mcp2515_reset(mcp2515_t *dev)
{
    uint8_t cmd;

    if (dev == NULL)
    {
        return HAL_ERROR;
    }

    cmd = MCP2515_CMD_RESET;
    return mcp2515_spi_write_bytes(dev, &cmd, 1U);
}

static uint8_t mcp2515_mode_to_reg(mcp2515_mode_t mode)
{
    uint8_t value;

    switch (mode)
    {
        case MCP2515_MODE_CONFIG:
            value = MCP2515_CANCTRL_REQOP_CONFIG;
            break;

        case MCP2515_MODE_LOOPBACK:
            value = MCP2515_CANCTRL_REQOP_LOOPBACK;
            break;

        case MCP2515_MODE_LISTEN_ONLY:
            value = MCP2515_CANCTRL_REQOP_LISTEN_ONLY;
            break;

        case MCP2515_MODE_NORMAL:
        default:
            value = MCP2515_CANCTRL_REQOP_NORMAL;
            break;
    }

    return value;
}

static mcp2515_status_t mcp2515_set_mode(mcp2515_t *dev, mcp2515_mode_t mode)
{
    HAL_StatusTypeDef hal_st;
    uint8_t canstat;
    uint8_t reqop;
    uint32_t i;

    if (dev == NULL)
    {
        return MCP2515_STATUS_BAD_PARAM;
    }

    reqop = mcp2515_mode_to_reg(mode);

    hal_st = mcp2515_bit_modify(dev, MCP2515_REG_CANCTRL, MCP2515_CANSTAT_MODE_MASK, reqop);
    if (hal_st != HAL_OK)
    {
        return MCP2515_STATUS_HAL_ERROR;
    }

    for (i = 0U; i < 100U; i++)
    {
        hal_st = mcp2515_read_reg(dev, MCP2515_REG_CANSTAT, &canstat);
        if (hal_st != HAL_OK)
        {
            return MCP2515_STATUS_HAL_ERROR;
        }

        if ((canstat & MCP2515_CANSTAT_MODE_MASK) == reqop)
        {
            return MCP2515_STATUS_OK;
        }
    }

    return MCP2515_STATUS_TIMEOUT;
}

static mcp2515_status_t mcp2515_set_bitrate(mcp2515_t *dev)
{
    uint8_t cnf1;
    uint8_t cnf2;
    uint8_t cnf3;
    HAL_StatusTypeDef hal_st;

    if (dev == NULL)
    {
        return MCP2515_STATUS_BAD_PARAM;
    }

    cnf1 = 0U;
    cnf2 = 0U;
    cnf3 = 0U;

    if ((dev->mcp2515_osc_hz == 8000000U) && (dev->can_bitrate == 250000U))
    {
        cnf1 = 0x00U;
        cnf2 = 0xACU;
        cnf3 = 0x03U;
    }
    else if ((dev->mcp2515_osc_hz == 8000000U) && (dev->can_bitrate == 500000U))
    {
        cnf1 = 0x00U;
        cnf2 = 0x91U;
        cnf3 = 0x01U;
    }
    else if ((dev->mcp2515_osc_hz == 16000000U) && (dev->can_bitrate == 250000U))
    {
        cnf1 = 0x01U;
        cnf2 = 0xACU;
        cnf3 = 0x03U;
    }
    else if ((dev->mcp2515_osc_hz == 16000000U) && (dev->can_bitrate == 500000U))
    {
        cnf1 = 0x01U;
        cnf2 = 0x91U;
        cnf3 = 0x01U;
    }
    else if ((dev->mcp2515_osc_hz == 16000000U) && (dev->can_bitrate == 1000000U))
    {
        cnf1 = 0x00U;
        cnf2 = 0x91U;
        cnf3 = 0x01U;
    }
    else
    {
        return MCP2515_STATUS_UNSUPPORTED;
    }

    hal_st = mcp2515_write_reg(dev, MCP2515_REG_CNF1, cnf1);
    if (hal_st != HAL_OK)
    {
        return MCP2515_STATUS_HAL_ERROR;
    }

    hal_st = mcp2515_write_reg(dev, MCP2515_REG_CNF2, cnf2);
    if (hal_st != HAL_OK)
    {
        return MCP2515_STATUS_HAL_ERROR;
    }

    hal_st = mcp2515_write_reg(dev, MCP2515_REG_CNF3, cnf3);
    if (hal_st != HAL_OK)
    {
        return MCP2515_STATUS_HAL_ERROR;
    }

    return MCP2515_STATUS_OK;
}

mcp2515_status_t mcp2515_init(mcp2515_t *dev)
{
    HAL_StatusTypeDef hal_st;
    mcp2515_status_t st;

    if ((dev == NULL) ||
        (dev->hspi == NULL) ||
        (dev->cs_port == NULL) ||
        (dev->int_port == NULL) ||
        (dev->cpu_hz == 0U) ||
        (dev->mcp2515_osc_hz == 0U) ||
        (dev->can_bitrate == 0U) ||
        (dev->node_id_1 == 0U) ||
        (dev->node_id_2 == 0U) ||
        (dev->node_id_1 == dev->node_id_2))
    {
        return MCP2515_STATUS_BAD_PARAM;
    }

    dev->hb_id_1 = (uint16_t)MCP2515_ODRIVE_HEARTBEAT_ID(dev->node_id_1);
    dev->hb_id_2 = (uint16_t)MCP2515_ODRIVE_HEARTBEAT_ID(dev->node_id_2);

    dev->spi_busy = 0U;
    dev->spi_done = 0U;
    dev->spi_error = 0U;
    dev->int_pending = 0U;
    dev->sched_tick = 0U;
    dev->rx_fifo_count = 0U;
    dev->tick_hi32 = 0U;
    dev->tick_last32 = 0U;
    dev->irq_tick64 = 0ULL;
    dev->irq_cnt = 0U;
    dev->dma_err_cnt = 0U;
    dev->hb_rx_cnt = 0U;
    dev->hb_bad_dlc_cnt = 0U;
    dev->rx_ok_cnt = 0U;
    dev->rx_lost_cnt = 0U;
    dev->timeout_cnt = 0U;
    dev->rx_ignored_cnt = 0U;
    dev->sample_out.tick64 = 0ULL;
    dev->sample_out.node_id = 0U;
    dev->sample_out.position_rev = 0.0f;
    dev->sample_out.velocity_rev_s = 0.0f;
    memset(&dev->hb_1, 0, sizeof(dev->hb_1));
    memset(&dev->hb_2, 0, sizeof(dev->hb_2));
    dev->is_initialized = 0U;
    dev->last_status = MCP2515_STATUS_NOT_READY;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    HAL_Delay(MCP2515_RESET_DELAY_MS);

    hal_st = mcp2515_reset(dev);
    if (hal_st != HAL_OK)
    {
        dev->last_status = MCP2515_STATUS_HAL_ERROR;
        return dev->last_status;
    }

    HAL_Delay(MCP2515_RESET_DELAY_MS);

    st = mcp2515_set_mode(dev, MCP2515_MODE_CONFIG);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    st = mcp2515_set_bitrate(dev);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    hal_st = mcp2515_write_reg(dev, MCP2515_REG_BFPCTRL, 0x00U);
    if (hal_st != HAL_OK)
    {
        dev->last_status = MCP2515_STATUS_HAL_ERROR;
        return dev->last_status;
    }

    hal_st = mcp2515_write_reg(dev, MCP2515_REG_TXRTSCTRL, 0x00U);
    if (hal_st != HAL_OK)
    {
        dev->last_status = MCP2515_STATUS_HAL_ERROR;
        return dev->last_status;
    }

    hal_st = mcp2515_write_reg(dev, MCP2515_REG_RXB0CTRL, MCP2515_RXB0CTRL_ACCEPT_ALL);
    if (hal_st != HAL_OK)
    {
        dev->last_status = MCP2515_STATUS_HAL_ERROR;
        return dev->last_status;
    }

    hal_st = mcp2515_write_reg(dev, MCP2515_REG_RXB1CTRL, MCP2515_RXB1CTRL_ACCEPT_ALL);
    if (hal_st != HAL_OK)
    {
        dev->last_status = MCP2515_STATUS_HAL_ERROR;
        return dev->last_status;
    }

    hal_st = mcp2515_write_reg(dev, MCP2515_REG_CANINTF, 0x00U);
    if (hal_st != HAL_OK)
    {
        dev->last_status = MCP2515_STATUS_HAL_ERROR;
        return dev->last_status;
    }

    hal_st = mcp2515_write_reg(dev, MCP2515_REG_EFLG, 0x00U);
    if (hal_st != HAL_OK)
    {
        dev->last_status = MCP2515_STATUS_HAL_ERROR;
        return dev->last_status;
    }

    hal_st = mcp2515_write_reg(dev, MCP2515_REG_CANINTE, MCP2515_CANINTE_RX0IE | MCP2515_CANINTE_RX1IE);
    if (hal_st != HAL_OK)
    {
        dev->last_status = MCP2515_STATUS_HAL_ERROR;
        return dev->last_status;
    }

    st = mcp2515_set_mode(dev, dev->can_mode);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    if (dev->htim_sched != NULL)
    {
        (void)HAL_TIM_Base_Stop_IT(dev->htim_sched);
    }

    dev->is_initialized = 1U;
    dev->last_status = MCP2515_STATUS_OK;

    return MCP2515_STATUS_OK;
}

mcp2515_status_t mcp2515_poll(mcp2515_t *dev)
{
    HAL_StatusTypeDef hal_st;
    uint8_t canintf;
    uint8_t raw[13];
    uint16_t id;
    uint8_t dlc;
    uint8_t need_service;
    uint8_t handled;
    uint64_t frame_tick;

    if (dev == NULL)
    {
        return MCP2515_STATUS_BAD_PARAM;
    }

    if (dev->is_initialized == 0U)
    {
        dev->last_status = MCP2515_STATUS_NOT_READY;
        return dev->last_status;
    }

    need_service = 0U;

    __disable_irq();
    if (dev->int_pending != 0U)
    {
        dev->int_pending = 0U;
        need_service = 1U;
    }
    __enable_irq();

    if (HAL_GPIO_ReadPin(dev->int_port, dev->int_pin) == GPIO_PIN_RESET)
    {
        need_service = 1U;
    }

    if (need_service == 0U)
    {
        dev->last_status = MCP2515_STATUS_OK;
        return dev->last_status;
    }

    frame_tick = dev->irq_tick64;
    if (frame_tick == 0ULL)
    {
        frame_tick = mcp2515_tick64_get(dev);
    }

    while (1)
    {
        hal_st = mcp2515_read_reg(dev, MCP2515_REG_CANINTF, &canintf);
        if (hal_st != HAL_OK)
        {
            dev->last_status = MCP2515_STATUS_HAL_ERROR;
            return dev->last_status;
        }

        handled = 0U;

        if ((canintf & MCP2515_CANINTF_RX0IF) != 0U)
        {
            hal_st = mcp2515_spi_read_regs(dev, MCP2515_REG_RXB0SIDH, raw, 13U);
            if (hal_st != HAL_OK)
            {
                dev->last_status = MCP2515_STATUS_HAL_ERROR;
                return dev->last_status;
            }

            id = (((uint16_t)raw[0]) << 3) | (((uint16_t)raw[1]) >> 5);
            dlc = raw[4] & 0x0FU;

            if (dlc == MCP2515_HEARTBEAT_DLC)
            {
                if (id == dev->hb_id_1)
                {
                    dev->hb_1.tick64 = frame_tick;
                    dev->hb_1.axis_error = ((uint32_t)raw[5]) |
                                           ((uint32_t)raw[6] << 8) |
                                           ((uint32_t)raw[7] << 16) |
                                           ((uint32_t)raw[8] << 24);
                    dev->hb_1.axis_state = raw[9];
                    dev->hb_1.procedure_result = raw[10];
                    dev->hb_1.trajectory_done = raw[11];
                    dev->hb_1.valid = 1U;
                    dev->hb_1.rx_count++;
                    dev->hb_rx_cnt++;
                    dev->rx_ok_cnt++;
                }
                else if (id == dev->hb_id_2)
                {
                    dev->hb_2.tick64 = frame_tick;
                    dev->hb_2.axis_error = ((uint32_t)raw[5]) |
                                           ((uint32_t)raw[6] << 8) |
                                           ((uint32_t)raw[7] << 16) |
                                           ((uint32_t)raw[8] << 24);
                    dev->hb_2.axis_state = raw[9];
                    dev->hb_2.procedure_result = raw[10];
                    dev->hb_2.trajectory_done = raw[11];
                    dev->hb_2.valid = 1U;
                    dev->hb_2.rx_count++;
                    dev->hb_rx_cnt++;
                    dev->rx_ok_cnt++;
                }
                else
                {
                    dev->rx_ignored_cnt++;
                }
            }
            else
            {
                if ((id == dev->hb_id_1) || (id == dev->hb_id_2))
                {
                    dev->hb_bad_dlc_cnt++;
                }
                else
                {
                    dev->rx_ignored_cnt++;
                }
            }

            hal_st = mcp2515_bit_modify(dev, MCP2515_REG_CANINTF, MCP2515_CANINTF_RX0IF, 0x00U);
            if (hal_st != HAL_OK)
            {
                dev->last_status = MCP2515_STATUS_HAL_ERROR;
                return dev->last_status;
            }

            handled = 1U;
        }

        if ((canintf & MCP2515_CANINTF_RX1IF) != 0U)
        {
            hal_st = mcp2515_spi_read_regs(dev, MCP2515_REG_RXB1SIDH, raw, 13U);
            if (hal_st != HAL_OK)
            {
                dev->last_status = MCP2515_STATUS_HAL_ERROR;
                return dev->last_status;
            }

            id = (((uint16_t)raw[0]) << 3) | (((uint16_t)raw[1]) >> 5);
            dlc = raw[4] & 0x0FU;

            if (dlc == MCP2515_HEARTBEAT_DLC)
            {
                if (id == dev->hb_id_1)
                {
                    dev->hb_1.tick64 = frame_tick;
                    dev->hb_1.axis_error = ((uint32_t)raw[5]) |
                                           ((uint32_t)raw[6] << 8) |
                                           ((uint32_t)raw[7] << 16) |
                                           ((uint32_t)raw[8] << 24);
                    dev->hb_1.axis_state = raw[9];
                    dev->hb_1.procedure_result = raw[10];
                    dev->hb_1.trajectory_done = raw[11];
                    dev->hb_1.valid = 1U;
                    dev->hb_1.rx_count++;
                    dev->hb_rx_cnt++;
                    dev->rx_ok_cnt++;
                }
                else if (id == dev->hb_id_2)
                {
                    dev->hb_2.tick64 = frame_tick;
                    dev->hb_2.axis_error = ((uint32_t)raw[5]) |
                                           ((uint32_t)raw[6] << 8) |
                                           ((uint32_t)raw[7] << 16) |
                                           ((uint32_t)raw[8] << 24);
                    dev->hb_2.axis_state = raw[9];
                    dev->hb_2.procedure_result = raw[10];
                    dev->hb_2.trajectory_done = raw[11];
                    dev->hb_2.valid = 1U;
                    dev->hb_2.rx_count++;
                    dev->hb_rx_cnt++;
                    dev->rx_ok_cnt++;
                }
                else
                {
                    dev->rx_ignored_cnt++;
                }
            }
            else
            {
                if ((id == dev->hb_id_1) || (id == dev->hb_id_2))
                {
                    dev->hb_bad_dlc_cnt++;
                }
                else
                {
                    dev->rx_ignored_cnt++;
                }
            }

            hal_st = mcp2515_bit_modify(dev, MCP2515_REG_CANINTF, MCP2515_CANINTF_RX1IF, 0x00U);
            if (hal_st != HAL_OK)
            {
                dev->last_status = MCP2515_STATUS_HAL_ERROR;
                return dev->last_status;
            }

            handled = 1U;
        }

        if (handled == 0U)
        {
            break;
        }
    }

    dev->last_status = MCP2515_STATUS_OK;
    return dev->last_status;
}

mcp2515_status_t mcp2515_pop_sample(mcp2515_t *dev)
{
    if (dev == NULL)
    {
        return MCP2515_STATUS_BAD_PARAM;
    }

    dev->last_status = MCP2515_STATUS_NOT_READY;
    return dev->last_status;
}

void mcp2515_exti_callback(mcp2515_t *dev, uint16_t gpio_pin)
{
    if ((dev == NULL) || (gpio_pin != dev->int_pin))
    {
        return;
    }

    dev->irq_tick64 = mcp2515_tick64_get(dev);
    dev->irq_cnt++;
    dev->int_pending = 1U;
}

void mcp2515_spi_txcplt_callback(mcp2515_t *dev, SPI_HandleTypeDef *hspi)
{
    if ((dev == NULL) || (hspi == NULL) || (dev->hspi == NULL))
    {
        return;
    }

    if (hspi->Instance == dev->hspi->Instance)
    {
        dev->spi_done = 1U;
        dev->spi_busy = 0U;
    }
}

void mcp2515_spi_error_callback(mcp2515_t *dev, SPI_HandleTypeDef *hspi)
{
    if ((dev == NULL) || (hspi == NULL) || (dev->hspi == NULL))
    {
        return;
    }

    if (hspi->Instance == dev->hspi->Instance)
    {
        dev->spi_error = 1U;
        dev->spi_busy = 0U;
        dev->dma_err_cnt++;
    }
}

void mcp2515_tim_period_elapsed_callback(mcp2515_t *dev, TIM_HandleTypeDef *htim)
{
    if ((dev == NULL) || (htim == NULL) || (dev->htim_sched == NULL))
    {
        return;
    }

    if (htim->Instance == dev->htim_sched->Instance)
    {
        dev->sched_tick = 1U;
    }
}

mcp2515_status_t mcp2515_debug_read_reg(mcp2515_t *dev, uint8_t reg, uint8_t *value)
{
    uint8_t tx_buf[3];
    uint8_t rx_buf[3];
    HAL_StatusTypeDef hal_status;

    if ((dev == NULL) || (value == NULL) || (dev->hspi == NULL) ||
        (dev->cs_port == NULL))
    {
        return MCP2515_STATUS_BAD_PARAM;
    }

    tx_buf[0] = MCP2515_CMD_READ;
    tx_buf[1] = reg;
    tx_buf[2] = 0x00U;

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);

    hal_status = HAL_SPI_TransmitReceive(
        dev->hspi,
        tx_buf,
        rx_buf,
        3U,
        MCP2515_SPI_TIMEOUT_MS
    );

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    if (hal_status != HAL_OK)
    {
        return MCP2515_STATUS_HAL_ERROR;
    }

    *value = rx_buf[2];
    return MCP2515_STATUS_OK;
}
