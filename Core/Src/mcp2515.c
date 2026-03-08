
#include "mcp2515.h"
#include <string.h>

static uint64_t mcp2515_tick64_get(mcp2515_t *dev);
static void mcp2515_encode_std_id(uint16_t can_id, uint8_t *sidh, uint8_t *sidl);
static mcp2515_status_t mcp2515_get_cnf(uint32_t osc_hz, uint32_t bitrate, uint8_t *cnf1, uint8_t *cnf2, uint8_t *cnf3);
static mcp2515_status_t mcp2515_spi_blocking(mcp2515_t *dev, uint16_t len);
static mcp2515_status_t mcp2515_read_reg(mcp2515_t *dev, uint8_t reg, uint8_t *value);
static mcp2515_status_t mcp2515_write_reg(mcp2515_t *dev, uint8_t reg, uint8_t value);
static mcp2515_status_t mcp2515_write_regs4(mcp2515_t *dev, uint8_t reg, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3);
static mcp2515_status_t mcp2515_bit_modify(mcp2515_t *dev, uint8_t reg, uint8_t mask, uint8_t value);
static mcp2515_status_t mcp2515_set_mode(mcp2515_t *dev, mcp2515_mode_t mode);
static mcp2515_status_t mcp2515_start_dma(mcp2515_t *dev, mcp2515_op_t op, uint16_t len);
static uint32_t mcp2515_get_tim_clk(TIM_HandleTypeDef *htim);
static void mcp2515_fifo_push(mcp2515_t *dev, const mcp2515_sample_t *sample);
static void mcp2515_process_rx_frame(mcp2515_t *dev, uint8_t is_rxb1);

/// Полная инициализация драйвера.
mcp2515_status_t mcp2515_init(mcp2515_t *dev)
{
    uint8_t cnf1;
    uint8_t cnf2;
    uint8_t cnf3;
    uint8_t canstat;
    uint8_t sidh;
    uint8_t sidl;
    uint32_t tim_clk_hz;
    uint32_t tim_tick_hz;
    uint32_t sched_rate_hz;
    uint32_t arr_value;
    mcp2515_status_t st;

    if ((dev == NULL) || (dev->hspi == NULL) || (dev->htim_sched == NULL) ||
        (dev->cs_port == NULL) || (dev->int_port == NULL))
    {
        return MCP2515_STATUS_BAD_PARAM;
    }

    if ((dev->node_id_1 > 63U) || (dev->node_id_2 > 63U) || (dev->node_id_1 == dev->node_id_2))
    {
        return MCP2515_STATUS_BAD_PARAM;
    }

    if ((dev->cpu_hz == 0U) || (dev->poll_rate_hz_per_node == 0U) || (dev->response_timeout_us == 0U))
    {
        return MCP2515_STATUS_BAD_PARAM;
    }

    st = mcp2515_get_cnf(dev->mcp2515_osc_hz, dev->can_bitrate, &cnf1, &cnf2, &cnf3);
    if (st != MCP2515_STATUS_OK)
    {
        return st;
    }

    dev->driver_version = MCP2515_DRIVER_VERSION_U32;
    dev->can_id_1 = (uint16_t)(((uint16_t)dev->node_id_1 << 5) | MCP2515_ODRIVE_CMD_GET_ENCODER_ESTIMATES);
    dev->can_id_2 = (uint16_t)(((uint16_t)dev->node_id_2 << 5) | MCP2515_ODRIVE_CMD_GET_ENCODER_ESTIMATES);

    dev->spi_len = 0U;
    dev->is_initialized = 0U;
    dev->spi_busy = 0U;
    dev->spi_done = 0U;
    dev->spi_error = 0U;
    dev->int_pending = 0U;
    dev->sched_tick = 0U;
    dev->wait_response = 0U;
    dev->rx_fifo_head = 0U;
    dev->rx_fifo_tail = 0U;
    dev->rx_fifo_count = 0U;
    dev->rx_lost_cnt = 0U;
    dev->rx_ok_cnt = 0U;
    dev->timeout_cnt = 0U;
    dev->irq_cnt = 0U;
    dev->dma_err_cnt = 0U;
    dev->sched_slot = 0U;
    dev->req_tick64 = 0U;
    dev->irq_tick64 = 0U;
    dev->tick_hi32 = 0U;
    dev->tick_last32 = 0U;
    dev->op = MCP2515_OP_NONE;
    dev->last_status = MCP2515_STATUS_OK;
    (void)memset(dev->tx_buf, 0, sizeof(dev->tx_buf));
    (void)memset(dev->rx_buf, 0, sizeof(dev->rx_buf));
    (void)memset(dev->rx_fifo, 0, sizeof(dev->rx_fifo));
    (void)memset(&dev->sample_out, 0, sizeof(dev->sample_out));

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    HAL_Delay(MCP2515_RESET_DELAY_MS);

    dev->tx_buf[0] = MCP2515_CMD_RESET;
    dev->spi_len = 1U;
    st = mcp2515_spi_blocking(dev, dev->spi_len);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    HAL_Delay(MCP2515_RESET_DELAY_MS);

    st = mcp2515_read_reg(dev, MCP2515_REG_CANSTAT, &canstat);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    if ((canstat & MCP2515_CANSTAT_OPMOD_MASK) != MCP2515_CANCTRL_REQOP_CONFIG)
    {
        dev->last_status = MCP2515_STATUS_ERROR;
        return MCP2515_STATUS_ERROR;
    }

    st = mcp2515_write_reg(dev, MCP2515_REG_CNF1, cnf1);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    st = mcp2515_write_reg(dev, MCP2515_REG_CNF2, cnf2);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    st = mcp2515_write_reg(dev, MCP2515_REG_CNF3, cnf3);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    st = mcp2515_write_reg(dev, MCP2515_REG_RXB0CTRL, 0x00U);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    st = mcp2515_write_reg(dev, MCP2515_REG_RXB1CTRL, 0x00U);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    st = mcp2515_write_regs4(dev, MCP2515_REG_RXM0SIDH, 0xFFU, 0xE0U, 0x00U, 0x00U);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    st = mcp2515_write_regs4(dev, MCP2515_REG_RXM1SIDH, 0xFFU, 0xE0U, 0x00U, 0x00U);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    mcp2515_encode_std_id(dev->can_id_1, &sidh, &sidl);
    st = mcp2515_write_regs4(dev, MCP2515_REG_RXF0SIDH, sidh, sidl, 0x00U, 0x00U);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    st = mcp2515_write_regs4(dev, MCP2515_REG_RXF1SIDH, sidh, sidl, 0x00U, 0x00U);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    mcp2515_encode_std_id(dev->can_id_2, &sidh, &sidl);
    st = mcp2515_write_regs4(dev, MCP2515_REG_RXF2SIDH, sidh, sidl, 0x00U, 0x00U);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    st = mcp2515_write_regs4(dev, MCP2515_REG_RXF3SIDH, sidh, sidl, 0x00U, 0x00U);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    st = mcp2515_write_regs4(dev, MCP2515_REG_RXF4SIDH, sidh, sidl, 0x00U, 0x00U);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    st = mcp2515_write_regs4(dev, MCP2515_REG_RXF5SIDH, sidh, sidl, 0x00U, 0x00U);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    st = mcp2515_write_reg(dev, MCP2515_REG_CANINTF, 0x00U);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    st = mcp2515_write_reg(dev, MCP2515_REG_CANINTE, MCP2515_CANINTE_RX0IE | MCP2515_CANINTE_RX1IE);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    st = mcp2515_set_mode(dev, dev->can_mode);
    if (st != MCP2515_STATUS_OK)
    {
        dev->last_status = st;
        return st;
    }

    if (dev->can_mode != MCP2515_MODE_CONFIG)
    {
        sched_rate_hz = (uint32_t)dev->poll_rate_hz_per_node * MCP2515_SCHED_NODE_COUNT;
        tim_clk_hz = mcp2515_get_tim_clk(dev->htim_sched);
        tim_tick_hz = tim_clk_hz / (dev->htim_sched->Init.Prescaler + 1U);
        if ((sched_rate_hz == 0U) || (tim_tick_hz == 0U) || (tim_tick_hz < sched_rate_hz))
        {
            dev->last_status = MCP2515_STATUS_BAD_PARAM;
            return MCP2515_STATUS_BAD_PARAM;
        }

        arr_value = (tim_tick_hz / sched_rate_hz) - 1U;
        __HAL_TIM_SET_AUTORELOAD(dev->htim_sched, arr_value);
        __HAL_TIM_SET_COUNTER(dev->htim_sched, 0U);
        __HAL_TIM_CLEAR_FLAG(dev->htim_sched, TIM_FLAG_UPDATE);

        dev->is_initialized = 1U;
        if (HAL_TIM_Base_Start_IT(dev->htim_sched) != HAL_OK)
        {
            dev->is_initialized = 0U;
            dev->last_status = MCP2515_STATUS_HAL_ERROR;
            return MCP2515_STATUS_HAL_ERROR;
        }
    }
    else
    {
        dev->is_initialized = 1U;
    }
    dev->last_status = MCP2515_STATUS_OK;
    return MCP2515_STATUS_OK;
}

/// Обслуживание автомата драйвера.
mcp2515_status_t mcp2515_poll(mcp2515_t *dev)
{
    uint64_t now_tick64;
    uint64_t timeout_ticks;
    uint32_t primask;
    mcp2515_status_t st;

    if ((dev == NULL) || (dev->is_initialized == 0U))
    {
        return MCP2515_STATUS_NOT_READY;
    }

    if (dev->spi_error != 0U)
    {
        primask = __get_PRIMASK();
        __disable_irq();
        dev->spi_error = 0U;
        dev->spi_busy = 0U;
        dev->op = MCP2515_OP_NONE;
        __set_PRIMASK(primask);

        dev->dma_err_cnt++;
        dev->wait_response = 0U;
        dev->last_status = MCP2515_STATUS_HAL_ERROR;
        return MCP2515_STATUS_HAL_ERROR;
    }

    now_tick64 = mcp2515_tick64_get(dev);
    timeout_ticks = ((uint64_t)dev->cpu_hz * (uint64_t)dev->response_timeout_us) / 1000000ULL;
    if ((dev->wait_response != 0U) && ((now_tick64 - dev->req_tick64) > timeout_ticks))
    {
        dev->wait_response = 0U;
        dev->timeout_cnt++;
    }

    if ((dev->spi_done != 0U) && (dev->spi_busy == 0U))
    {
        primask = __get_PRIMASK();
        __disable_irq();
        dev->spi_done = 0U;
        __set_PRIMASK(primask);

        switch (dev->op)
        {
            case MCP2515_OP_TXB0_LOAD_RTR:
                dev->tx_buf[0] = MCP2515_CMD_RTS_TXB0;
                st = mcp2515_start_dma(dev, MCP2515_OP_TXB0_RTS, 1U);
                if (st != MCP2515_STATUS_OK)
                {
                    dev->last_status = st;
                    return st;
                }
                return MCP2515_STATUS_OK;

            case MCP2515_OP_TXB0_RTS:
                dev->wait_response = 1U;
                dev->op = MCP2515_OP_NONE;
                return MCP2515_STATUS_OK;

            case MCP2515_OP_READ_CANINTF:
                if ((dev->rx_buf[2] & MCP2515_CANINTF_RX0IF) != 0U)
                {
                    dev->tx_buf[0] = MCP2515_CMD_READ;
                    dev->tx_buf[1] = MCP2515_REG_RXB0CTRL;
                    (void)memset(&dev->tx_buf[2], 0, 14U);
                    st = mcp2515_start_dma(dev, MCP2515_OP_READ_RXB0, 16U);
                    if (st != MCP2515_STATUS_OK)
                    {
                        dev->last_status = st;
                        return st;
                    }
                    return MCP2515_STATUS_OK;
                }

                if ((dev->rx_buf[2] & MCP2515_CANINTF_RX1IF) != 0U)
                {
                    dev->tx_buf[0] = MCP2515_CMD_READ;
                    dev->tx_buf[1] = MCP2515_REG_RXB1CTRL;
                    (void)memset(&dev->tx_buf[2], 0, 14U);
                    st = mcp2515_start_dma(dev, MCP2515_OP_READ_RXB1, 16U);
                    if (st != MCP2515_STATUS_OK)
                    {
                        dev->last_status = st;
                        return st;
                    }
                    return MCP2515_STATUS_OK;
                }

                dev->int_pending = 0U;
                dev->op = MCP2515_OP_NONE;
                return MCP2515_STATUS_OK;

            case MCP2515_OP_READ_RXB0:
                mcp2515_process_rx_frame(dev, 0U);
                dev->tx_buf[0] = MCP2515_CMD_BIT_MODIFY;
                dev->tx_buf[1] = MCP2515_REG_CANINTF;
                dev->tx_buf[2] = MCP2515_CANINTF_RX0IF;
                dev->tx_buf[3] = 0x00U;
                st = mcp2515_start_dma(dev, MCP2515_OP_CLEAR_RX0IF, 4U);
                if (st != MCP2515_STATUS_OK)
                {
                    dev->last_status = st;
                    return st;
                }
                return MCP2515_STATUS_OK;

            case MCP2515_OP_READ_RXB1:
                mcp2515_process_rx_frame(dev, 1U);
                dev->tx_buf[0] = MCP2515_CMD_BIT_MODIFY;
                dev->tx_buf[1] = MCP2515_REG_CANINTF;
                dev->tx_buf[2] = MCP2515_CANINTF_RX1IF;
                dev->tx_buf[3] = 0x00U;
                st = mcp2515_start_dma(dev, MCP2515_OP_CLEAR_RX1IF, 4U);
                if (st != MCP2515_STATUS_OK)
                {
                    dev->last_status = st;
                    return st;
                }
                return MCP2515_STATUS_OK;

            case MCP2515_OP_CLEAR_RX0IF:
            case MCP2515_OP_CLEAR_RX1IF:
                dev->op = MCP2515_OP_NONE;
                if (HAL_GPIO_ReadPin(dev->int_port, dev->int_pin) == GPIO_PIN_RESET)
                {
                    dev->int_pending = 1U;
                }
                else
                {
                    dev->int_pending = 0U;
                }
                return MCP2515_STATUS_OK;

            case MCP2515_OP_NONE:
            default:
                dev->op = MCP2515_OP_NONE;
                return MCP2515_STATUS_OK;
        }
    }

    if (dev->spi_busy != 0U)
    {
        return MCP2515_STATUS_OK;
    }

    if (dev->int_pending != 0U)
    {
        dev->tx_buf[0] = MCP2515_CMD_READ;
        dev->tx_buf[1] = MCP2515_REG_CANINTF;
        dev->tx_buf[2] = 0x00U;
        st = mcp2515_start_dma(dev, MCP2515_OP_READ_CANINTF, 3U);
        if (st != MCP2515_STATUS_OK)
        {
            dev->last_status = st;
            return st;
        }
        return MCP2515_STATUS_OK;
    }

    if ((dev->sched_tick != 0U) && (dev->wait_response == 0U))
    {
        uint16_t can_id;
        uint8_t sidh;
        uint8_t sidl;

        primask = __get_PRIMASK();
        __disable_irq();
        dev->sched_tick = 0U;
        __set_PRIMASK(primask);

        if (dev->sched_slot == 0U)
        {
            can_id = dev->can_id_1;
            dev->sched_slot = 1U;
        }
        else
        {
            can_id = dev->can_id_2;
            dev->sched_slot = 0U;
        }

        mcp2515_encode_std_id(can_id, &sidh, &sidl);
        dev->tx_buf[0] = MCP2515_CMD_WRITE;
        dev->tx_buf[1] = MCP2515_REG_TXB0SIDH;
        dev->tx_buf[2] = sidh;
        dev->tx_buf[3] = sidl;
        dev->tx_buf[4] = 0x00U;
        dev->tx_buf[5] = 0x00U;
        dev->tx_buf[6] = MCP2515_TXB0DLC_RTR | 8U;

        dev->req_tick64 = now_tick64;
        st = mcp2515_start_dma(dev, MCP2515_OP_TXB0_LOAD_RTR, 7U);
        if (st != MCP2515_STATUS_OK)
        {
            dev->last_status = st;
            return st;
        }
        return MCP2515_STATUS_OK;
    }

    dev->last_status = MCP2515_STATUS_OK;
    return MCP2515_STATUS_OK;
}

/// Извлечение одного sample из FIFO.
mcp2515_status_t mcp2515_pop_sample(mcp2515_t *dev)
{
    uint32_t primask;

    if ((dev == NULL) || (dev->is_initialized == 0U))
    {
        return MCP2515_STATUS_NOT_READY;
    }

    if (dev->rx_fifo_count == 0U)
    {
        return MCP2515_STATUS_BUSY;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    dev->sample_out = dev->rx_fifo[dev->rx_fifo_tail];
    dev->rx_fifo_tail++;
    if (dev->rx_fifo_tail >= MCP2515_RX_FIFO_LEN)
    {
        dev->rx_fifo_tail = 0U;
    }
    dev->rx_fifo_count--;
    __set_PRIMASK(primask);

    return MCP2515_STATUS_OK;
}

/// Вызов из HAL_GPIO_EXTI_Callback.
void mcp2515_exti_callback(mcp2515_t *dev, uint16_t gpio_pin)
{
    if ((dev == NULL) || (dev->is_initialized == 0U))
    {
        return;
    }

    if ((gpio_pin == dev->int_pin) && (HAL_GPIO_ReadPin(dev->int_port, dev->int_pin) == GPIO_PIN_RESET))
    {
        dev->irq_tick64 = mcp2515_tick64_get(dev);
        dev->irq_cnt++;
        dev->int_pending = 1U;
    }
}

/// Вызов из HAL_SPI_TxRxCpltCallback.
void mcp2515_spi_txcplt_callback(mcp2515_t *dev, SPI_HandleTypeDef *hspi)
{
    if ((dev == NULL) || (dev->is_initialized == 0U))
    {
        return;
    }

    if (hspi == dev->hspi)
    {
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
        dev->spi_busy = 0U;
        dev->spi_done = 1U;
    }
}

/// Вызов из HAL_SPI_ErrorCallback.
void mcp2515_spi_error_callback(mcp2515_t *dev, SPI_HandleTypeDef *hspi)
{
    if ((dev == NULL) || (dev->is_initialized == 0U))
    {
        return;
    }

    if (hspi == dev->hspi)
    {
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
        dev->spi_busy = 0U;
        dev->spi_error = 1U;
    }
}

/// Вызов из HAL_TIM_PeriodElapsedCallback.
void mcp2515_tim_period_elapsed_callback(mcp2515_t *dev, TIM_HandleTypeDef *htim)
{
    if ((dev == NULL) || (dev->is_initialized == 0U))
    {
        return;
    }

    if (htim == dev->htim_sched)
    {
        dev->sched_tick = 1U;
    }
}

static uint64_t mcp2515_tick64_get(mcp2515_t *dev)
{
    uint32_t primask;
    uint32_t tick32;
    uint64_t tick64;

    primask = __get_PRIMASK();
    __disable_irq();

    tick32 = DWT->CYCCNT;
    if (tick32 < dev->tick_last32)
    {
        dev->tick_hi32++;
    }

    dev->tick_last32 = tick32;
    tick64 = ((uint64_t)dev->tick_hi32 << 32) | tick32;

    __set_PRIMASK(primask);
    return tick64;
}

static void mcp2515_encode_std_id(uint16_t can_id, uint8_t *sidh, uint8_t *sidl)
{
    *sidh = (uint8_t)(can_id >> 3);
    *sidl = (uint8_t)((can_id & 0x07U) << 5);
}

static mcp2515_status_t mcp2515_get_cnf(uint32_t osc_hz, uint32_t bitrate, uint8_t *cnf1, uint8_t *cnf2, uint8_t *cnf3)
{
    if ((cnf1 == NULL) || (cnf2 == NULL) || (cnf3 == NULL))
    {
        return MCP2515_STATUS_BAD_PARAM;
    }

    *cnf2 = 0x98U;
    *cnf3 = 0x01U;

    if ((osc_hz == 8000000U) && (bitrate == 125000U))
    {
        *cnf1 = 0x03U;
        return MCP2515_STATUS_OK;
    }

    if ((osc_hz == 8000000U) && (bitrate == 250000U))
    {
        *cnf1 = 0x01U;
        return MCP2515_STATUS_OK;
    }

    if ((osc_hz == 8000000U) && (bitrate == 500000U))
    {
        *cnf1 = 0x00U;
        return MCP2515_STATUS_OK;
    }

    if ((osc_hz == 16000000U) && (bitrate == 250000U))
    {
        *cnf1 = 0x03U;
        return MCP2515_STATUS_OK;
    }

    if ((osc_hz == 16000000U) && (bitrate == 500000U))
    {
        *cnf1 = 0x01U;
        return MCP2515_STATUS_OK;
    }

    if ((osc_hz == 16000000U) && (bitrate == 1000000U))
    {
        *cnf1 = 0x00U;
        return MCP2515_STATUS_OK;
    }

    return MCP2515_STATUS_BAD_PARAM;
}

static mcp2515_status_t mcp2515_spi_blocking(mcp2515_t *dev, uint16_t len)
{
    if ((len == 0U) || (len > MCP2515_SPI_FRAME_MAX_LEN))
    {
        return MCP2515_STATUS_BAD_PARAM;
    }

    (void)memset(dev->rx_buf, 0, sizeof(dev->rx_buf));
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive(dev->hspi, dev->tx_buf, dev->rx_buf, len, MCP2515_SPI_TIMEOUT_MS) != HAL_OK)
    {
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
        return MCP2515_STATUS_HAL_ERROR;
    }
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    return MCP2515_STATUS_OK;
}

static mcp2515_status_t mcp2515_read_reg(mcp2515_t *dev, uint8_t reg, uint8_t *value)
{
    mcp2515_status_t st;

    if (value == NULL)
    {
        return MCP2515_STATUS_BAD_PARAM;
    }

    dev->tx_buf[0] = MCP2515_CMD_READ;
    dev->tx_buf[1] = reg;
    dev->tx_buf[2] = 0x00U;
    st = mcp2515_spi_blocking(dev, 3U);
    if (st != MCP2515_STATUS_OK)
    {
        return st;
    }

    *value = dev->rx_buf[2];
    return MCP2515_STATUS_OK;
}

static mcp2515_status_t mcp2515_write_reg(mcp2515_t *dev, uint8_t reg, uint8_t value)
{
    dev->tx_buf[0] = MCP2515_CMD_WRITE;
    dev->tx_buf[1] = reg;
    dev->tx_buf[2] = value;
    return mcp2515_spi_blocking(dev, 3U);
}

static mcp2515_status_t mcp2515_write_regs4(mcp2515_t *dev, uint8_t reg, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
    dev->tx_buf[0] = MCP2515_CMD_WRITE;
    dev->tx_buf[1] = reg;
    dev->tx_buf[2] = b0;
    dev->tx_buf[3] = b1;
    dev->tx_buf[4] = b2;
    dev->tx_buf[5] = b3;
    return mcp2515_spi_blocking(dev, 6U);
}

static mcp2515_status_t mcp2515_bit_modify(mcp2515_t *dev, uint8_t reg, uint8_t mask, uint8_t value)
{
    dev->tx_buf[0] = MCP2515_CMD_BIT_MODIFY;
    dev->tx_buf[1] = reg;
    dev->tx_buf[2] = mask;
    dev->tx_buf[3] = value;
    return mcp2515_spi_blocking(dev, 4U);
}

static mcp2515_status_t mcp2515_set_mode(mcp2515_t *dev, mcp2515_mode_t mode)
{
    uint8_t reqop;
    uint8_t canstat;
    mcp2515_status_t st;

    if (mode == MCP2515_MODE_CONFIG)
    {
        reqop = MCP2515_CANCTRL_REQOP_CONFIG;
    }
    else if (mode == MCP2515_MODE_LOOPBACK)
    {
        reqop = MCP2515_CANCTRL_REQOP_LOOPBACK;
    }
    else if (mode == MCP2515_MODE_NORMAL)
    {
        reqop = MCP2515_CANCTRL_REQOP_NORMAL;
    }
    else
    {
        return MCP2515_STATUS_BAD_PARAM;
    }

    st = mcp2515_bit_modify(dev, MCP2515_REG_CANCTRL, MCP2515_CANCTRL_REQOP_MASK, reqop);
    if (st != MCP2515_STATUS_OK)
    {
        return st;
    }

    HAL_Delay(1U);

    st = mcp2515_read_reg(dev, MCP2515_REG_CANSTAT, &canstat);
    if (st != MCP2515_STATUS_OK)
    {
        return st;
    }

    if ((canstat & MCP2515_CANSTAT_OPMOD_MASK) != reqop)
    {
        return MCP2515_STATUS_ERROR;
    }

    return MCP2515_STATUS_OK;
}

static mcp2515_status_t mcp2515_start_dma(mcp2515_t *dev, mcp2515_op_t op, uint16_t len)
{
    uint32_t primask;

    if ((len == 0U) || (len > MCP2515_SPI_FRAME_MAX_LEN))
    {
        return MCP2515_STATUS_BAD_PARAM;
    }

    if (dev->spi_busy != 0U)
    {
        return MCP2515_STATUS_BUSY;
    }

    (void)memset(dev->rx_buf, 0, sizeof(dev->rx_buf));
    dev->spi_len = len;
    dev->op = op;

    primask = __get_PRIMASK();
    __disable_irq();
    dev->spi_done = 0U;
    dev->spi_error = 0U;
    dev->spi_busy = 1U;
    __set_PRIMASK(primask);

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive_DMA(dev->hspi, dev->tx_buf, dev->rx_buf, len) != HAL_OK)
    {
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
        primask = __get_PRIMASK();
        __disable_irq();
        dev->spi_busy = 0U;
        __set_PRIMASK(primask);
        dev->op = MCP2515_OP_NONE;
        return MCP2515_STATUS_HAL_ERROR;
    }

    return MCP2515_STATUS_OK;
}

static uint32_t mcp2515_get_tim_clk(TIM_HandleTypeDef *htim)
{
    RCC_ClkInitTypeDef clk_cfg;
    uint32_t flash_latency;
    uint32_t pclk_hz;
    uint32_t apb_prescaler;

    HAL_RCC_GetClockConfig(&clk_cfg, &flash_latency);

    if ((htim->Instance == TIM1) ||
        (htim->Instance == TIM9) ||
        (htim->Instance == TIM10) ||
        (htim->Instance == TIM11))
    {
        pclk_hz = HAL_RCC_GetPCLK2Freq();
        apb_prescaler = clk_cfg.APB2CLKDivider;
    }
    else
    {
        pclk_hz = HAL_RCC_GetPCLK1Freq();
        apb_prescaler = clk_cfg.APB1CLKDivider;
    }

    if (apb_prescaler == RCC_HCLK_DIV1)
    {
        return pclk_hz;
    }

    return (pclk_hz * 2U);
}

static void mcp2515_fifo_push(mcp2515_t *dev, const mcp2515_sample_t *sample)
{
    uint32_t primask;
    uint8_t head;

    primask = __get_PRIMASK();
    __disable_irq();

    if (dev->rx_fifo_count >= MCP2515_RX_FIFO_LEN)
    {
        dev->rx_lost_cnt++;
        __set_PRIMASK(primask);
        return;
    }

    head = dev->rx_fifo_head;
    dev->rx_fifo[head] = *sample;
    head++;
    if (head >= MCP2515_RX_FIFO_LEN)
    {
        head = 0U;
    }

    dev->rx_fifo_head = head;
    dev->rx_fifo_count++;
    dev->rx_ok_cnt++;

    __set_PRIMASK(primask);
}

static void mcp2515_process_rx_frame(mcp2515_t *dev, uint8_t is_rxb1)
{
    mcp2515_sample_t sample;
    uint8_t ctrl;
    uint8_t sidh;
    uint8_t sidl;
    uint8_t dlc;
    uint16_t can_id;
    uint8_t node_id;
    float pos_rev;
    float vel_rev_s;

    if (is_rxb1 == 0U)
    {
        ctrl = dev->rx_buf[2];
    }
    else
    {
        ctrl = dev->rx_buf[2];
    }

    sidh = dev->rx_buf[3];
    sidl = dev->rx_buf[4];
    dlc = dev->rx_buf[7] & MCP2515_DLC_MASK;
    can_id = (uint16_t)(((uint16_t)sidh << 3) | ((uint16_t)sidl >> 5));

    if ((ctrl & MCP2515_RXB0CTRL_RXRTR) != 0U)
    {
        return;
    }

    if (dlc != 8U)
    {
        return;
    }

    if (can_id == dev->can_id_1)
    {
        node_id = dev->node_id_1;
    }
    else if (can_id == dev->can_id_2)
    {
        node_id = dev->node_id_2;
    }
    else
    {
        return;
    }

    (void)memcpy(&pos_rev, &dev->rx_buf[8], sizeof(float));
    (void)memcpy(&vel_rev_s, &dev->rx_buf[12], sizeof(float));

    sample.tick64 = dev->irq_tick64;
    sample.node_id = node_id;
    sample.position_rev = pos_rev;
    sample.velocity_rev_s = vel_rev_s;

    mcp2515_fifo_push(dev, &sample);
}
