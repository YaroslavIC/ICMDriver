#include "can_mcp2515_odrive.h"
#include <string.h>
#include <stdio.h>

#define MCP2515_CMD_RESET          0xC0U
#define MCP2515_CMD_READ           0x03U
#define MCP2515_CMD_WRITE          0x02U
#define MCP2515_CMD_BIT_MODIFY     0x05U
#define MCP2515_CMD_READ_RXB0      0x90U
#define MCP2515_CMD_READ_RXB1      0x94U
#define MCP2515_CMD_RTS_TXB0       0x81U

#define MCP2515_REG_CANSTAT        0x0EU
#define MCP2515_REG_CANCTRL        0x0FU
#define MCP2515_REG_CNF3           0x28U
#define MCP2515_REG_CNF2           0x29U
#define MCP2515_REG_CNF1           0x2AU
#define MCP2515_REG_CANINTE        0x2BU
#define MCP2515_REG_CANINTF        0x2CU
#define MCP2515_REG_EFLG           0x2DU
#define MCP2515_REG_TXB0CTRL       0x30U
#define MCP2515_REG_TXB0SIDH       0x31U
#define MCP2515_REG_RXB0CTRL       0x60U
#define MCP2515_REG_RXB1CTRL       0x70U

#define MCP2515_CANINTE_RX0IE      0x01U
#define MCP2515_CANINTE_RX1IE      0x02U
#define MCP2515_CANINTE_ERRIE      0x20U

#define MCP2515_CANINTF_RX0IF      0x01U
#define MCP2515_CANINTF_RX1IF      0x02U
#define MCP2515_CANINTF_TX0IF      0x04U
#define MCP2515_CANINTF_ERRIF      0x20U
#define MCP2515_CANINTF_MERRF      0x80U

#define MCP2515_CANCTRL_REQOP_MASK 0xE0U
#define MCP2515_CANCTRL_REQOP_NORM 0x00U
#define MCP2515_CANCTRL_REQOP_LIST 0x60U
#define MCP2515_CANCTRL_REQOP_CONF 0x80U

#define MCP2515_CANSTAT_OPMOD_MASK 0xE0U
#define MCP2515_TXBCTRL_TXREQ      0x08U

#define MCP2515_EFLG_RX0OVR        0x40U
#define MCP2515_EFLG_RX1OVR        0x80U
#define MCP2515_EFLG_TXBO          0x20U

#define CAN_MCP2515_ODRIVE_PHASE_NODE1 0U
#define CAN_MCP2515_ODRIVE_PHASE_NODE2 1U

static void can_mcp2515_odrive_cs_high(can_mcp2515_odrive_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static void can_mcp2515_odrive_cs_low(can_mcp2515_odrive_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static uint16_t can_mcp2515_odrive_make_can_id(uint8_t node_id, uint8_t cmd_id)
{
    return (uint16_t)(((uint16_t)node_id << 5) | (uint16_t)(cmd_id & 0x1FU));
}

static uint32_t can_mcp2515_odrive_u32_from_le(const uint8_t *src)
{
    return ((uint32_t)src[0]) |
           ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) |
           ((uint32_t)src[3] << 24);
}

static float can_mcp2515_odrive_f32_from_le(const uint8_t *src)
{
    union
    {
        uint32_t u32;
        float f32;
    } cvt;

    cvt.u32 = can_mcp2515_odrive_u32_from_le(src);
    return cvt.f32;
}

static void can_mcp2515_odrive_u32_to_le(uint32_t value, uint8_t *dst)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static void can_mcp2515_odrive_i16_to_le(int16_t value, uint8_t *dst)
{
    uint16_t u = (uint16_t)value;
    dst[0] = (uint8_t)(u & 0xFFU);
    dst[1] = (uint8_t)((u >> 8) & 0xFFU);
}

static void can_mcp2515_odrive_f32_to_le(float value, uint8_t *dst)
{
    union
    {
        uint32_t u32;
        float f32;
    } cvt;

    cvt.f32 = value;
    can_mcp2515_odrive_u32_to_le(cvt.u32, dst);
}

static void can_mcp2515_odrive_timebase_update(can_mcp2515_odrive_t *dev)
{
#if defined(DWT) && defined(CoreDebug)
    uint32_t now_low = DWT->CYCCNT;
    if (now_low < dev->diag.time_last_low32)
    {
        dev->diag.time_high32++;
    }
    dev->diag.time_last_low32 = now_low;
    dev->diag.time_cycles_64 = (((uint64_t)dev->diag.time_high32) << 32) | (uint64_t)now_low;
    if (SystemCoreClock != 0U)
    {
        dev->diag.time_us = (dev->diag.time_cycles_64 * 1000000ULL) / (uint64_t)SystemCoreClock;
    }
    else
    {
        dev->diag.time_us = (uint64_t)HAL_GetTick() * 1000ULL;
    }
#else
    dev->diag.time_cycles_64 = 0ULL;
    dev->diag.time_us = (uint64_t)HAL_GetTick() * 1000ULL;
#endif
}

static void can_mcp2515_odrive_timebase_init(can_mcp2515_odrive_t *dev)
{
#if defined(DWT) && defined(CoreDebug)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
    dev->diag.time_high32 = 0U;
    dev->diag.time_last_low32 = 0U;
    dev->diag.time_cycles_64 = 0ULL;
    dev->diag.time_us = 0ULL;
    can_mcp2515_odrive_timebase_update(dev);
}

static can_mcp2515_odrive_status_t can_mcp2515_odrive_spi_blocking_xfer(can_mcp2515_odrive_t *dev, const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    HAL_StatusTypeDef hal_status;

    can_mcp2515_odrive_cs_low(dev);
    hal_status = HAL_SPI_TransmitReceive(dev->hspi, (uint8_t *)tx, rx, len, 100U);
    can_mcp2515_odrive_cs_high(dev);

    if (hal_status != HAL_OK)
    {
        dev->diag.last_status = CAN_MCP2515_ODRIVE_STATUS_HAL_ERROR;
        return CAN_MCP2515_ODRIVE_STATUS_HAL_ERROR;
    }

    return CAN_MCP2515_ODRIVE_STATUS_OK;
}

static can_mcp2515_odrive_status_t can_mcp2515_odrive_write_reg(can_mcp2515_odrive_t *dev, uint8_t addr, uint8_t value)
{
    uint8_t tx[3];
    uint8_t rx[3];

    tx[0] = MCP2515_CMD_WRITE;
    tx[1] = addr;
    tx[2] = value;
    memset(rx, 0, sizeof(rx));
    return can_mcp2515_odrive_spi_blocking_xfer(dev, tx, rx, (uint16_t)sizeof(tx));
}

static can_mcp2515_odrive_status_t can_mcp2515_odrive_read_reg(can_mcp2515_odrive_t *dev, uint8_t addr, uint8_t *value)
{
    uint8_t tx[3];
    uint8_t rx[3];
    can_mcp2515_odrive_status_t st;

    tx[0] = MCP2515_CMD_READ;
    tx[1] = addr;
    tx[2] = 0U;
    memset(rx, 0, sizeof(rx));
    st = can_mcp2515_odrive_spi_blocking_xfer(dev, tx, rx, (uint16_t)sizeof(tx));
    if (st != CAN_MCP2515_ODRIVE_STATUS_OK)
    {
        return st;
    }

    *value = rx[2];
    return CAN_MCP2515_ODRIVE_STATUS_OK;
}

static can_mcp2515_odrive_status_t can_mcp2515_odrive_bit_modify(can_mcp2515_odrive_t *dev, uint8_t addr, uint8_t mask, uint8_t value)
{
    uint8_t tx[4];
    uint8_t rx[4];

    tx[0] = MCP2515_CMD_BIT_MODIFY;
    tx[1] = addr;
    tx[2] = mask;
    tx[3] = value;
    memset(rx, 0, sizeof(rx));
    return can_mcp2515_odrive_spi_blocking_xfer(dev, tx, rx, (uint16_t)sizeof(tx));
}

static can_mcp2515_odrive_status_t can_mcp2515_odrive_reset_mcp(can_mcp2515_odrive_t *dev)
{
    uint8_t tx[1];
    uint8_t rx[1];

    tx[0] = MCP2515_CMD_RESET;
    rx[0] = 0U;
    return can_mcp2515_odrive_spi_blocking_xfer(dev, tx, rx, 1U);
}

static can_mcp2515_odrive_status_t can_mcp2515_odrive_set_mode(can_mcp2515_odrive_t *dev, can_mcp2515_odrive_mode_t mode)
{
    uint8_t reqop;
    uint8_t canstat = 0U;
    uint32_t retry;
    can_mcp2515_odrive_status_t st;

    reqop = (mode == CAN_MCP2515_ODRIVE_MODE_LISTEN_ONLY) ? MCP2515_CANCTRL_REQOP_LIST : MCP2515_CANCTRL_REQOP_NORM;
    st = can_mcp2515_odrive_bit_modify(dev, MCP2515_REG_CANCTRL, MCP2515_CANCTRL_REQOP_MASK, reqop);
    if (st != CAN_MCP2515_ODRIVE_STATUS_OK)
    {
        return st;
    }

    for (retry = 0U; retry < 50U; ++retry)
    {
        st = can_mcp2515_odrive_read_reg(dev, MCP2515_REG_CANSTAT, &canstat);
        if (st != CAN_MCP2515_ODRIVE_STATUS_OK)
        {
            return st;
        }
        if ((canstat & MCP2515_CANSTAT_OPMOD_MASK) == reqop)
        {
            dev->diag.last_canstat = canstat;
            return CAN_MCP2515_ODRIVE_STATUS_OK;
        }
        HAL_Delay(1U);
    }

    return CAN_MCP2515_ODRIVE_STATUS_TIMEOUT;
}

static can_mcp2515_odrive_status_t can_mcp2515_odrive_set_bitrate(can_mcp2515_odrive_t *dev)
{
    uint8_t cnf1;
    uint8_t cnf2;
    uint8_t cnf3;

    if ((dev->mcp2515_osc_hz == 8000000U) && (dev->can_bitrate == 250000U))
    {
        cnf1 = 0x00U;
        cnf2 = 0xB1U;
        cnf3 = 0x05U;
    }
    else if ((dev->mcp2515_osc_hz == 8000000U) && (dev->can_bitrate == 500000U))
    {
        cnf1 = 0x00U;
        cnf2 = 0x90U;
        cnf3 = 0x02U;
    }
    else if ((dev->mcp2515_osc_hz == 16000000U) && (dev->can_bitrate == 250000U))
    {
        cnf1 = 0x41U;
        cnf2 = 0xF1U;
        cnf3 = 0x85U;
    }
    else if ((dev->mcp2515_osc_hz == 16000000U) && (dev->can_bitrate == 500000U))
    {
        cnf1 = 0x00U;
        cnf2 = 0xF0U;
        cnf3 = 0x86U;
    }
    else if ((dev->mcp2515_osc_hz == 16000000U) && (dev->can_bitrate == 1000000U))
    {
        cnf1 = 0x00U;
        cnf2 = 0xD0U;
        cnf3 = 0x82U;
    }
    else
    {
        return CAN_MCP2515_ODRIVE_STATUS_UNSUPPORTED;
    }

    if (can_mcp2515_odrive_write_reg(dev, MCP2515_REG_CNF1, cnf1) != CAN_MCP2515_ODRIVE_STATUS_OK)
    {
        return CAN_MCP2515_ODRIVE_STATUS_HAL_ERROR;
    }
    if (can_mcp2515_odrive_write_reg(dev, MCP2515_REG_CNF2, cnf2) != CAN_MCP2515_ODRIVE_STATUS_OK)
    {
        return CAN_MCP2515_ODRIVE_STATUS_HAL_ERROR;
    }
    if (can_mcp2515_odrive_write_reg(dev, MCP2515_REG_CNF3, cnf3) != CAN_MCP2515_ODRIVE_STATUS_OK)
    {
        return CAN_MCP2515_ODRIVE_STATUS_HAL_ERROR;
    }

    return CAN_MCP2515_ODRIVE_STATUS_OK;
}

static can_mcp2515_odrive_status_t can_mcp2515_odrive_start_dma(can_mcp2515_odrive_t *dev, uint8_t len)
{
    HAL_StatusTypeDef hal_status;

    dev->diag.spi_busy = 1U;
    dev->diag.spi_done = 0U;
    dev->diag.spi_error = 0U;
    can_mcp2515_odrive_timebase_update(dev);
    dev->diag.last_dma_start_us = dev->diag.time_us;

    can_mcp2515_odrive_cs_low(dev);
    hal_status = HAL_SPI_TransmitReceive_DMA(dev->hspi, dev->spi_tx_buf, dev->spi_rx_buf, len);
    if (hal_status != HAL_OK)
    {
        can_mcp2515_odrive_cs_high(dev);
        dev->diag.spi_busy = 0U;
        dev->diag.spi_error = 1U;
        dev->diag.last_status = CAN_MCP2515_ODRIVE_STATUS_HAL_ERROR;
        return CAN_MCP2515_ODRIVE_STATUS_HAL_ERROR;
    }

    dev->diag.spi_dma_count++;
    return CAN_MCP2515_ODRIVE_STATUS_OK;
}

static can_mcp2515_odrive_node_t *can_mcp2515_odrive_node_by_index(can_mcp2515_odrive_t *dev, uint8_t index)
{
    return (index == 0U) ? &dev->node1 : &dev->node2;
}

static can_mcp2515_odrive_node_t *can_mcp2515_odrive_node_by_node_id(can_mcp2515_odrive_t *dev, uint8_t node_id)
{
    if (node_id == dev->node_id_1)
    {
        return &dev->node1;
    }
    if (node_id == dev->node_id_2)
    {
        return &dev->node2;
    }
    return NULL;
}

static void can_mcp2515_odrive_mark_pair_encoder_ready(can_mcp2515_odrive_t *dev)
{
    uint64_t ts1;
    uint64_t ts2;

    if ((dev->diag.current_reply_ok[0] == 0U) || (dev->diag.current_reply_ok[1] == 0U))
    {
        dev->pair.ready = 0U;
        return;
    }

    ts1 = dev->node1.encoder.rsp_ts_us;
    ts2 = dev->node2.encoder.rsp_ts_us;

    dev->pair.pos1 = dev->node1.encoder.pos_estimate;
    dev->pair.vel1 = dev->node1.encoder.vel_estimate;
    dev->pair.pos2 = dev->node2.encoder.pos_estimate;
    dev->pair.vel2 = dev->node2.encoder.vel_estimate;
    dev->pair.ts1_us = ts1;
    dev->pair.ts2_us = ts2;
    dev->pair.pair_complete_ts_us = (ts1 >= ts2) ? ts1 : ts2;
    dev->pair.pair_dt_us = (ts1 >= ts2) ? (ts1 - ts2) : (ts2 - ts1);
    dev->pair.ready = 1U;
}

static void can_mcp2515_odrive_update_expected_reply(can_mcp2515_odrive_t *dev, uint8_t node_index)
{
    uint8_t node_id = (node_index == 0U) ? dev->node_id_1 : dev->node_id_2;
    uint8_t reply_cmd = dev->diag.current_cmd_id;

    if ((dev->current_op.type == CAN_MCP2515_ODRIVE_OP_PAIR_SDO_READ_U32) ||
        (dev->current_op.type == CAN_MCP2515_ODRIVE_OP_PAIR_SDO_WRITE_U32))
    {
        reply_cmd = CAN_MCP2515_ODRIVE_CMD_TX_SDO;
    }

    dev->diag.expected_reply_cmd = reply_cmd;
    dev->diag.expected_reply_id = can_mcp2515_odrive_make_can_id(node_id, reply_cmd);
    dev->diag.current_node_index = node_index;
}

static void can_mcp2515_odrive_prepare_can_frame(can_mcp2515_odrive_t *dev, uint16_t can_id, uint8_t dlc, uint8_t is_rtr, const uint8_t *payload)
{
    uint8_t i;

    memset(dev->spi_tx_buf, 0, sizeof(dev->spi_tx_buf));
    memset(dev->spi_rx_buf, 0, sizeof(dev->spi_rx_buf));

    dev->spi_tx_buf[0] = MCP2515_CMD_WRITE;
    dev->spi_tx_buf[1] = MCP2515_REG_TXB0SIDH;
    dev->spi_tx_buf[2] = (uint8_t)(can_id >> 3);
    dev->spi_tx_buf[3] = (uint8_t)((can_id & 0x07U) << 5);
    dev->spi_tx_buf[4] = 0U;
    dev->spi_tx_buf[5] = 0U;
    dev->spi_tx_buf[6] = (uint8_t)(dlc & 0x0FU);
    if (is_rtr != 0U)
    {
        dev->spi_tx_buf[6] |= 0x40U;
    }
    for (i = 0U; i < 8U; ++i)
    {
        dev->spi_tx_buf[7U + i] = (payload != NULL) ? payload[i] : 0U;
    }
}

static void can_mcp2515_odrive_prepare_read_reg(can_mcp2515_odrive_t *dev, uint8_t addr)
{
    memset(dev->spi_tx_buf, 0, sizeof(dev->spi_tx_buf));
    memset(dev->spi_rx_buf, 0, sizeof(dev->spi_rx_buf));
    dev->spi_tx_buf[0] = MCP2515_CMD_READ;
    dev->spi_tx_buf[1] = addr;
}

static void can_mcp2515_odrive_prepare_bit_modify(can_mcp2515_odrive_t *dev, uint8_t addr, uint8_t mask, uint8_t value)
{
    memset(dev->spi_tx_buf, 0, sizeof(dev->spi_tx_buf));
    memset(dev->spi_rx_buf, 0, sizeof(dev->spi_rx_buf));
    dev->spi_tx_buf[0] = MCP2515_CMD_BIT_MODIFY;
    dev->spi_tx_buf[1] = addr;
    dev->spi_tx_buf[2] = mask;
    dev->spi_tx_buf[3] = value;
}

static void can_mcp2515_odrive_prepare_read_rxb(can_mcp2515_odrive_t *dev, uint8_t rxb_index)
{
    memset(dev->spi_tx_buf, 0, sizeof(dev->spi_tx_buf));
    memset(dev->spi_rx_buf, 0, sizeof(dev->spi_rx_buf));
    dev->spi_tx_buf[0] = (rxb_index == 0U) ? MCP2515_CMD_READ_RXB0 : MCP2515_CMD_READ_RXB1;
}

static void can_mcp2515_odrive_prepare_rts_txb0(can_mcp2515_odrive_t *dev)
{
    memset(dev->spi_tx_buf, 0, sizeof(dev->spi_tx_buf));
    memset(dev->spi_rx_buf, 0, sizeof(dev->spi_rx_buf));
    dev->spi_tx_buf[0] = MCP2515_CMD_RTS_TXB0;
}

static uint8_t can_mcp2515_odrive_decode_dlc(uint8_t reg_dlc)
{
    uint8_t dlc = (uint8_t)(reg_dlc & 0x0FU);
    return (dlc > 8U) ? 8U : dlc;
}

static uint16_t can_mcp2515_odrive_decode_std_id(uint8_t sidh, uint8_t sidl)
{
    return (uint16_t)(((uint16_t)sidh << 3) | ((uint16_t)sidl >> 5));
}

static void can_mcp2515_odrive_store_last_frame(can_mcp2515_odrive_t *dev, uint16_t can_id, uint8_t dlc, const uint8_t *data)
{
    uint8_t i;
    dev->diag.last_id = can_id;
    dev->diag.last_dlc = dlc;
    for (i = 0U; i < 8U; ++i)
    {
        dev->diag.last_data[i] = (i < dlc) ? data[i] : 0U;
    }
}

static void can_mcp2515_odrive_parse_frame(can_mcp2515_odrive_t *dev, uint16_t can_id, uint8_t dlc, uint8_t is_rtr, const uint8_t *data)
{
    uint8_t cmd_id;
    uint8_t node_id;
    can_mcp2515_odrive_node_t *node;

    can_mcp2515_odrive_timebase_update(dev);
    can_mcp2515_odrive_store_last_frame(dev, can_id, dlc, data);

    cmd_id = (uint8_t)(can_id & 0x1FU);
    node_id = (uint8_t)(can_id >> 5);
    node = can_mcp2515_odrive_node_by_node_id(dev, node_id);
    if (node == NULL)
    {
        return;
    }

    node->online = 1U;
    node->fresh = 1U;
    node->stale = 0U;
    node->rx_count++;
    node->last_rx.valid = 1U;
    node->last_rx.can_id = can_id;
    node->last_rx.cmd_id = cmd_id;
    node->last_rx.dlc = dlc;
    node->last_rx.is_rtr = is_rtr;
    node->last_rx.ts_cycles_64 = dev->diag.time_cycles_64;
    node->last_rx.ts_us = dev->diag.time_us;
    memset(node->last_rx.data, 0, sizeof(node->last_rx.data));
    memcpy(node->last_rx.data, data, dlc);

    if ((dev->diag.current_expect_reply != 0U) && (can_id == dev->diag.expected_reply_id) && (cmd_id == dev->diag.expected_reply_cmd))
    {
        dev->diag.current_reply_ok[dev->diag.current_node_index] = 1U;
    }

    if ((cmd_id == CAN_MCP2515_ODRIVE_CMD_HEARTBEAT) && (dlc >= 7U))
    {
        node->heartbeat.valid = 1U;
        node->heartbeat.axis_error = can_mcp2515_odrive_u32_from_le(&data[0]);
        node->heartbeat.axis_state = data[4];
        node->heartbeat.procedure_result = data[5];
        node->heartbeat.trajectory_done = data[6];
        node->heartbeat.rx_count++;
        node->heartbeat.ts_cycles_64 = dev->diag.time_cycles_64;
        node->heartbeat.ts_us = dev->diag.time_us;
        return;
    }

    if ((cmd_id == CAN_MCP2515_ODRIVE_CMD_GET_ENCODER_ESTIMATES) && (dlc == 8U))
    {
        node->encoder.valid = 1U;
        node->encoder.pos_estimate = can_mcp2515_odrive_f32_from_le(&data[0]);
        node->encoder.vel_estimate = can_mcp2515_odrive_f32_from_le(&data[4]);
        node->encoder.rx_count++;
        node->encoder.rsp_ts_cycles_64 = dev->diag.time_cycles_64;
        node->encoder.rsp_ts_us = dev->diag.time_us;
        if (node->encoder.req_ts_us <= node->encoder.rsp_ts_us)
        {
            node->encoder.latency_us = node->encoder.rsp_ts_us - node->encoder.req_ts_us;
        }
        else
        {
            node->encoder.latency_us = 0ULL;
        }
        return;
    }

    if ((cmd_id == CAN_MCP2515_ODRIVE_CMD_GET_ERROR) && (dlc == 8U))
    {
        node->error.valid = 1U;
        node->error.active_errors = can_mcp2515_odrive_u32_from_le(&data[0]);
        node->error.disarm_reason = can_mcp2515_odrive_u32_from_le(&data[4]);
        node->error.ts_us = dev->diag.time_us;
        return;
    }

    if ((cmd_id == CAN_MCP2515_ODRIVE_CMD_GET_BUS_VOLTAGE_CURRENT) && (dlc == 8U))
    {
        node->bus.valid = 1U;
        node->bus.bus_voltage = can_mcp2515_odrive_f32_from_le(&data[0]);
        node->bus.bus_current = can_mcp2515_odrive_f32_from_le(&data[4]);
        node->bus.ts_us = dev->diag.time_us;
        return;
    }

    if ((cmd_id == CAN_MCP2515_ODRIVE_CMD_GET_IQ) && (dlc == 8U))
    {
        node->iq.valid = 1U;
        node->iq.iq_setpoint = can_mcp2515_odrive_f32_from_le(&data[0]);
        node->iq.iq_measured = can_mcp2515_odrive_f32_from_le(&data[4]);
        node->iq.ts_us = dev->diag.time_us;
        return;
    }

    if ((cmd_id == CAN_MCP2515_ODRIVE_CMD_TX_SDO) && (dlc == 8U))
    {
        node->sdo.valid = 1U;
        node->sdo.endpoint_id = (uint16_t)(((uint16_t)data[1]) | ((uint16_t)data[2] << 8));
        node->sdo.value_u32 = can_mcp2515_odrive_u32_from_le(&data[4]);
        node->sdo.value_f32 = can_mcp2515_odrive_f32_from_le(&data[4]);
        node->sdo.ts_us = dev->diag.time_us;
        return;
    }
}

static can_mcp2515_odrive_status_t can_mcp2515_odrive_queue_push(can_mcp2515_odrive_t *dev, const can_mcp2515_odrive_op_t *op)
{
    if ((dev == NULL) || (op == NULL))
    {
        return CAN_MCP2515_ODRIVE_STATUS_BAD_PARAM;
    }
    if (dev->diag.queue_count >= CAN_MCP2515_ODRIVE_QUEUE_CAPACITY)
    {
        return CAN_MCP2515_ODRIVE_STATUS_QUEUE_FULL;
    }

    dev->queue[dev->diag.queue_tail] = *op;
    dev->diag.queue_tail = (uint8_t)((dev->diag.queue_tail + 1U) % CAN_MCP2515_ODRIVE_QUEUE_CAPACITY);
    dev->diag.queue_count++;
    return CAN_MCP2515_ODRIVE_STATUS_OK;
}

static uint8_t can_mcp2515_odrive_queue_pop(can_mcp2515_odrive_t *dev, can_mcp2515_odrive_op_t *op)
{
    if ((dev->diag.queue_count == 0U) || (op == NULL))
    {
        return 0U;
    }

    *op = dev->queue[dev->diag.queue_head];
    dev->diag.queue_head = (uint8_t)((dev->diag.queue_head + 1U) % CAN_MCP2515_ODRIVE_QUEUE_CAPACITY);
    dev->diag.queue_count--;
    return 1U;
}

static void can_mcp2515_odrive_start_current_op(can_mcp2515_odrive_t *dev, const can_mcp2515_odrive_op_t *op, uint8_t is_periodic)
{
    uint8_t i;

    dev->current_op = *op;
    dev->diag.current_started = 1U;
    dev->diag.current_is_periodic = is_periodic;
    dev->diag.current_phase = CAN_MCP2515_ODRIVE_PHASE_NODE1;
    dev->diag.current_expect_reply = 0U;
    dev->diag.current_cmd_id = op->cmd_id;
    dev->diag.current_reply_ok[0] = 0U;
    dev->diag.current_reply_ok[1] = 0U;
    dev->diag.current_tx_ok[0] = 0U;
    dev->diag.current_tx_ok[1] = 0U;
    dev->diag.tx_poll_count = 0U;
    dev->pair.op_active = 1U;
    dev->pair.last_op_type = op->type;
    dev->pair.op_seq++;
    dev->diag.last_poll_start_us = dev->diag.time_us;

    for (i = 0U; i < 2U; ++i)
    {
        can_mcp2515_odrive_node_t *node = can_mcp2515_odrive_node_by_index(dev, i);
        node->fresh = 0U;
    }
}

static void can_mcp2515_odrive_prepare_node_payload(can_mcp2515_odrive_t *dev, uint8_t node_index, uint16_t *can_id, uint8_t *dlc, uint8_t *is_rtr, uint8_t payload[8])
{
    uint8_t i;
    uint8_t node_id = (node_index == 0U) ? dev->node_id_1 : dev->node_id_2;
    memset(payload, 0, 8U);
    *dlc = 0U;
    *is_rtr = 0U;
    *can_id = can_mcp2515_odrive_make_can_id(node_id, dev->current_op.cmd_id);

    switch (dev->current_op.type)
    {
        case CAN_MCP2515_ODRIVE_OP_PAIR_SET_AXIS_STATE:
            *dlc = 4U;
            can_mcp2515_odrive_u32_to_le(dev->current_op.u32_a[node_index], payload);
            break;

        case CAN_MCP2515_ODRIVE_OP_PAIR_SET_CONTROLLER_MODE:
            *dlc = 8U;
            can_mcp2515_odrive_u32_to_le(dev->current_op.u32_a[node_index], &payload[0]);
            can_mcp2515_odrive_u32_to_le(dev->current_op.u32_b[node_index], &payload[4]);
            break;

        case CAN_MCP2515_ODRIVE_OP_PAIR_SET_INPUT_POS:
            *dlc = 8U;
            can_mcp2515_odrive_f32_to_le(dev->current_op.f32_a[node_index], &payload[0]);
            can_mcp2515_odrive_i16_to_le(dev->current_op.i16_a[node_index], &payload[4]);
            can_mcp2515_odrive_i16_to_le(dev->current_op.i16_b[node_index], &payload[6]);
            break;

        case CAN_MCP2515_ODRIVE_OP_PAIR_SET_INPUT_VEL:
            *dlc = 8U;
            can_mcp2515_odrive_f32_to_le(dev->current_op.f32_a[node_index], &payload[0]);
            can_mcp2515_odrive_f32_to_le(dev->current_op.f32_b[node_index], &payload[4]);
            break;

        case CAN_MCP2515_ODRIVE_OP_PAIR_SET_INPUT_TORQUE:
            *dlc = 4U;
            can_mcp2515_odrive_f32_to_le(dev->current_op.f32_a[node_index], &payload[0]);
            break;

        case CAN_MCP2515_ODRIVE_OP_PAIR_SET_LIMITS:
            *dlc = 8U;
            can_mcp2515_odrive_f32_to_le(dev->current_op.f32_a[node_index], &payload[0]);
            can_mcp2515_odrive_f32_to_le(dev->current_op.f32_b[node_index], &payload[4]);
            break;

        case CAN_MCP2515_ODRIVE_OP_PAIR_SET_VEL_GAINS:
            *dlc = 8U;
            can_mcp2515_odrive_f32_to_le(dev->current_op.f32_a[node_index], &payload[0]);
            can_mcp2515_odrive_f32_to_le(dev->current_op.f32_b[node_index], &payload[4]);
            break;

        case CAN_MCP2515_ODRIVE_OP_PAIR_CLEAR_ERRORS:
            *dlc = 1U;
            payload[0] = dev->current_op.identify;
            break;

        case CAN_MCP2515_ODRIVE_OP_PAIR_ESTOP:
            *dlc = 0U;
            break;

        case CAN_MCP2515_ODRIVE_OP_PAIR_REQ_ENCODER:
        case CAN_MCP2515_ODRIVE_OP_PAIR_REQ_RAW_RTR:
            *dlc = 0U;
            *is_rtr = 1U;
            break;

        case CAN_MCP2515_ODRIVE_OP_PAIR_SEND_RAW:
            *dlc = dev->current_op.raw_dlc[node_index];
            for (i = 0U; i < *dlc; ++i)
            {
                payload[i] = dev->current_op.raw_data[node_index][i];
            }
            *is_rtr = dev->current_op.raw_is_rtr;
            break;

        case CAN_MCP2515_ODRIVE_OP_PAIR_SDO_READ_U32:
            *dlc = 8U;
            payload[0] = CAN_MCP2515_ODRIVE_SDO_OPCODE_READ;
            payload[1] = (uint8_t)(dev->current_op.u32_a[node_index] & 0xFFU);
            payload[2] = (uint8_t)((dev->current_op.u32_a[node_index] >> 8) & 0xFFU);
            payload[3] = 0U;
            memset(&payload[4], 0, 4U);
            break;

        case CAN_MCP2515_ODRIVE_OP_PAIR_SDO_WRITE_U32:
            *dlc = 8U;
            payload[0] = CAN_MCP2515_ODRIVE_SDO_OPCODE_WRITE;
            payload[1] = (uint8_t)(dev->current_op.u32_a[node_index] & 0xFFU);
            payload[2] = (uint8_t)((dev->current_op.u32_a[node_index] >> 8) & 0xFFU);
            payload[3] = 0U;
            can_mcp2515_odrive_u32_to_le(dev->current_op.u32_b[node_index], &payload[4]);
            break;

        default:
            break;
    }
}

static uint8_t can_mcp2515_odrive_current_op_requires_reply(const can_mcp2515_odrive_t *dev)
{
    switch (dev->current_op.type)
    {
        case CAN_MCP2515_ODRIVE_OP_PAIR_REQ_ENCODER:
        case CAN_MCP2515_ODRIVE_OP_PAIR_REQ_RAW_RTR:
        case CAN_MCP2515_ODRIVE_OP_PAIR_SDO_READ_U32:
        case CAN_MCP2515_ODRIVE_OP_PAIR_SDO_WRITE_U32:
            return 1U;
        default:
            return 0U;
    }
}

static void can_mcp2515_odrive_finish_current_op(can_mcp2515_odrive_t *dev)
{
    dev->pair.op_active = 0U;
    if (dev->current_op.type == CAN_MCP2515_ODRIVE_OP_PAIR_REQ_ENCODER)
    {
        can_mcp2515_odrive_mark_pair_encoder_ready(dev);
    }
    if (dev->current_op.type == CAN_MCP2515_ODRIVE_OP_PAIR_SET_INPUT_TORQUE)
    {
        dev->pair.last_torque1 = dev->current_op.f32_a[0];
        dev->pair.last_torque2 = dev->current_op.f32_a[1];
        dev->pair.last_pair_cmd_ts_us = dev->diag.time_us;
    }
    if (dev->current_op.type == CAN_MCP2515_ODRIVE_OP_PAIR_SET_INPUT_VEL)
    {
        dev->pair.last_vel1 = dev->current_op.f32_a[0];
        dev->pair.last_vel2 = dev->current_op.f32_a[1];
        dev->pair.last_pair_cmd_ts_us = dev->diag.time_us;
    }
    if (dev->current_op.type == CAN_MCP2515_ODRIVE_OP_PAIR_SET_INPUT_POS)
    {
        dev->pair.last_pos1 = dev->current_op.f32_a[0];
        dev->pair.last_pos2 = dev->current_op.f32_a[1];
        dev->pair.last_pair_cmd_ts_us = dev->diag.time_us;
    }
    dev->diag.current_started = 0U;
    dev->diag.current_expect_reply = 0U;
    memset(&dev->current_op, 0, sizeof(dev->current_op));
    dev->diag.sm = CAN_MCP2515_ODRIVE_SM_IDLE;
}

static can_mcp2515_odrive_status_t can_mcp2515_odrive_service_interrupt_start(can_mcp2515_odrive_t *dev, can_mcp2515_odrive_sm_t resume_sm)
{
    dev->diag.resume_sm = resume_sm;
    can_mcp2515_odrive_prepare_read_reg(dev, MCP2515_REG_CANINTF);
    dev->diag.sm = CAN_MCP2515_ODRIVE_SM_IRQ_READ_CANINTF_WAIT;
    return can_mcp2515_odrive_start_dma(dev, 3U);
}

static can_mcp2515_odrive_status_t can_mcp2515_odrive_check_runtime(can_mcp2515_odrive_t *dev)
{
    if (dev == NULL)
    {
        return CAN_MCP2515_ODRIVE_STATUS_BAD_PARAM;
    }

    can_mcp2515_odrive_timebase_update(dev);
    if ((dev->diag.is_initialized == 0U) || (dev->hspi == NULL))
    {
        return CAN_MCP2515_ODRIVE_STATUS_NOT_READY;
    }
    if (dev->diag.spi_error != 0U)
    {
        dev->diag.last_status = CAN_MCP2515_ODRIVE_STATUS_HAL_ERROR;
        dev->diag.sm = CAN_MCP2515_ODRIVE_SM_ERROR_RECOVERY;
    }
    if ((dev->diag.spi_busy != 0U) && ((dev->diag.time_us - dev->diag.last_dma_start_us) > (uint64_t)dev->spi_timeout_us))
    {
        dev->diag.spi_busy = 0U;
        dev->diag.spi_error = 1U;
        dev->diag.last_status = CAN_MCP2515_ODRIVE_STATUS_TIMEOUT;
        dev->diag.sm = CAN_MCP2515_ODRIVE_SM_ERROR_RECOVERY;
    }
    return CAN_MCP2515_ODRIVE_STATUS_OK;
}

static void can_mcp2515_odrive_update_node_cmd_cache(can_mcp2515_odrive_t *dev, uint8_t node_index)
{
    can_mcp2515_odrive_node_t *node = can_mcp2515_odrive_node_by_index(dev, node_index);

    node->tx_count++;
    node->last_cmd_ts_us = dev->diag.time_us;

    switch (dev->current_op.type)
    {
        case CAN_MCP2515_ODRIVE_OP_PAIR_SET_AXIS_STATE:
            node->last_axis_state_cmd = dev->current_op.u32_a[node_index];
            break;
        case CAN_MCP2515_ODRIVE_OP_PAIR_SET_CONTROLLER_MODE:
            node->last_control_mode_cmd = dev->current_op.u32_a[node_index];
            node->last_input_mode_cmd = dev->current_op.u32_b[node_index];
            break;
        case CAN_MCP2515_ODRIVE_OP_PAIR_SET_INPUT_POS:
            node->last_pos_cmd = dev->current_op.f32_a[node_index];
            break;
        case CAN_MCP2515_ODRIVE_OP_PAIR_SET_INPUT_VEL:
            node->last_vel_cmd = dev->current_op.f32_a[node_index];
            break;
        case CAN_MCP2515_ODRIVE_OP_PAIR_SET_INPUT_TORQUE:
            node->last_torque_cmd = dev->current_op.f32_a[node_index];
            break;
        case CAN_MCP2515_ODRIVE_OP_PAIR_REQ_ENCODER:
            node->encoder.req_count++;
            node->encoder.req_ts_cycles_64 = dev->diag.time_cycles_64;
            node->encoder.req_ts_us = dev->diag.time_us;
            dev->diag.last_poll_start_us = dev->diag.time_us;
            break;
        default:
            break;
    }

    if (can_mcp2515_odrive_current_op_requires_reply(dev) != 0U)
    {
        dev->diag.last_poll_start_us = dev->diag.time_us;
    }
}

can_mcp2515_odrive_status_t can_mcp2515_odrive_init(can_mcp2515_odrive_t *dev)
{
    uint8_t mode_req;
    can_mcp2515_odrive_status_t st;

    if ((dev == NULL) || (dev->hspi == NULL) || (dev->cs_port == NULL) || (dev->int_port == NULL))
    {
        return CAN_MCP2515_ODRIVE_STATUS_BAD_PARAM;
    }
    if ((dev->node_id_1 == 0U) || (dev->node_id_2 == 0U) || (dev->node_id_1 == dev->node_id_2))
    {
        return CAN_MCP2515_ODRIVE_STATUS_BAD_PARAM;
    }
    if ((dev->poll_pair_rate_hz != 0U) && ((dev->poll_pair_rate_hz < 250U) || (dev->poll_pair_rate_hz > 1200U)))
    {
        return CAN_MCP2515_ODRIVE_STATUS_BAD_PARAM;
    }

    memset(&dev->node1, 0, sizeof(dev->node1));
    memset(&dev->node2, 0, sizeof(dev->node2));
    memset(&dev->pair, 0, sizeof(dev->pair));
    memset(&dev->diag, 0, sizeof(dev->diag));
    memset(dev->queue, 0, sizeof(dev->queue));
    memset(&dev->current_op, 0, sizeof(dev->current_op));
    memset(dev->spi_tx_buf, 0, sizeof(dev->spi_tx_buf));
    memset(dev->spi_rx_buf, 0, sizeof(dev->spi_rx_buf));

    dev->node1.node_id = dev->node_id_1;
    dev->node2.node_id = dev->node_id_2;
    dev->can_id_1_heartbeat = can_mcp2515_odrive_make_can_id(dev->node_id_1, CAN_MCP2515_ODRIVE_CMD_HEARTBEAT);
    dev->can_id_2_heartbeat = can_mcp2515_odrive_make_can_id(dev->node_id_2, CAN_MCP2515_ODRIVE_CMD_HEARTBEAT);
    dev->can_id_1_encoder = can_mcp2515_odrive_make_can_id(dev->node_id_1, CAN_MCP2515_ODRIVE_CMD_GET_ENCODER_ESTIMATES);
    dev->can_id_2_encoder = can_mcp2515_odrive_make_can_id(dev->node_id_2, CAN_MCP2515_ODRIVE_CMD_GET_ENCODER_ESTIMATES);
    dev->diag.queue_capacity = CAN_MCP2515_ODRIVE_QUEUE_CAPACITY;
    dev->diag.tx_poll_limit = 32U;
    dev->diag.sm = CAN_MCP2515_ODRIVE_SM_STOPPED;
    dev->diag.last_status = CAN_MCP2515_ODRIVE_STATUS_OK;

    if (dev->reply_timeout_us == 0U)
    {
        dev->reply_timeout_us = 3000U;
    }
    if (dev->spi_timeout_us == 0U)
    {
        dev->spi_timeout_us = 2000U;
    }

    can_mcp2515_odrive_timebase_init(dev);
    can_mcp2515_odrive_cs_high(dev);
    HAL_Delay(2U);

    st = can_mcp2515_odrive_reset_mcp(dev);
    if (st != CAN_MCP2515_ODRIVE_STATUS_OK)
    {
        return st;
    }
    HAL_Delay(2U);

    st = can_mcp2515_odrive_bit_modify(dev, MCP2515_REG_CANCTRL, MCP2515_CANCTRL_REQOP_MASK, MCP2515_CANCTRL_REQOP_CONF);
    if (st != CAN_MCP2515_ODRIVE_STATUS_OK)
    {
        return st;
    }

    st = can_mcp2515_odrive_set_bitrate(dev);
    if (st != CAN_MCP2515_ODRIVE_STATUS_OK)
    {
        return st;
    }

    st = can_mcp2515_odrive_write_reg(dev, MCP2515_REG_RXB0CTRL, 0x64U);
    if (st != CAN_MCP2515_ODRIVE_STATUS_OK)
    {
        return st;
    }
    st = can_mcp2515_odrive_write_reg(dev, MCP2515_REG_RXB1CTRL, 0x60U);
    if (st != CAN_MCP2515_ODRIVE_STATUS_OK)
    {
        return st;
    }

    st = can_mcp2515_odrive_write_reg(dev, MCP2515_REG_CANINTF, 0x00U);
    if (st != CAN_MCP2515_ODRIVE_STATUS_OK)
    {
        return st;
    }
    st = can_mcp2515_odrive_write_reg(dev, MCP2515_REG_EFLG, 0x00U);
    if (st != CAN_MCP2515_ODRIVE_STATUS_OK)
    {
        return st;
    }

    st = can_mcp2515_odrive_write_reg(dev, MCP2515_REG_CANINTE, MCP2515_CANINTE_RX0IE | MCP2515_CANINTE_RX1IE | MCP2515_CANINTE_ERRIE);
    if (st != CAN_MCP2515_ODRIVE_STATUS_OK)
    {
        return st;
    }

    mode_req = (dev->mode == CAN_MCP2515_ODRIVE_MODE_LISTEN_ONLY) ? MCP2515_CANCTRL_REQOP_LIST : MCP2515_CANCTRL_REQOP_NORM;
    (void)mode_req;
    st = can_mcp2515_odrive_set_mode(dev, dev->mode);
    if (st != CAN_MCP2515_ODRIVE_STATUS_OK)
    {
        return st;
    }

    (void)can_mcp2515_odrive_read_reg(dev, MCP2515_REG_CANINTE, &dev->diag.last_caninte);
    (void)can_mcp2515_odrive_read_reg(dev, MCP2515_REG_CANSTAT, &dev->diag.last_canstat);
    (void)can_mcp2515_odrive_read_reg(dev, MCP2515_REG_CANINTF, &dev->diag.last_canintf);
    (void)can_mcp2515_odrive_read_reg(dev, MCP2515_REG_EFLG, &dev->diag.last_eflg);

    dev->diag.is_initialized = 1U;
    dev->diag.poll_enabled = 0U;
    dev->diag.sm = CAN_MCP2515_ODRIVE_SM_IDLE;
    dev->diag.next_periodic_poll_us = dev->diag.time_us;
    return CAN_MCP2515_ODRIVE_STATUS_OK;
}

void can_mcp2515_odrive_exti_callback(can_mcp2515_odrive_t *dev, uint16_t gpio_pin)
{
    if ((dev == NULL) || (dev->diag.is_initialized == 0U))
    {
        return;
    }
    if (gpio_pin != dev->int_pin)
    {
        return;
    }

    dev->diag.int_pending = 1U;
    dev->diag.exti_count++;
}

void can_mcp2515_odrive_spi_txrx_cplt_callback(can_mcp2515_odrive_t *dev, SPI_HandleTypeDef *hspi)
{
    if ((dev == NULL) || (hspi != dev->hspi))
    {
        return;
    }

    can_mcp2515_odrive_timebase_update(dev);
    can_mcp2515_odrive_cs_high(dev);
    dev->diag.spi_busy = 0U;
    dev->diag.spi_done = 1U;
}

void can_mcp2515_odrive_spi_error_callback(can_mcp2515_odrive_t *dev, SPI_HandleTypeDef *hspi)
{
    if ((dev == NULL) || (hspi != dev->hspi))
    {
        return;
    }

    can_mcp2515_odrive_cs_high(dev);
    dev->diag.spi_busy = 0U;
    dev->diag.spi_done = 0U;
    dev->diag.spi_error = 1U;
    dev->diag.spi_error_count++;
}

can_mcp2515_odrive_status_t can_mcp2515_odrive_enable_polling(can_mcp2515_odrive_t *dev, uint8_t enable)
{
    if (dev == NULL)
    {
        return CAN_MCP2515_ODRIVE_STATUS_BAD_PARAM;
    }
    dev->diag.poll_enabled = (enable != 0U) ? 1U : 0U;
    can_mcp2515_odrive_timebase_update(dev);
    dev->diag.next_periodic_poll_us = dev->diag.time_us;
    return CAN_MCP2515_ODRIVE_STATUS_OK;
}

can_mcp2515_odrive_status_t can_mcp2515_odrive_set_poll_pair_rate_hz(can_mcp2515_odrive_t *dev, uint16_t rate_hz)
{
    if (dev == NULL)
    {
        return CAN_MCP2515_ODRIVE_STATUS_BAD_PARAM;
    }
    if ((rate_hz < 250U) || (rate_hz > 1200U))
    {
        return CAN_MCP2515_ODRIVE_STATUS_BAD_PARAM;
    }
    dev->poll_pair_rate_hz = rate_hz;
    return CAN_MCP2515_ODRIVE_STATUS_OK;
}

static can_mcp2515_odrive_status_t can_mcp2515_odrive_start_next_op_if_needed(can_mcp2515_odrive_t *dev)
{
    can_mcp2515_odrive_op_t op;

    if (dev->diag.current_started != 0U)
    {
        return CAN_MCP2515_ODRIVE_STATUS_OK;
    }

    if ((dev->diag.poll_enabled != 0U) && (dev->poll_pair_rate_hz >= 250U) && (dev->diag.queue_count < CAN_MCP2515_ODRIVE_QUEUE_CAPACITY))
    {
        if (dev->diag.time_us >= dev->diag.next_periodic_poll_us)
        {
            memset(&op, 0, sizeof(op));
            op.type = CAN_MCP2515_ODRIVE_OP_PAIR_REQ_ENCODER;
            op.cmd_id = CAN_MCP2515_ODRIVE_CMD_GET_ENCODER_ESTIMATES;
            dev->diag.next_periodic_poll_us = dev->diag.time_us + (1000000ULL / (uint64_t)dev->poll_pair_rate_hz);
            (void)can_mcp2515_odrive_queue_push(dev, &op);
        }
    }

    if (can_mcp2515_odrive_queue_pop(dev, &op) != 0U)
    {
        can_mcp2515_odrive_start_current_op(dev, &op, (op.type == CAN_MCP2515_ODRIVE_OP_PAIR_REQ_ENCODER) ? 1U : 0U);
        dev->diag.sm = CAN_MCP2515_ODRIVE_SM_OP_LOAD_NODE1_WAIT;
    }

    return CAN_MCP2515_ODRIVE_STATUS_OK;
}

static can_mcp2515_odrive_status_t can_mcp2515_odrive_process_irq_state(can_mcp2515_odrive_t *dev)
{
    uint16_t can_id;
    uint8_t dlc;
    uint8_t data[8];
    uint8_t i;

    switch (dev->diag.sm)
    {
        case CAN_MCP2515_ODRIVE_SM_IRQ_READ_CANINTF_WAIT:
            if (dev->diag.spi_done == 0U)
            {
                return CAN_MCP2515_ODRIVE_STATUS_OK;
            }
            dev->diag.spi_done = 0U;
            dev->diag.last_canintf = dev->spi_rx_buf[2];
            dev->diag.service_rx0 = ((dev->diag.last_canintf & MCP2515_CANINTF_RX0IF) != 0U) ? 1U : 0U;
            dev->diag.service_rx1 = ((dev->diag.last_canintf & MCP2515_CANINTF_RX1IF) != 0U) ? 1U : 0U;
            if ((dev->diag.last_canintf & (MCP2515_CANINTF_ERRIF | MCP2515_CANINTF_MERRF)) != 0U)
            {
                dev->diag.service_clear_mask |= (MCP2515_CANINTF_ERRIF | MCP2515_CANINTF_MERRF);
            }
            can_mcp2515_odrive_prepare_read_reg(dev, MCP2515_REG_EFLG);
            dev->diag.sm = CAN_MCP2515_ODRIVE_SM_IRQ_READ_EFLG_WAIT;
            return can_mcp2515_odrive_start_dma(dev, 3U);

        case CAN_MCP2515_ODRIVE_SM_IRQ_READ_EFLG_WAIT:
            if (dev->diag.spi_done == 0U)
            {
                return CAN_MCP2515_ODRIVE_STATUS_OK;
            }
            dev->diag.spi_done = 0U;
            dev->diag.last_eflg = dev->spi_rx_buf[2];
            if ((dev->diag.last_eflg & (MCP2515_EFLG_RX0OVR | MCP2515_EFLG_RX1OVR)) != 0U)
            {
                dev->diag.overflow_count++;
            }
            if ((dev->diag.last_eflg & MCP2515_EFLG_TXBO) != 0U)
            {
                dev->diag.busoff_count++;
            }
            can_mcp2515_odrive_prepare_read_reg(dev, MCP2515_REG_CANSTAT);
            dev->diag.sm = CAN_MCP2515_ODRIVE_SM_IRQ_READ_CANSTAT_WAIT;
            return can_mcp2515_odrive_start_dma(dev, 3U);

        case CAN_MCP2515_ODRIVE_SM_IRQ_READ_CANSTAT_WAIT:
            if (dev->diag.spi_done == 0U)
            {
                return CAN_MCP2515_ODRIVE_STATUS_OK;
            }
            dev->diag.spi_done = 0U;
            dev->diag.last_canstat = dev->spi_rx_buf[2];
            if (dev->diag.service_rx0 != 0U)
            {
                can_mcp2515_odrive_prepare_read_rxb(dev, 0U);
                dev->diag.sm = CAN_MCP2515_ODRIVE_SM_IRQ_READ_RXB0_WAIT;
                return can_mcp2515_odrive_start_dma(dev, 14U);
            }
            if (dev->diag.service_rx1 != 0U)
            {
                can_mcp2515_odrive_prepare_read_rxb(dev, 1U);
                dev->diag.sm = CAN_MCP2515_ODRIVE_SM_IRQ_READ_RXB1_WAIT;
                return can_mcp2515_odrive_start_dma(dev, 14U);
            }
            if (dev->diag.service_clear_mask != 0U)
            {
                can_mcp2515_odrive_prepare_bit_modify(dev, MCP2515_REG_CANINTF, dev->diag.service_clear_mask, 0U);
                dev->diag.sm = CAN_MCP2515_ODRIVE_SM_IRQ_CLEAR_FLAGS_WAIT;
                return can_mcp2515_odrive_start_dma(dev, 4U);
            }
            dev->diag.int_pending = 0U;
            dev->diag.sm = dev->diag.resume_sm;
            return CAN_MCP2515_ODRIVE_STATUS_OK;

        case CAN_MCP2515_ODRIVE_SM_IRQ_READ_RXB0_WAIT:
        case CAN_MCP2515_ODRIVE_SM_IRQ_READ_RXB1_WAIT:
            if (dev->diag.spi_done == 0U)
            {
                return CAN_MCP2515_ODRIVE_STATUS_OK;
            }
            dev->diag.spi_done = 0U;
            can_id = can_mcp2515_odrive_decode_std_id(dev->spi_rx_buf[1], dev->spi_rx_buf[2]);
            dlc = can_mcp2515_odrive_decode_dlc(dev->spi_rx_buf[5]);
            memset(data, 0, sizeof(data));
            for (i = 0U; i < dlc; ++i)
            {
                data[i] = dev->spi_rx_buf[6U + i];
            }
            can_mcp2515_odrive_parse_frame(dev, can_id, dlc, 0U, data);
            if (dev->diag.sm == CAN_MCP2515_ODRIVE_SM_IRQ_READ_RXB0_WAIT)
            {
                dev->diag.service_rx0 = 0U;
                dev->diag.service_clear_mask |= MCP2515_CANINTF_RX0IF;
                if (dev->diag.service_rx1 != 0U)
                {
                    can_mcp2515_odrive_prepare_read_rxb(dev, 1U);
                    dev->diag.sm = CAN_MCP2515_ODRIVE_SM_IRQ_READ_RXB1_WAIT;
                    return can_mcp2515_odrive_start_dma(dev, 14U);
                }
            }
            else
            {
                dev->diag.service_rx1 = 0U;
                dev->diag.service_clear_mask |= MCP2515_CANINTF_RX1IF;
            }
            if (dev->diag.service_clear_mask != 0U)
            {
                can_mcp2515_odrive_prepare_bit_modify(dev, MCP2515_REG_CANINTF, dev->diag.service_clear_mask, 0U);
                dev->diag.sm = CAN_MCP2515_ODRIVE_SM_IRQ_CLEAR_FLAGS_WAIT;
                return can_mcp2515_odrive_start_dma(dev, 4U);
            }
            dev->diag.int_pending = 0U;
            dev->diag.sm = dev->diag.resume_sm;
            return CAN_MCP2515_ODRIVE_STATUS_OK;

        case CAN_MCP2515_ODRIVE_SM_IRQ_CLEAR_FLAGS_WAIT:
            if (dev->diag.spi_done == 0U)
            {
                return CAN_MCP2515_ODRIVE_STATUS_OK;
            }
            dev->diag.spi_done = 0U;
            dev->diag.service_clear_mask = 0U;
            dev->diag.int_pending = 0U;
            dev->diag.sm = dev->diag.resume_sm;
            return CAN_MCP2515_ODRIVE_STATUS_OK;

        default:
            return CAN_MCP2515_ODRIVE_STATUS_OK;
    }
}

static can_mcp2515_odrive_status_t can_mcp2515_odrive_process_op_state(can_mcp2515_odrive_t *dev)
{
    uint16_t can_id;
    uint8_t dlc;
    uint8_t is_rtr;
    uint8_t payload[8];

    switch (dev->diag.sm)
    {
        case CAN_MCP2515_ODRIVE_SM_OP_LOAD_NODE1_WAIT:
        case CAN_MCP2515_ODRIVE_SM_OP_LOAD_NODE2_WAIT:
            if (dev->diag.spi_done != 0U)
            {
                dev->diag.spi_done = 0U;
                dev->diag.sm = (dev->diag.current_phase == CAN_MCP2515_ODRIVE_PHASE_NODE1) ? CAN_MCP2515_ODRIVE_SM_OP_RTS_NODE1_WAIT : CAN_MCP2515_ODRIVE_SM_OP_RTS_NODE2_WAIT;
                return CAN_MCP2515_ODRIVE_STATUS_OK;
            }
            if (dev->diag.spi_busy == 0U)
            {
                dev->diag.current_phase = (dev->diag.sm == CAN_MCP2515_ODRIVE_SM_OP_LOAD_NODE1_WAIT) ? CAN_MCP2515_ODRIVE_PHASE_NODE1 : CAN_MCP2515_ODRIVE_PHASE_NODE2;
                can_mcp2515_odrive_prepare_node_payload(dev, dev->diag.current_phase, &can_id, &dlc, &is_rtr, payload);
                can_mcp2515_odrive_timebase_update(dev);
                can_mcp2515_odrive_update_node_cmd_cache(dev, dev->diag.current_phase);
                can_mcp2515_odrive_prepare_can_frame(dev, can_id, dlc, is_rtr, payload);
                return can_mcp2515_odrive_start_dma(dev, 15U);
            }
            return CAN_MCP2515_ODRIVE_STATUS_OK;

        case CAN_MCP2515_ODRIVE_SM_OP_RTS_NODE1_WAIT:
        case CAN_MCP2515_ODRIVE_SM_OP_RTS_NODE2_WAIT:
            if (dev->diag.spi_done != 0U)
            {
                dev->diag.spi_done = 0U;
                dev->diag.sm = (dev->diag.current_phase == CAN_MCP2515_ODRIVE_PHASE_NODE1) ? CAN_MCP2515_ODRIVE_SM_OP_WAIT_TX_NODE1_WAIT : CAN_MCP2515_ODRIVE_SM_OP_WAIT_TX_NODE2_WAIT;
                dev->diag.tx_poll_count = 0U;
                return CAN_MCP2515_ODRIVE_STATUS_OK;
            }
            if (dev->diag.spi_busy == 0U)
            {
                can_mcp2515_odrive_prepare_rts_txb0(dev);
                return can_mcp2515_odrive_start_dma(dev, 1U);
            }
            return CAN_MCP2515_ODRIVE_STATUS_OK;

        case CAN_MCP2515_ODRIVE_SM_OP_WAIT_TX_NODE1_WAIT:
        case CAN_MCP2515_ODRIVE_SM_OP_WAIT_TX_NODE2_WAIT:
            if (dev->diag.spi_done != 0U)
            {
                dev->diag.spi_done = 0U;
                dev->diag.last_txb0ctrl = dev->spi_rx_buf[2];
                if ((dev->diag.last_txb0ctrl & MCP2515_TXBCTRL_TXREQ) == 0U)
                {
                    dev->diag.current_tx_ok[dev->diag.current_phase] = 1U;
                    if (can_mcp2515_odrive_current_op_requires_reply(dev) != 0U)
                    {
                        can_mcp2515_odrive_update_expected_reply(dev, dev->diag.current_phase);
                        dev->diag.current_expect_reply = 1U;
                        dev->diag.sm = (dev->diag.current_phase == CAN_MCP2515_ODRIVE_PHASE_NODE1) ? CAN_MCP2515_ODRIVE_SM_OP_WAIT_REPLY_NODE1 : CAN_MCP2515_ODRIVE_SM_OP_WAIT_REPLY_NODE2;
                    }
                    else
                    {
                        if (dev->diag.current_phase == CAN_MCP2515_ODRIVE_PHASE_NODE1)
                        {
                            dev->diag.sm = CAN_MCP2515_ODRIVE_SM_OP_LOAD_NODE2_WAIT;
                        }
                        else
                        {
                            dev->diag.sm = CAN_MCP2515_ODRIVE_SM_OP_FINISH;
                        }
                    }
                    return CAN_MCP2515_ODRIVE_STATUS_OK;
                }
                dev->diag.tx_poll_count++;
                if (dev->diag.tx_poll_count > dev->diag.tx_poll_limit)
                {
                    dev->diag.last_status = CAN_MCP2515_ODRIVE_STATUS_TIMEOUT;
                    dev->diag.sm = CAN_MCP2515_ODRIVE_SM_ERROR_RECOVERY;
                }
                return CAN_MCP2515_ODRIVE_STATUS_OK;
            }
            if (dev->diag.spi_busy == 0U)
            {
                can_mcp2515_odrive_prepare_read_reg(dev, MCP2515_REG_TXB0CTRL);
                return can_mcp2515_odrive_start_dma(dev, 3U);
            }
            return CAN_MCP2515_ODRIVE_STATUS_OK;

        case CAN_MCP2515_ODRIVE_SM_OP_WAIT_REPLY_NODE1:
        case CAN_MCP2515_ODRIVE_SM_OP_WAIT_REPLY_NODE2:
            if ((dev->diag.int_pending != 0U) || (HAL_GPIO_ReadPin(dev->int_port, dev->int_pin) == GPIO_PIN_RESET))
            {
                return can_mcp2515_odrive_service_interrupt_start(dev, dev->diag.sm);
            }
            if (dev->diag.current_reply_ok[dev->diag.current_phase] != 0U)
            {
                dev->diag.current_expect_reply = 0U;
                if (dev->diag.current_phase == CAN_MCP2515_ODRIVE_PHASE_NODE1)
                {
                    dev->diag.sm = CAN_MCP2515_ODRIVE_SM_OP_LOAD_NODE2_WAIT;
                }
                else
                {
                    dev->diag.sm = CAN_MCP2515_ODRIVE_SM_OP_FINISH;
                }
                return CAN_MCP2515_ODRIVE_STATUS_OK;
            }
            if ((dev->diag.time_us - dev->diag.last_poll_start_us) > (uint64_t)dev->reply_timeout_us)
            {
                can_mcp2515_odrive_node_t *node = can_mcp2515_odrive_node_by_index(dev, dev->diag.current_phase);
                node->timeout_count++;
                node->stale = 1U;
                if (dev->diag.current_phase == CAN_MCP2515_ODRIVE_PHASE_NODE1)
                {
                    dev->node1.encoder.timeout_count++;
                    dev->diag.sm = CAN_MCP2515_ODRIVE_SM_OP_LOAD_NODE2_WAIT;
                }
                else
                {
                    dev->node2.encoder.timeout_count++;
                    dev->diag.sm = CAN_MCP2515_ODRIVE_SM_OP_FINISH;
                }
                dev->diag.current_expect_reply = 0U;
                dev->diag.last_status = CAN_MCP2515_ODRIVE_STATUS_TIMEOUT;
            }
            return CAN_MCP2515_ODRIVE_STATUS_OK;

        case CAN_MCP2515_ODRIVE_SM_OP_FINISH:
            can_mcp2515_odrive_timebase_update(dev);
            can_mcp2515_odrive_finish_current_op(dev);
            return CAN_MCP2515_ODRIVE_STATUS_OK;

        default:
            return CAN_MCP2515_ODRIVE_STATUS_OK;
    }
}

can_mcp2515_odrive_status_t can_mcp2515_odrive_process(can_mcp2515_odrive_t *dev)
{
    can_mcp2515_odrive_status_t st;

    if (dev == NULL)
    {
        return CAN_MCP2515_ODRIVE_STATUS_BAD_PARAM;
    }

    st = can_mcp2515_odrive_check_runtime(dev);
    if (st != CAN_MCP2515_ODRIVE_STATUS_OK)
    {
        return st;
    }

    if (dev->diag.sm == CAN_MCP2515_ODRIVE_SM_ERROR_RECOVERY)
    {
        dev->diag.reinit_count++;
        st = can_mcp2515_odrive_init(dev);
        return st;
    }

    if ((dev->diag.sm == CAN_MCP2515_ODRIVE_SM_IDLE) && ((dev->diag.int_pending != 0U) || (HAL_GPIO_ReadPin(dev->int_port, dev->int_pin) == GPIO_PIN_RESET)))
    {
        return can_mcp2515_odrive_service_interrupt_start(dev, CAN_MCP2515_ODRIVE_SM_IDLE);
    }

    if ((dev->diag.sm >= CAN_MCP2515_ODRIVE_SM_IRQ_READ_CANINTF_WAIT) && (dev->diag.sm <= CAN_MCP2515_ODRIVE_SM_IRQ_CLEAR_FLAGS_WAIT))
    {
        return can_mcp2515_odrive_process_irq_state(dev);
    }

    if ((dev->diag.sm == CAN_MCP2515_ODRIVE_SM_IDLE) && (dev->diag.spi_busy == 0U))
    {
        st = can_mcp2515_odrive_start_next_op_if_needed(dev);
        if (st != CAN_MCP2515_ODRIVE_STATUS_OK)
        {
            return st;
        }
    }

    return can_mcp2515_odrive_process_op_state(dev);
}

static can_mcp2515_odrive_status_t can_mcp2515_odrive_enqueue_simple(can_mcp2515_odrive_t *dev, can_mcp2515_odrive_op_t *op)
{
    if ((dev == NULL) || (op == NULL))
    {
        return CAN_MCP2515_ODRIVE_STATUS_BAD_PARAM;
    }
    if (dev->diag.is_initialized == 0U)
    {
        return CAN_MCP2515_ODRIVE_STATUS_NOT_READY;
    }
    return can_mcp2515_odrive_queue_push(dev, op);
}

can_mcp2515_odrive_status_t odrive_pair_set_axis_state(can_mcp2515_odrive_t *dev, uint32_t state1, uint32_t state2)
{
    can_mcp2515_odrive_op_t op;
    memset(&op, 0, sizeof(op));
    op.type = CAN_MCP2515_ODRIVE_OP_PAIR_SET_AXIS_STATE;
    op.cmd_id = CAN_MCP2515_ODRIVE_CMD_SET_AXIS_STATE;
    op.u32_a[0] = state1;
    op.u32_a[1] = state2;
    return can_mcp2515_odrive_enqueue_simple(dev, &op);
}

can_mcp2515_odrive_status_t odrive_pair_set_controller_mode(can_mcp2515_odrive_t *dev, uint32_t control_mode1, uint32_t input_mode1, uint32_t control_mode2, uint32_t input_mode2)
{
    can_mcp2515_odrive_op_t op;
    memset(&op, 0, sizeof(op));
    op.type = CAN_MCP2515_ODRIVE_OP_PAIR_SET_CONTROLLER_MODE;
    op.cmd_id = CAN_MCP2515_ODRIVE_CMD_SET_CONTROLLER_MODE;
    op.u32_a[0] = control_mode1;
    op.u32_b[0] = input_mode1;
    op.u32_a[1] = control_mode2;
    op.u32_b[1] = input_mode2;
    return can_mcp2515_odrive_enqueue_simple(dev, &op);
}

can_mcp2515_odrive_status_t odrive_pair_set_input_pos(can_mcp2515_odrive_t *dev, float pos1, int16_t vel_ff1_milli_rev_s, int16_t torque_ff1_milli_nm, float pos2, int16_t vel_ff2_milli_rev_s, int16_t torque_ff2_milli_nm)
{
    can_mcp2515_odrive_op_t op;
    memset(&op, 0, sizeof(op));
    op.type = CAN_MCP2515_ODRIVE_OP_PAIR_SET_INPUT_POS;
    op.cmd_id = CAN_MCP2515_ODRIVE_CMD_SET_INPUT_POS;
    op.f32_a[0] = pos1;
    op.i16_a[0] = vel_ff1_milli_rev_s;
    op.i16_b[0] = torque_ff1_milli_nm;
    op.f32_a[1] = pos2;
    op.i16_a[1] = vel_ff2_milli_rev_s;
    op.i16_b[1] = torque_ff2_milli_nm;
    return can_mcp2515_odrive_enqueue_simple(dev, &op);
}

can_mcp2515_odrive_status_t odrive_pair_set_input_vel(can_mcp2515_odrive_t *dev, float vel1_rev_s, float torque_ff1_nm, float vel2_rev_s, float torque_ff2_nm)
{
    can_mcp2515_odrive_op_t op;
    memset(&op, 0, sizeof(op));
    op.type = CAN_MCP2515_ODRIVE_OP_PAIR_SET_INPUT_VEL;
    op.cmd_id = CAN_MCP2515_ODRIVE_CMD_SET_INPUT_VEL;
    op.f32_a[0] = vel1_rev_s;
    op.f32_b[0] = torque_ff1_nm;
    op.f32_a[1] = vel2_rev_s;
    op.f32_b[1] = torque_ff2_nm;
    return can_mcp2515_odrive_enqueue_simple(dev, &op);
}

can_mcp2515_odrive_status_t odrive_pair_set_input_torque(can_mcp2515_odrive_t *dev, float torque1_nm, float torque2_nm)
{
    can_mcp2515_odrive_op_t op;
    memset(&op, 0, sizeof(op));
    op.type = CAN_MCP2515_ODRIVE_OP_PAIR_SET_INPUT_TORQUE;
    op.cmd_id = CAN_MCP2515_ODRIVE_CMD_SET_INPUT_TORQUE;
    op.f32_a[0] = torque1_nm;
    op.f32_a[1] = torque2_nm;
    return can_mcp2515_odrive_enqueue_simple(dev, &op);
}

can_mcp2515_odrive_status_t odrive_pair_set_limits(can_mcp2515_odrive_t *dev, float vel_limit1_rev_s, float current_limit1_a, float vel_limit2_rev_s, float current_limit2_a)
{
    can_mcp2515_odrive_op_t op;
    memset(&op, 0, sizeof(op));
    op.type = CAN_MCP2515_ODRIVE_OP_PAIR_SET_LIMITS;
    op.cmd_id = CAN_MCP2515_ODRIVE_CMD_SET_LIMITS;
    op.f32_a[0] = vel_limit1_rev_s;
    op.f32_b[0] = current_limit1_a;
    op.f32_a[1] = vel_limit2_rev_s;
    op.f32_b[1] = current_limit2_a;
    return can_mcp2515_odrive_enqueue_simple(dev, &op);
}

can_mcp2515_odrive_status_t odrive_pair_set_vel_gains(can_mcp2515_odrive_t *dev, float vel_gain1, float vel_integrator_gain1, float vel_gain2, float vel_integrator_gain2)
{
    can_mcp2515_odrive_op_t op;
    memset(&op, 0, sizeof(op));
    op.type = CAN_MCP2515_ODRIVE_OP_PAIR_SET_VEL_GAINS;
    op.cmd_id = CAN_MCP2515_ODRIVE_CMD_SET_VEL_GAINS;
    op.f32_a[0] = vel_gain1;
    op.f32_b[0] = vel_integrator_gain1;
    op.f32_a[1] = vel_gain2;
    op.f32_b[1] = vel_integrator_gain2;
    return can_mcp2515_odrive_enqueue_simple(dev, &op);
}

can_mcp2515_odrive_status_t odrive_pair_clear_errors(can_mcp2515_odrive_t *dev, uint8_t identify)
{
    can_mcp2515_odrive_op_t op;
    memset(&op, 0, sizeof(op));
    op.type = CAN_MCP2515_ODRIVE_OP_PAIR_CLEAR_ERRORS;
    op.cmd_id = CAN_MCP2515_ODRIVE_CMD_CLEAR_ERRORS;
    op.identify = identify;
    return can_mcp2515_odrive_enqueue_simple(dev, &op);
}

can_mcp2515_odrive_status_t odrive_pair_estop(can_mcp2515_odrive_t *dev)
{
    can_mcp2515_odrive_op_t op;
    memset(&op, 0, sizeof(op));
    op.type = CAN_MCP2515_ODRIVE_OP_PAIR_ESTOP;
    op.cmd_id = CAN_MCP2515_ODRIVE_CMD_ESTOP;
    return can_mcp2515_odrive_enqueue_simple(dev, &op);
}

can_mcp2515_odrive_status_t odrive_pair_request_encoder_estimates(can_mcp2515_odrive_t *dev)
{
    can_mcp2515_odrive_op_t op;
    memset(&op, 0, sizeof(op));
    op.type = CAN_MCP2515_ODRIVE_OP_PAIR_REQ_ENCODER;
    op.cmd_id = CAN_MCP2515_ODRIVE_CMD_GET_ENCODER_ESTIMATES;
    return can_mcp2515_odrive_enqueue_simple(dev, &op);
}

can_mcp2515_odrive_status_t odrive_pair_request_raw_rtr(can_mcp2515_odrive_t *dev, uint8_t cmd_id)
{
    can_mcp2515_odrive_op_t op;
    memset(&op, 0, sizeof(op));
    op.type = CAN_MCP2515_ODRIVE_OP_PAIR_REQ_RAW_RTR;
    op.cmd_id = (uint8_t)(cmd_id & 0x1FU);
    return can_mcp2515_odrive_enqueue_simple(dev, &op);
}

can_mcp2515_odrive_status_t odrive_pair_send_raw(can_mcp2515_odrive_t *dev, uint8_t cmd_id, const uint8_t data1[8], uint8_t dlc1, const uint8_t data2[8], uint8_t dlc2)
{
    can_mcp2515_odrive_op_t op;

    if ((dlc1 > 8U) || (dlc2 > 8U))
    {
        return CAN_MCP2515_ODRIVE_STATUS_BAD_PARAM;
    }

    memset(&op, 0, sizeof(op));
    op.type = CAN_MCP2515_ODRIVE_OP_PAIR_SEND_RAW;
    op.cmd_id = (uint8_t)(cmd_id & 0x1FU);
    op.raw_dlc[0] = dlc1;
    op.raw_dlc[1] = dlc2;
    if ((data1 != NULL) && (dlc1 != 0U))
    {
        memcpy(op.raw_data[0], data1, dlc1);
    }
    if ((data2 != NULL) && (dlc2 != 0U))
    {
        memcpy(op.raw_data[1], data2, dlc2);
    }
    return can_mcp2515_odrive_enqueue_simple(dev, &op);
}

can_mcp2515_odrive_status_t odrive_pair_sdo_read_u32(can_mcp2515_odrive_t *dev, uint16_t endpoint1, uint16_t endpoint2)
{
    can_mcp2515_odrive_op_t op;
    memset(&op, 0, sizeof(op));
    op.type = CAN_MCP2515_ODRIVE_OP_PAIR_SDO_READ_U32;
    op.cmd_id = CAN_MCP2515_ODRIVE_CMD_RX_SDO;
    op.u32_a[0] = endpoint1;
    op.u32_a[1] = endpoint2;
    return can_mcp2515_odrive_enqueue_simple(dev, &op);
}

can_mcp2515_odrive_status_t odrive_pair_sdo_write_u32(can_mcp2515_odrive_t *dev, uint16_t endpoint1, uint32_t value1, uint16_t endpoint2, uint32_t value2)
{
    can_mcp2515_odrive_op_t op;
    memset(&op, 0, sizeof(op));
    op.type = CAN_MCP2515_ODRIVE_OP_PAIR_SDO_WRITE_U32;
    op.cmd_id = CAN_MCP2515_ODRIVE_CMD_RX_SDO;
    op.u32_a[0] = endpoint1;
    op.u32_b[0] = value1;
    op.u32_a[1] = endpoint2;
    op.u32_b[1] = value2;
    return can_mcp2515_odrive_enqueue_simple(dev, &op);
}

can_mcp2515_odrive_status_t can_mcp2515_odrive_format_diag_line(can_mcp2515_odrive_t *dev, char *buf, uint16_t buf_len)
{
    int n;

    if ((dev == NULL) || (buf == NULL) || (buf_len == 0U))
    {
        return CAN_MCP2515_ODRIVE_STATUS_BAD_PARAM;
    }

    can_mcp2515_odrive_timebase_update(dev);
    n = snprintf(buf,
                 (size_t)buf_len,
                 "CAN v%s us=%lu sm=%u q=%u irq=%lu hb1=%lu hb2=%lu enc1=%lu enc2=%lu pair_dt=%lu last_id=0x%03lX eflg=0x%02X",
                 CAN_MCP2515_ODRIVE_DRIVER_VERSION_STR,
                 (unsigned long)dev->diag.time_us,
                 (unsigned int)dev->diag.sm,
                 (unsigned int)dev->diag.queue_count,
                 (unsigned long)dev->diag.exti_count,
                 (unsigned long)dev->node1.heartbeat.rx_count,
                 (unsigned long)dev->node2.heartbeat.rx_count,
                 (unsigned long)dev->node1.encoder.rx_count,
                 (unsigned long)dev->node2.encoder.rx_count,
                 (unsigned long)dev->pair.pair_dt_us,
                 (unsigned long)dev->diag.last_id,
                 (unsigned int)dev->diag.last_eflg);
    if (n < 0)
    {
        return CAN_MCP2515_ODRIVE_STATUS_HAL_ERROR;
    }

    return CAN_MCP2515_ODRIVE_STATUS_OK;
}
