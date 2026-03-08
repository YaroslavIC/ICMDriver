
#ifndef MCP2515_H
#define MCP2515_H

#include "main.h"

// Пример использования:
//
// mcp2515_t can1;
// mcp2515_status_t st;
//
// can1.hspi = &hspi3;
// can1.htim_sched = &htim2;
// can1.cs_port = MCP2515_CS_GPIO_Port;
// can1.cs_pin = MCP2515_CS_Pin;
// can1.int_port = MCP2515_INT_GPIO_Port;
// can1.int_pin = MCP2515_INT_Pin;
// can1.cpu_hz = 84000000U;
// can1.mcp2515_osc_hz = 8000000U;
// can1.can_bitrate = 500000U;
// can1.poll_rate_hz_per_node = 200U;
// can1.response_timeout_us = 300U;
// can1.node_id_1 = 1U;
// can1.node_id_2 = 2U;
// can1.can_mode = MCP2515_MODE_LOOPBACK;
//
// st = mcp2515_init(&can1);
// if (st != MCP2515_STATUS_OK)
// {
//     Error_Handler();
// }
//
// while (1)
// {
//     st = mcp2515_poll(&can1);
//     if (st != MCP2515_STATUS_OK)
//     {
//         Error_Handler();
//     }
//
//     st = mcp2515_pop_sample(&can1);
//     if (st == MCP2515_STATUS_OK)
//     {
//         // Данные лежат в can1.sample_out
//     }
// }

// Версия драйвера 0.1.0
#define MCP2515_DRIVER_VERSION_MAJOR                0U
#define MCP2515_DRIVER_VERSION_MINOR                1U
#define MCP2515_DRIVER_VERSION_PATCH                0U
#define MCP2515_DRIVER_VERSION_U32                  0x000100U

#define MCP2515_RX_FIFO_LEN                         64U
// Пример: poll_rate_hz_per_node = 200, два узла -> scheduler_rate_hz = 400
#define MCP2515_SCHED_NODE_COUNT                    2U

#define MCP2515_ODRIVE_CMD_GET_ENCODER_ESTIMATES    0x09U
// Пример: node_id = 1 -> can_id = (1 << 5) | 0x09 = 0x29
// Пример: node_id = 2 -> can_id = (2 << 5) | 0x09 = 0x49

#define MCP2515_STD_ID_MASK                         0x07FFU
// Пример кодирования standard id 0x29:
// sidh = 0x29 >> 3 = 0x05
// sidl = (0x29 & 0x07) << 5 = 0x20

#define MCP2515_SPI_FRAME_MAX_LEN                   16U
#define MCP2515_SPI_TIMEOUT_MS                      10U
#define MCP2515_RESET_DELAY_MS                      2U

#define MCP2515_CMD_RESET                           0xC0U
#define MCP2515_CMD_READ                            0x03U
#define MCP2515_CMD_WRITE                           0x02U
#define MCP2515_CMD_BIT_MODIFY                      0x05U
#define MCP2515_CMD_RTS_TXB0                        0x81U

#define MCP2515_REG_RXF0SIDH                        0x00U
#define MCP2515_REG_RXF1SIDH                        0x04U
#define MCP2515_REG_RXF2SIDH                        0x08U
#define MCP2515_REG_RXF3SIDH                        0x10U
#define MCP2515_REG_RXF4SIDH                        0x14U
#define MCP2515_REG_RXF5SIDH                        0x18U
#define MCP2515_REG_RXM0SIDH                        0x20U
#define MCP2515_REG_RXM1SIDH                        0x24U
#define MCP2515_REG_CNF3                            0x28U
#define MCP2515_REG_CNF2                            0x29U
#define MCP2515_REG_CNF1                            0x2AU
#define MCP2515_REG_CANINTE                         0x2BU
#define MCP2515_REG_CANINTF                         0x2CU
#define MCP2515_REG_EFLG                            0x2DU
#define MCP2515_REG_CANSTAT                         0x0EU
#define MCP2515_REG_CANCTRL                         0x0FU
#define MCP2515_REG_TXB0SIDH                        0x31U
#define MCP2515_REG_TXB0SIDL                        0x32U
#define MCP2515_REG_TXB0EID8                        0x33U
#define MCP2515_REG_TXB0EID0                        0x34U
#define MCP2515_REG_TXB0DLC                         0x35U
#define MCP2515_REG_RXB0CTRL                        0x60U
#define MCP2515_REG_RXB1CTRL                        0x70U

#define MCP2515_CANINTE_RX0IE                       0x01U
#define MCP2515_CANINTE_RX1IE                       0x02U

#define MCP2515_CANINTF_RX0IF                       0x01U
#define MCP2515_CANINTF_RX1IF                       0x02U

#define MCP2515_CANCTRL_REQOP_MASK                  0xE0U
#define MCP2515_CANCTRL_REQOP_NORMAL                0x00U
#define MCP2515_CANCTRL_REQOP_LOOPBACK              0x40U
#define MCP2515_CANCTRL_REQOP_CONFIG                0x80U

#define MCP2515_CANSTAT_OPMOD_MASK                  0xE0U

#define MCP2515_TXB0DLC_RTR                         0x40U
#define MCP2515_DLC_MASK                            0x0FU

#define MCP2515_RXB0CTRL_RXRTR                      0x08U
#define MCP2515_RXB1CTRL_RXRTR                      0x08U

typedef enum
{
    MCP2515_STATUS_OK = 0,
    MCP2515_STATUS_BUSY,
    MCP2515_STATUS_ERROR,
    MCP2515_STATUS_TIMEOUT,
    MCP2515_STATUS_BAD_PARAM,
    MCP2515_STATUS_HAL_ERROR,
    MCP2515_STATUS_NOT_READY
} mcp2515_status_t;

typedef enum
{
    MCP2515_MODE_CONFIG = 0,
    MCP2515_MODE_LOOPBACK,
    MCP2515_MODE_NORMAL
} mcp2515_mode_t;

typedef enum
{
    MCP2515_OP_NONE = 0,
    MCP2515_OP_TXB0_LOAD_RTR,
    MCP2515_OP_TXB0_RTS,
    MCP2515_OP_READ_CANINTF,
    MCP2515_OP_READ_RXB0,
    MCP2515_OP_READ_RXB1,
    MCP2515_OP_CLEAR_RX0IF,
    MCP2515_OP_CLEAR_RX1IF
} mcp2515_op_t;

typedef struct
{
    uint64_t tick64;
    uint8_t node_id;
    float position_rev;
    float velocity_rev_s;
} mcp2515_sample_t;

typedef struct
{
    SPI_HandleTypeDef *hspi;
    TIM_HandleTypeDef *htim_sched;

    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    GPIO_TypeDef *int_port;
    uint16_t int_pin;

    uint32_t cpu_hz;
    uint32_t mcp2515_osc_hz;
    uint32_t can_bitrate;
    uint16_t poll_rate_hz_per_node;
    uint16_t response_timeout_us;

    uint8_t node_id_1;
    uint8_t node_id_2;
    mcp2515_mode_t can_mode;

    uint32_t driver_version;

    uint16_t can_id_1;
    uint16_t can_id_2;

    uint8_t tx_buf[MCP2515_SPI_FRAME_MAX_LEN];
    uint8_t rx_buf[MCP2515_SPI_FRAME_MAX_LEN];
    uint16_t spi_len;

    volatile uint8_t is_initialized;
    volatile uint8_t spi_busy;
    volatile uint8_t spi_done;
    volatile uint8_t spi_error;

    volatile uint8_t int_pending;
    volatile uint8_t sched_tick;
    volatile uint8_t wait_response;

    volatile uint8_t rx_fifo_head;
    volatile uint8_t rx_fifo_tail;
    volatile uint8_t rx_fifo_count;

    mcp2515_sample_t rx_fifo[MCP2515_RX_FIFO_LEN];
    mcp2515_sample_t sample_out;

    uint32_t rx_lost_cnt;
    uint32_t rx_ok_cnt;
    uint32_t timeout_cnt;
    uint32_t irq_cnt;
    uint32_t dma_err_cnt;

    uint8_t sched_slot;
    uint64_t req_tick64;
    uint64_t irq_tick64;

    uint32_t tick_hi32;
    uint32_t tick_last32;

    mcp2515_op_t op;
    mcp2515_status_t last_status;
} mcp2515_t;

/// Инициализация драйвера, MCP2515, DWT и таймера планировщика.
mcp2515_status_t mcp2515_init(mcp2515_t *dev);

/// Обслуживание автомата драйвера в основном цикле.
mcp2515_status_t mcp2515_poll(mcp2515_t *dev);

/// Извлечь один готовый sample из FIFO в поле sample_out структуры.
mcp2515_status_t mcp2515_pop_sample(mcp2515_t *dev);

/// Привязка к HAL_GPIO_EXTI_Callback.
void mcp2515_exti_callback(mcp2515_t *dev, uint16_t gpio_pin);

/// Привязка к HAL_SPI_TxRxCpltCallback.
void mcp2515_spi_txcplt_callback(mcp2515_t *dev, SPI_HandleTypeDef *hspi);

/// Привязка к HAL_SPI_ErrorCallback.
void mcp2515_spi_error_callback(mcp2515_t *dev, SPI_HandleTypeDef *hspi);

/// Привязка к HAL_TIM_PeriodElapsedCallback.
void mcp2515_tim_period_elapsed_callback(mcp2515_t *dev, TIM_HandleTypeDef *htim);

#endif
