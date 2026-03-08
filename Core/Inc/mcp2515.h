#ifndef MCP2515_H
#define MCP2515_H

#include "main.h"

// Пример использования
// mcp2515_t can1;
// mcp2515_status_t st;
//
// can1.hspi = &hspi3;
// can1.htim_sched = &htim2;
// can1.cs_port = GPIOB;
// can1.cs_pin = GPIO_PIN_6;
// can1.int_port = GPIOB;
// can1.int_pin = GPIO_PIN_7;
// can1.cpu_hz = 84000000U;
// can1.mcp2515_osc_hz = 8000000U;
// can1.can_bitrate = 250000U;
// can1.node_id_1 = 1U;
// can1.node_id_2 = 2U;
// can1.can_mode = MCP2515_MODE_NORMAL;
//
// st = mcp2515_init(&can1);
// if (st != MCP2515_STATUS_OK)
// {
//     Error_Handler();
// }
//
// while (1)
// {
//     (void)mcp2515_poll(&can1);
//
//     if (can1.hb_1.valid != 0U)
//     {
//         // can1.hb_1.axis_error
//         // can1.hb_1.axis_state
//         // can1.hb_1.procedure_result
//         // can1.hb_1.trajectory_done
//         // can1.hb_1.tick64
//     }
//
//     if (can1.hb_2.valid != 0U)
//     {
//         // can1.hb_2.axis_error
//         // can1.hb_2.axis_state
//         // can1.hb_2.procedure_result
//         // can1.hb_2.trajectory_done
//         // can1.hb_2.tick64
//     }
// }

// Версия драйвера: 00.11
// Дата сборки: 07.03.26 21:54:41

#define MCP2515_DRIVER_VERSION_STR           "00.11"
#define MCP2515_DRIVER_BUILD_STR             "07.03.26 21:54:41"

#define MCP2515_SPI_TIMEOUT_MS               10U
#define MCP2515_RESET_DELAY_MS               2U
#define MCP2515_CANSTAT_MODE_MASK            0xE0U

#define MCP2515_CMD_RESET                    0xC0U
#define MCP2515_CMD_READ                     0x03U
#define MCP2515_CMD_WRITE                    0x02U
#define MCP2515_CMD_BIT_MODIFY               0x05U
#define MCP2515_CMD_READ_STATUS              0xA0U

#define MCP2515_REG_CANSTAT                  0x0EU
#define MCP2515_REG_CANCTRL                  0x0FU
#define MCP2515_REG_CNF3                     0x28U
#define MCP2515_REG_CNF2                     0x29U
#define MCP2515_REG_CNF1                     0x2AU
#define MCP2515_REG_CANINTE                  0x2BU
#define MCP2515_REG_CANINTF                  0x2CU
#define MCP2515_REG_EFLG                     0x2DU
#define MCP2515_REG_RXB0CTRL                 0x60U
#define MCP2515_REG_RXB0SIDH                 0x61U
#define MCP2515_REG_RXB1CTRL                 0x70U
#define MCP2515_REG_RXB1SIDH                 0x71U
#define MCP2515_REG_BFPCTRL                  0x0CU
#define MCP2515_REG_TXRTSCTRL                0x0DU

#define MCP2515_CANINTE_RX0IE                0x01U
#define MCP2515_CANINTE_RX1IE                0x02U
#define MCP2515_CANINTF_RX0IF                0x01U
#define MCP2515_CANINTF_RX1IF                0x02U

#define MCP2515_CANCTRL_REQOP_NORMAL         0x00U
#define MCP2515_CANCTRL_REQOP_LOOPBACK       0x40U
#define MCP2515_CANCTRL_REQOP_LISTEN_ONLY    0x60U
#define MCP2515_CANCTRL_REQOP_CONFIG         0x80U

#define MCP2515_RXB0CTRL_ACCEPT_ALL          0x64U
#define MCP2515_RXB1CTRL_ACCEPT_ALL          0x60U

#define MCP2515_CAN_CMD_HEARTBEAT            0x01U
// Пример расчёта: node_id = 1 -> ((1 << 5) | 0x01) = 0x21
// Пример расчёта: node_id = 2 -> ((2 << 5) | 0x01) = 0x41
#define MCP2515_ODRIVE_HEARTBEAT_ID(node_id) ((((uint16_t)(node_id)) << 5) | MCP2515_CAN_CMD_HEARTBEAT)

#define MCP2515_HEARTBEAT_DLC                8U
#define MCP2515_HEARTBEAT_LOST_FACTOR        3U


#define MCP2515_REG_CANSTAT  0x0EU
#define MCP2515_REG_CANINTE  0x2BU
#define MCP2515_REG_CANINTF  0x2CU
#define MCP2515_REG_EFLG     0x2DU



// Примеры битрейтов:
// 8 МГц  + 250000 -> поддерживается
// 8 МГц  + 500000 -> поддерживается
// 16 МГц + 250000 -> поддерживается
// 16 МГц + 500000 -> поддерживается
// 16 МГц + 1000000 -> поддерживается

typedef enum
{
    MCP2515_STATUS_OK = 0,
    MCP2515_STATUS_BUSY,
    MCP2515_STATUS_ERROR,
    MCP2515_STATUS_TIMEOUT,
    MCP2515_STATUS_BAD_PARAM,
    MCP2515_STATUS_HAL_ERROR,
    MCP2515_STATUS_NOT_READY,
    MCP2515_STATUS_UNSUPPORTED
} mcp2515_status_t;

typedef enum
{
    MCP2515_MODE_CONFIG = 0,
    MCP2515_MODE_LOOPBACK,
    MCP2515_MODE_NORMAL,
    MCP2515_MODE_LISTEN_ONLY
} mcp2515_mode_t;

typedef struct
{
    uint64_t tick64;
    uint32_t axis_error;
    uint8_t axis_state;
    uint8_t procedure_result;
    uint8_t trajectory_done;
    uint8_t valid;
    uint32_t rx_count;
} mcp2515_heartbeat_t;

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

    uint16_t hb_id_1;
    uint16_t hb_id_2;

    uint8_t tx_buf[16];
    uint8_t rx_buf[16];
    uint16_t spi_len;

    volatile uint8_t spi_busy;
    volatile uint8_t spi_done;
    volatile uint8_t spi_error;
    volatile uint8_t int_pending;
    volatile uint8_t sched_tick;
    volatile uint8_t rx_fifo_count;

    uint32_t tick_hi32;
    uint32_t tick_last32;
    uint64_t irq_tick64;

    uint8_t is_initialized;

    uint32_t irq_cnt;
    uint32_t dma_err_cnt;
    uint32_t hb_rx_cnt;
    uint32_t hb_bad_dlc_cnt;
    uint32_t rx_ok_cnt;
    uint32_t rx_lost_cnt;
    uint32_t timeout_cnt;
    uint32_t rx_ignored_cnt;

    mcp2515_sample_t sample_out;
    mcp2515_heartbeat_t hb_1;
    mcp2515_heartbeat_t hb_2;

    mcp2515_status_t last_status;
} mcp2515_t;

/// Инициализация драйвера и самого MCP2515
mcp2515_status_t mcp2515_init(mcp2515_t *dev);

/// Обслуживание принятых кадров heartbeat
mcp2515_status_t mcp2515_poll(mcp2515_t *dev);

/// Совместимость с предыдущей версией
mcp2515_status_t mcp2515_pop_sample(mcp2515_t *dev);

/// Обработчик EXTI от линии INT MCP2515
void mcp2515_exti_callback(mcp2515_t *dev, uint16_t gpio_pin);

/// Совместимость с предыдущей версией
void mcp2515_spi_txcplt_callback(mcp2515_t *dev, SPI_HandleTypeDef *hspi);

/// Совместимость с предыдущей версией
void mcp2515_spi_error_callback(mcp2515_t *dev, SPI_HandleTypeDef *hspi);

/// Совместимость с предыдущей версией
void mcp2515_tim_period_elapsed_callback(mcp2515_t *dev, TIM_HandleTypeDef *htim);

mcp2515_status_t mcp2515_debug_read_reg(mcp2515_t *dev, uint8_t reg, uint8_t *value);

#endif
