#ifndef CAN_MCP2515_ODRIVE_H
#define CAN_MCP2515_ODRIVE_H

#include "main.h"
#include <stdint.h>
#include <stddef.h>

// Пример использования:
//
// static can_mcp2515_odrive_t g_can;
//
// void app_can_init(void)
// {
//     memset(&g_can, 0, sizeof(g_can));
//     g_can.hspi = &hspi3;
//     g_can.cs_port = GPIOB;
//     g_can.cs_pin = GPIO_PIN_6;
//     g_can.int_port = GPIOB;
//     g_can.int_pin = GPIO_PIN_7;
//     g_can.mcp2515_osc_hz = 8000000U;
//     g_can.can_bitrate = 250000U;
//     g_can.node_id_1 = 1U;
//     g_can.node_id_2 = 2U;
//     g_can.mode = CAN_MCP2515_ODRIVE_MODE_NORMAL;
//     g_can.poll_pair_rate_hz = 250U;
//     g_can.reply_timeout_us = 3000U;
//     g_can.spi_timeout_us = 2000U;
//     g_can.queue_capacity = CAN_MCP2515_ODRIVE_QUEUE_CAPACITY;
//
//     if (can_mcp2515_odrive_init(&g_can) != CAN_MCP2515_ODRIVE_STATUS_OK)
//     {
//         Error_Handler();
//     }
//
//     can_mcp2515_odrive_enable_polling(&g_can, 1U);
// }
//
// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//     can_mcp2515_odrive_exti_callback(&g_can, GPIO_Pin);
// }
//
// void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
// {
//     can_mcp2515_odrive_spi_txrx_cplt_callback(&g_can, hspi);
// }
//
// void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
// {
//     can_mcp2515_odrive_spi_error_callback(&g_can, hspi);
// }
//
// int main(void)
// {
//     HAL_Init();
//     SystemClock_Config();
//     MX_GPIO_Init();
//     MX_DMA_Init();
//     MX_SPI3_Init();
//     app_can_init();
//
//     for (;;)
//     {
//         can_mcp2515_odrive_process(&g_can);
//     }
// }

#define CAN_MCP2515_ODRIVE_DRIVER_VERSION_STR "0.1"
#define CAN_MCP2515_ODRIVE_DRIVER_BUILD_STR   "09.03.26 00:00:00"

#define CAN_MCP2515_ODRIVE_QUEUE_CAPACITY 8U
#define CAN_MCP2515_ODRIVE_SPI_BUF_SIZE   32U

#define CAN_MCP2515_ODRIVE_CMD_GET_VERSION              0x00U
#define CAN_MCP2515_ODRIVE_CMD_HEARTBEAT                0x01U
#define CAN_MCP2515_ODRIVE_CMD_ESTOP                    0x02U
#define CAN_MCP2515_ODRIVE_CMD_GET_ERROR                0x03U
#define CAN_MCP2515_ODRIVE_CMD_RX_SDO                   0x04U
#define CAN_MCP2515_ODRIVE_CMD_TX_SDO                   0x05U
#define CAN_MCP2515_ODRIVE_CMD_ADDRESS                  0x06U
#define CAN_MCP2515_ODRIVE_CMD_SET_AXIS_STATE           0x07U
#define CAN_MCP2515_ODRIVE_CMD_GET_ENCODER_ESTIMATES    0x09U
#define CAN_MCP2515_ODRIVE_CMD_SET_CONTROLLER_MODE      0x0BU
#define CAN_MCP2515_ODRIVE_CMD_SET_INPUT_POS            0x0CU
#define CAN_MCP2515_ODRIVE_CMD_SET_INPUT_VEL            0x0DU
#define CAN_MCP2515_ODRIVE_CMD_SET_INPUT_TORQUE         0x0EU
#define CAN_MCP2515_ODRIVE_CMD_SET_LIMITS               0x0FU
#define CAN_MCP2515_ODRIVE_CMD_GET_IQ                   0x14U
#define CAN_MCP2515_ODRIVE_CMD_GET_TEMPERATURE          0x15U
#define CAN_MCP2515_ODRIVE_CMD_REBOOT                   0x16U
#define CAN_MCP2515_ODRIVE_CMD_GET_BUS_VOLTAGE_CURRENT  0x17U
#define CAN_MCP2515_ODRIVE_CMD_CLEAR_ERRORS             0x18U
#define CAN_MCP2515_ODRIVE_CMD_SET_ABSOLUTE_POSITION    0x19U
#define CAN_MCP2515_ODRIVE_CMD_SET_POS_GAIN             0x1AU
#define CAN_MCP2515_ODRIVE_CMD_SET_VEL_GAINS            0x1BU
#define CAN_MCP2515_ODRIVE_CMD_GET_TORQUES              0x1CU
#define CAN_MCP2515_ODRIVE_CMD_GET_POWERS               0x1DU

#define CAN_MCP2515_ODRIVE_SDO_OPCODE_READ              0x00U
#define CAN_MCP2515_ODRIVE_SDO_OPCODE_WRITE             0x01U

typedef enum
{
    CAN_MCP2515_ODRIVE_STATUS_OK = 0,
    CAN_MCP2515_ODRIVE_STATUS_BAD_PARAM,
    CAN_MCP2515_ODRIVE_STATUS_HAL_ERROR,
    CAN_MCP2515_ODRIVE_STATUS_TIMEOUT,
    CAN_MCP2515_ODRIVE_STATUS_NOT_READY,
    CAN_MCP2515_ODRIVE_STATUS_UNSUPPORTED,
    CAN_MCP2515_ODRIVE_STATUS_BUSY,
    CAN_MCP2515_ODRIVE_STATUS_QUEUE_FULL
} can_mcp2515_odrive_status_t;

typedef enum
{
    CAN_MCP2515_ODRIVE_MODE_NORMAL = 0,
    CAN_MCP2515_ODRIVE_MODE_LISTEN_ONLY = 1
} can_mcp2515_odrive_mode_t;

typedef enum
{
    CAN_MCP2515_ODRIVE_OP_NONE = 0,
    CAN_MCP2515_ODRIVE_OP_PAIR_SET_AXIS_STATE,
    CAN_MCP2515_ODRIVE_OP_PAIR_SET_CONTROLLER_MODE,
    CAN_MCP2515_ODRIVE_OP_PAIR_SET_INPUT_POS,
    CAN_MCP2515_ODRIVE_OP_PAIR_SET_INPUT_VEL,
    CAN_MCP2515_ODRIVE_OP_PAIR_SET_INPUT_TORQUE,
    CAN_MCP2515_ODRIVE_OP_PAIR_SET_LIMITS,
    CAN_MCP2515_ODRIVE_OP_PAIR_CLEAR_ERRORS,
    CAN_MCP2515_ODRIVE_OP_PAIR_ESTOP,
    CAN_MCP2515_ODRIVE_OP_PAIR_REQ_ENCODER,
    CAN_MCP2515_ODRIVE_OP_PAIR_REQ_RAW_RTR,
    CAN_MCP2515_ODRIVE_OP_PAIR_SEND_RAW,
    CAN_MCP2515_ODRIVE_OP_PAIR_SDO_READ_U32,
    CAN_MCP2515_ODRIVE_OP_PAIR_SDO_WRITE_U32
} can_mcp2515_odrive_op_type_t;

typedef enum
{
    CAN_MCP2515_ODRIVE_SM_STOPPED = 0,
    CAN_MCP2515_ODRIVE_SM_IDLE,
    CAN_MCP2515_ODRIVE_SM_IRQ_READ_CANINTF_WAIT,
    CAN_MCP2515_ODRIVE_SM_IRQ_READ_EFLG_WAIT,
    CAN_MCP2515_ODRIVE_SM_IRQ_READ_CANSTAT_WAIT,
    CAN_MCP2515_ODRIVE_SM_IRQ_READ_RXB0_WAIT,
    CAN_MCP2515_ODRIVE_SM_IRQ_READ_RXB1_WAIT,
    CAN_MCP2515_ODRIVE_SM_IRQ_CLEAR_FLAGS_WAIT,
    CAN_MCP2515_ODRIVE_SM_OP_LOAD_NODE1_WAIT,
    CAN_MCP2515_ODRIVE_SM_OP_RTS_NODE1_WAIT,
    CAN_MCP2515_ODRIVE_SM_OP_WAIT_TX_NODE1_WAIT,
    CAN_MCP2515_ODRIVE_SM_OP_WAIT_REPLY_NODE1,
    CAN_MCP2515_ODRIVE_SM_OP_LOAD_NODE2_WAIT,
    CAN_MCP2515_ODRIVE_SM_OP_RTS_NODE2_WAIT,
    CAN_MCP2515_ODRIVE_SM_OP_WAIT_TX_NODE2_WAIT,
    CAN_MCP2515_ODRIVE_SM_OP_WAIT_REPLY_NODE2,
    CAN_MCP2515_ODRIVE_SM_OP_FINISH,
    CAN_MCP2515_ODRIVE_SM_ERROR_RECOVERY
} can_mcp2515_odrive_sm_t;

typedef struct
{
    uint8_t valid;
    uint32_t axis_error;
    uint8_t axis_state;
    uint8_t procedure_result;
    uint8_t trajectory_done;
    uint32_t rx_count;
    uint64_t ts_cycles_64;
    uint64_t ts_us;
} can_mcp2515_odrive_heartbeat_t;

typedef struct
{
    uint8_t valid;
    float pos_estimate;
    float vel_estimate;
    uint32_t req_count;
    uint32_t rx_count;
    uint32_t timeout_count;
    uint64_t req_ts_cycles_64;
    uint64_t req_ts_us;
    uint64_t rsp_ts_cycles_64;
    uint64_t rsp_ts_us;
    uint64_t latency_us;
} can_mcp2515_odrive_encoder_t;

typedef struct
{
    uint8_t valid;
    uint32_t active_errors;
    uint32_t disarm_reason;
    uint64_t ts_us;
} can_mcp2515_odrive_error_t;

typedef struct
{
    uint8_t valid;
    float bus_voltage;
    float bus_current;
    uint64_t ts_us;
} can_mcp2515_odrive_bus_t;

typedef struct
{
    uint8_t valid;
    float iq_setpoint;
    float iq_measured;
    uint64_t ts_us;
} can_mcp2515_odrive_iq_t;

typedef struct
{
    uint8_t valid;
    uint16_t endpoint_id;
    uint32_t value_u32;
    float value_f32;
    uint64_t ts_us;
} can_mcp2515_odrive_sdo_t;

typedef struct
{
    uint8_t valid;
    uint16_t can_id;
    uint8_t cmd_id;
    uint8_t dlc;
    uint8_t is_rtr;
    uint8_t data[8];
    uint64_t ts_cycles_64;
    uint64_t ts_us;
} can_mcp2515_odrive_raw_msg_t;

typedef struct
{
    uint8_t node_id;
    uint8_t online;
    uint8_t fresh;
    uint8_t stale;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t timeout_count;
    uint32_t parse_error_count;
    uint64_t last_cmd_ts_us;
    float last_torque_cmd;
    float last_vel_cmd;
    float last_pos_cmd;
    uint32_t last_axis_state_cmd;
    uint32_t last_control_mode_cmd;
    uint32_t last_input_mode_cmd;
    can_mcp2515_odrive_heartbeat_t heartbeat;
    can_mcp2515_odrive_encoder_t encoder;
    can_mcp2515_odrive_error_t error;
    can_mcp2515_odrive_bus_t bus;
    can_mcp2515_odrive_iq_t iq;
    can_mcp2515_odrive_sdo_t sdo;
    can_mcp2515_odrive_raw_msg_t last_rx;
} can_mcp2515_odrive_node_t;

typedef struct
{
    uint8_t ready;
    uint8_t op_active;
    uint32_t op_seq;
    can_mcp2515_odrive_op_type_t last_op_type;
    float pos1;
    float vel1;
    float pos2;
    float vel2;
    uint64_t ts1_us;
    uint64_t ts2_us;
    uint64_t pair_complete_ts_us;
    uint64_t pair_dt_us;
    float last_torque1;
    float last_torque2;
    float last_vel1;
    float last_vel2;
    float last_pos1;
    float last_pos2;
    uint64_t last_pair_cmd_ts_us;
} can_mcp2515_odrive_pair_t;

typedef struct
{
    uint32_t exti_count;
    uint32_t spi_dma_count;
    uint32_t spi_error_count;
    uint32_t reinit_count;
    uint32_t overflow_count;
    uint32_t busoff_count;
    uint8_t int_pending;
    uint8_t spi_busy;
    uint8_t spi_done;
    uint8_t spi_error;
    uint8_t is_initialized;
    uint8_t poll_enabled;
    uint8_t service_rx0;
    uint8_t service_rx1;
    uint8_t service_clear_mask;
    uint8_t current_phase;
    uint8_t current_node_index;
    uint8_t current_expect_reply;
    uint8_t current_reply_ok[2];
    uint8_t current_tx_ok[2];
    uint8_t current_started;
    uint8_t current_is_periodic;
    uint8_t current_cmd_id;
    uint8_t queue_head;
    uint8_t queue_tail;
    uint8_t queue_count;
    uint8_t queue_capacity;
    uint8_t tx_poll_count;
    uint8_t tx_poll_limit;
    uint8_t last_canstat;
    uint8_t last_caninte;
    uint8_t last_canintf;
    uint8_t last_eflg;
    uint8_t last_txb0ctrl;
    uint16_t expected_reply_id;
    uint8_t expected_reply_cmd;
    uint32_t last_id;
    uint8_t last_dlc;
    uint8_t last_data[8];
    uint64_t last_dma_start_us;
    uint64_t last_poll_start_us;
    uint64_t next_periodic_poll_us;
    uint32_t time_high32;
    uint32_t time_last_low32;
    uint64_t time_cycles_64;
    uint64_t time_us;
    can_mcp2515_odrive_sm_t sm;
    can_mcp2515_odrive_sm_t resume_sm;
    can_mcp2515_odrive_status_t last_status;
} can_mcp2515_odrive_diag_t;

typedef struct
{
    can_mcp2515_odrive_op_type_t type;
    uint8_t cmd_id;
    uint8_t identify;
    uint8_t raw_dlc[2];
    uint8_t raw_data[2][8];
    uint8_t raw_is_rtr;
    uint32_t u32_a[2];
    uint32_t u32_b[2];
    int16_t i16_a[2];
    int16_t i16_b[2];
    float f32_a[2];
    float f32_b[2];
    float f32_c[2];
} can_mcp2515_odrive_op_t;

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *int_port;
    uint16_t int_pin;
    uint32_t mcp2515_osc_hz;
    uint32_t can_bitrate;
    uint8_t node_id_1;
    uint8_t node_id_2;
    uint16_t can_id_1_heartbeat;
    uint16_t can_id_2_heartbeat;
    uint16_t can_id_1_encoder;
    uint16_t can_id_2_encoder;
    can_mcp2515_odrive_mode_t mode;
    uint16_t poll_pair_rate_hz;
    uint32_t reply_timeout_us;
    uint32_t spi_timeout_us;
    can_mcp2515_odrive_node_t node1;
    can_mcp2515_odrive_node_t node2;
    can_mcp2515_odrive_pair_t pair;
    can_mcp2515_odrive_diag_t diag;
    can_mcp2515_odrive_op_t queue[CAN_MCP2515_ODRIVE_QUEUE_CAPACITY];
    can_mcp2515_odrive_op_t current_op;
    uint8_t spi_tx_buf[CAN_MCP2515_ODRIVE_SPI_BUF_SIZE];
    uint8_t spi_rx_buf[CAN_MCP2515_ODRIVE_SPI_BUF_SIZE];
} can_mcp2515_odrive_t;

can_mcp2515_odrive_status_t can_mcp2515_odrive_init(can_mcp2515_odrive_t *dev);
can_mcp2515_odrive_status_t can_mcp2515_odrive_process(can_mcp2515_odrive_t *dev);
void can_mcp2515_odrive_exti_callback(can_mcp2515_odrive_t *dev, uint16_t gpio_pin);
void can_mcp2515_odrive_spi_txrx_cplt_callback(can_mcp2515_odrive_t *dev, SPI_HandleTypeDef *hspi);
void can_mcp2515_odrive_spi_error_callback(can_mcp2515_odrive_t *dev, SPI_HandleTypeDef *hspi);
can_mcp2515_odrive_status_t can_mcp2515_odrive_enable_polling(can_mcp2515_odrive_t *dev, uint8_t enable);
can_mcp2515_odrive_status_t can_mcp2515_odrive_set_poll_pair_rate_hz(can_mcp2515_odrive_t *dev, uint16_t rate_hz);

can_mcp2515_odrive_status_t odrive_pair_set_axis_state(can_mcp2515_odrive_t *dev, uint32_t state1, uint32_t state2);
can_mcp2515_odrive_status_t odrive_pair_set_controller_mode(can_mcp2515_odrive_t *dev, uint32_t control_mode1, uint32_t input_mode1, uint32_t control_mode2, uint32_t input_mode2);
can_mcp2515_odrive_status_t odrive_pair_set_input_pos(can_mcp2515_odrive_t *dev, float pos1, int16_t vel_ff1_milli_rev_s, int16_t torque_ff1_milli_nm, float pos2, int16_t vel_ff2_milli_rev_s, int16_t torque_ff2_milli_nm);
can_mcp2515_odrive_status_t odrive_pair_set_input_vel(can_mcp2515_odrive_t *dev, float vel1_rev_s, float torque_ff1_nm, float vel2_rev_s, float torque_ff2_nm);
can_mcp2515_odrive_status_t odrive_pair_set_input_torque(can_mcp2515_odrive_t *dev, float torque1_nm, float torque2_nm);
can_mcp2515_odrive_status_t odrive_pair_set_limits(can_mcp2515_odrive_t *dev, float vel_limit1_rev_s, float current_limit1_a, float vel_limit2_rev_s, float current_limit2_a);
can_mcp2515_odrive_status_t odrive_pair_clear_errors(can_mcp2515_odrive_t *dev, uint8_t identify);
can_mcp2515_odrive_status_t odrive_pair_estop(can_mcp2515_odrive_t *dev);
can_mcp2515_odrive_status_t odrive_pair_request_encoder_estimates(can_mcp2515_odrive_t *dev);
can_mcp2515_odrive_status_t odrive_pair_request_raw_rtr(can_mcp2515_odrive_t *dev, uint8_t cmd_id);
can_mcp2515_odrive_status_t odrive_pair_send_raw(can_mcp2515_odrive_t *dev, uint8_t cmd_id, const uint8_t data1[8], uint8_t dlc1, const uint8_t data2[8], uint8_t dlc2);
can_mcp2515_odrive_status_t odrive_pair_sdo_read_u32(can_mcp2515_odrive_t *dev, uint16_t endpoint1, uint16_t endpoint2);
can_mcp2515_odrive_status_t odrive_pair_sdo_write_u32(can_mcp2515_odrive_t *dev, uint16_t endpoint1, uint32_t value1, uint16_t endpoint2, uint32_t value2);
can_mcp2515_odrive_status_t can_mcp2515_odrive_format_diag_line(can_mcp2515_odrive_t *dev, char *buf, uint16_t buf_len);

#endif
