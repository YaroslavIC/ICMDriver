// balance_lqr.h
// LQR-регулятор балансировки для STM32F401CCU6
// Версия драйвера: 0.1.1
//
// ПРАВИЛА:
// - Все данные хранятся в структурах и передаются через структуры.
// - Все функции возвращают только статус выполнения.
// - Все комментарии только через //.
// - Драйвер не использует динамическую память.

#ifndef BALANCE_LQR_H
#define BALANCE_LQR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BALANCE_LQR_VERSION_MAJOR        (0u)
#define BALANCE_LQR_VERSION_MINOR        (1u)
#define BALANCE_LQR_VERSION_PATCH        (1u)
#define BALANCE_LQR_STATE_DIM            (4u)

typedef enum
{
    BALANCE_LQR_STATUS_OK = 0,
    BALANCE_LQR_STATUS_BAD_PARAM,
    BALANCE_LQR_STATUS_NOT_READY
} balance_lqr_status_t;

typedef struct
{
    float pitch_rad;
    float pitch_rate_rad_s;
    float wheel_pos_avg;
    float wheel_vel_avg;
} balance_lqr_state_input_t;

typedef struct
{
    uint8_t enabled;
    float K[BALANCE_LQR_STATE_DIM];
    float u_min;
    float u_max;
} balance_lqr_config_t;

typedef struct
{
    uint8_t valid;
    uint32_t step_count;
    float x[BALANCE_LQR_STATE_DIM];
    float u_unsat;
    float u_sat;
} balance_lqr_state_t;

typedef struct
{
    balance_lqr_config_t cfg;
    balance_lqr_state_t state;
} balance_lqr_t;

balance_lqr_status_t balance_lqr_init(balance_lqr_t *dev, const balance_lqr_config_t *cfg);
balance_lqr_status_t balance_lqr_reset(balance_lqr_t *dev);
balance_lqr_status_t balance_lqr_set_config(balance_lqr_t *dev, const balance_lqr_config_t *cfg);
balance_lqr_status_t balance_lqr_step(balance_lqr_t *dev, const balance_lqr_state_input_t *in, float *u_out);

#ifdef __cplusplus
}
#endif

#endif
