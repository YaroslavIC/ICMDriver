#ifndef BALANCE_H
#define BALANCE_H

#ifdef __cplusplus
extern "C" {
#endif

#define BALANCE_MODULE_VERSION           "B11"

#include "main.h"
// Версия 10.10 05.04.26 12:05:00
#include <stdint.h>

// Версия: 01.00 29.03.26 12:55:00
// Назначение:
// Модуль балансировки робота. Содержит режимы IDLE/CATCH/BALANCE и
// подготовку к будущим режимам движения. Модуль не вызывает HAL и не знает
// ничего о USB/CAN/Flash. Он работает только со структурами входа/выхода.
//
// Пример инициализации:
// balance_controller_t bal;
// balance_input_t in;
// balance_command_t cmd;
// balance_output_t out;
// memset(&bal, 0, sizeof(bal));
// if (balance_init(&bal) != BALANCE_STATUS_OK)
// {
//     Error_Handler();
// }
// bal.params.control_k_pitch = 0.80f;
//
// Пример цикла:
// in.now_ms = HAL_GetTick();
// in.pitch_rad = snap.pitch_rad;
// in.pitch_rate_rad_s = snap.pitch_rate_rad_s;
// in.wheel_vel_avg = snap.wheel_vel_avg;
// in.wheel_pos_avg = snap.wheel_pos_avg;
// in.wheel_vel_diff = snap.wheel_vel_diff;
// in.valid = snap.valid;
// in.control_enabled = 1u;
// cmd.motion_fwd_cmd = 0.00f;
// cmd.motion_turn_cmd = 0.00f;
// if (balance_step(&bal, &in, &cmd, &out) == BALANCE_STATUS_OK)
// {
//     // out.u_left / out.u_right -> ODrive
// }

typedef enum
{
    BALANCE_STATUS_OK = 0,
    BALANCE_STATUS_BAD_ARG,
    BALANCE_STATUS_BAD_STATE
} balance_status_t;

typedef enum
{
    BALANCE_MODE_IDLE = 0,
    BALANCE_MODE_CATCH,
    BALANCE_MODE_BALANCE,
    BALANCE_MODE_SAFE_STOP
} balance_mode_t;




typedef struct
{
    float control_u_limit;
    float control_k_pitch;
    float control_k_pitch_rate;
    float control_k_wheel_vel;
    float control_k_wheel_pos;
    float control_k_sync;
    float control_u_sync_limit;
    float vertical_pitch_thresh_mrad;
    float vertical_rate_thresh_mrads;

    float imu_pitch_zero_offset_rad;
    float balance_target_pitch_rad;

    float target_trim_limit_rad;
    float target_trim_rate_rad_s;
    float target_trim_err_gate_rad;
    float target_trim_rate_gate_rads;
    float target_trim_u_gate;

    float catch2bal_pitch_th_rad;
    float catch2bal_rate_th_rads;
    float bal2catch_pitch_th_rad;
    float catch_hold_ms;
    float catch_u_limit;
    float catch_k_pitch;
    float catch_k_pitch_rate;
    float catch_k_wheel_vel;
    float catch_k_wheel_pos;
    float catch_drive_u;

    float fall_pitch_pos_th_rad;
    float fall_pitch_neg_th_rad;

    float motion_pitch_bias_per_cmd_rad;
    float motion_cmd_rate_per_s;
    float motion_turn_u_limit;
} balance_params_t;

typedef struct
{
    float motion_fwd_cmd;
    float motion_turn_cmd;
} balance_command_t;

typedef struct
{
    float pitch_rad;
    float pitch_rate_rad_s;
    float wheel_vel_avg;
    float wheel_pos_avg;
    float wheel_vel_diff;
    uint8_t valid;
    uint8_t control_enabled;
    uint16_t reserved0;
    uint32_t now_ms;
} balance_input_t;

typedef struct
{
    balance_mode_t mode;
    uint32_t mode_enter_ms;
    uint32_t catch_in_band_since_ms;
    uint32_t last_step_ms;
    float wheel_pos_ref;
    float motion_fwd_cmd_cur;
    float motion_turn_cmd_cur;
    float target_trim_rad;
    float trim_u_avg;
    float trim_pitch_err_avg;
    uint8_t catch_in_band_active;
    uint8_t reserved1;
    uint16_t reserved2;
} balance_state_t;

typedef struct
{
    balance_mode_t mode;
    uint8_t valid;
    uint8_t saturated;
    uint16_t reserved0;

    float pitch_corr_rad;
    float pitch_error_rad;
    float target_pitch_rad;
    float wheel_pos_rel;

    float p_term;
    float d_term;
    float v_term;
    float x_term;
    float sync_term;
    float turn_term;

    float u_raw;
    float u_base;
    float u_left;
    float u_right;

    float motion_fwd_cmd;
    float motion_turn_cmd;
} balance_output_t;

typedef struct
{
    balance_params_t params;
    balance_state_t state;
} balance_controller_t;

#define BALANCE_CONTROL_U_LIMIT_DEFAULT             0.40f
#define BALANCE_CONTROL_K_PITCH_DEFAULT             1.00f
#define BALANCE_CONTROL_K_PITCH_RATE_DEFAULT        0.60f
#define BALANCE_CONTROL_K_WHEEL_VEL_DEFAULT         0.20f
#define BALANCE_CONTROL_K_WHEEL_POS_DEFAULT         0.06f
#define BALANCE_CONTROL_K_SYNC_DEFAULT              0.06f
#define BALANCE_CONTROL_U_SYNC_LIMIT_DEFAULT        0.03f
#define BALANCE_VERTICAL_PITCH_THRESH_MRAD_DEFAULT  80.0f
#define BALANCE_VERTICAL_RATE_THRESH_MRADS_DEFAULT  200.0f
#define BALANCE_IMU_PITCH_ZERO_OFFSET_RAD_DEFAULT   0.00f
#define BALANCE_TARGET_PITCH_RAD_DEFAULT            0.20f
#define BALANCE_TARGET_TRIM_LIMIT_RAD_DEFAULT       0.060f
#define BALANCE_TARGET_TRIM_RATE_RAD_S_DEFAULT      0.050f
#define BALANCE_TARGET_TRIM_ERR_GATE_RAD_DEFAULT    0.150f
#define BALANCE_TARGET_TRIM_RATE_GATE_RADS_DEFAULT  0.500f
#define BALANCE_TARGET_TRIM_U_GATE_DEFAULT          80.0f
#define BALANCE_CATCH2BAL_PITCH_TH_RAD_DEFAULT      0.05f
#define BALANCE_CATCH2BAL_RATE_TH_RADS_DEFAULT      0.50f
#define BALANCE_BAL2CATCH_PITCH_TH_RAD_DEFAULT      0.16f
#define BALANCE_CATCH_HOLD_MS_DEFAULT               300.0f
#define BALANCE_CATCH_U_LIMIT_DEFAULT               0.45f
#define BALANCE_CATCH_K_PITCH_DEFAULT               0.60f
#define BALANCE_CATCH_K_PITCH_RATE_DEFAULT          0.70f
#define BALANCE_CATCH_K_WHEEL_VEL_DEFAULT           0.06f
#define BALANCE_CATCH_K_WHEEL_POS_DEFAULT           0.00f
#define BALANCE_CATCH_DRIVE_U_DEFAULT               0.45f
#define BALANCE_FALL_PITCH_POS_TH_RAD_DEFAULT       (750.0f / 1000.0f)
#define BALANCE_FALL_PITCH_NEG_TH_RAD_DEFAULT       (-1200.0f / 1000.0f)
#define BALANCE_MOTION_PITCH_BIAS_PER_CMD_DEFAULT   0.06f
#define BALANCE_MOTION_CMD_RATE_PER_S_DEFAULT       2.50f
#define BALANCE_MOTION_TURN_U_LIMIT_DEFAULT         0.08f
#define BALANCE_MOTION_CMD_LIMIT                     1.00f

balance_status_t balance_init(balance_controller_t *bal);
balance_status_t balance_reset(balance_controller_t *bal, float wheel_pos_ref, uint32_t now_ms);
balance_status_t balance_step(balance_controller_t *bal,
                              const balance_input_t *in,
                              const balance_command_t *cmd,
                              balance_output_t *out);
const char *balance_mode_to_string(balance_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif
