#ifndef BALANCE_CALIBRATION_H
#define BALANCE_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

// Версия: 01.00 31.03.26 19:35:00
// Назначение:
// Полностью автоматическая калибровка deadzone, breakaway и грубой
// характеристики момент->скорость для левого и правого колеса.
// Модуль не знает деталей ODrive/USB/Flash и работает только через
// callbacks, переданные в структуре io.
//
// Пример инициализации:
// balance_calibration_t cal;
// memset(&cal, 0, sizeof(cal));
// cal.io.user_ctx = &app;
// cal.io.get_left_vel = app_get_left_vel;
// cal.io.get_right_vel = app_get_right_vel;
// cal.io.set_torque = app_set_pair_torque;
// cal.io.are_axes_ready = app_are_axes_ready;
// cal.io.has_errors = app_has_errors;
// cal.io.is_control_enabled = app_is_control_enabled;
// cal.io.set_control_enabled = app_set_control_enabled;
// cal.io.log_line = app_log_line;
// cal.io.save_persist = app_save_calibration;
// balance_calibration_init(&cal, NULL);
//
// Пример вызова в main loop:
// balance_calibration_process(&cal, HAL_GetTick());
// if (balance_calibration_is_active(&cal) != 0u)
// {
//     // обычный балансный регулятор не вызывать
// }

#define BALANCE_CALIBRATION_VERSION_U32                 0x00010000u
#define BALANCE_CALIBRATION_FLAG_VALID                  (1ul << 0)
#define BALANCE_CALIBRATION_FLAG_L_POS_VALID            (1ul << 1)
#define BALANCE_CALIBRATION_FLAG_L_NEG_VALID            (1ul << 2)
#define BALANCE_CALIBRATION_FLAG_R_POS_VALID            (1ul << 3)
#define BALANCE_CALIBRATION_FLAG_R_NEG_VALID            (1ul << 4)

#define BALANCE_CALIBRATION_U_START_SEARCH_DEFAULT      0.010f
#define BALANCE_CALIBRATION_U_STEP_SEARCH_DEFAULT       0.005f
#define BALANCE_CALIBRATION_U_MAX_SEARCH_DEFAULT        0.120f
#define BALANCE_CALIBRATION_CURVE_MARGIN_U_DEFAULT      0.010f
#define BALANCE_CALIBRATION_CURVE_STEP_U_DEFAULT        0.010f
#define BALANCE_CALIBRATION_CURVE_POINTS_DEFAULT        6u
#define BALANCE_CALIBRATION_SETTLE_MS_BEFORE_DEFAULT    500u
#define BALANCE_CALIBRATION_SETTLE_MS_BETWEEN_DEFAULT   300u
#define BALANCE_CALIBRATION_HOLD_MS_SEARCH_DEFAULT      400u
#define BALANCE_CALIBRATION_HOLD_MS_CURVE_DEFAULT       500u
#define BALANCE_CALIBRATION_AVG_WINDOW_MS_DEFAULT       150u
#define BALANCE_CALIBRATION_VEL_THRESH_MOVE_DEFAULT     0.10f
#define BALANCE_CALIBRATION_CONFIRM_MS_MOVE_DEFAULT     80u
#define BALANCE_CALIBRATION_VEL_STILL_THRESH_DEFAULT    0.05f
#define BALANCE_CALIBRATION_STILL_CONFIRM_MS_DEFAULT    200u
#define BALANCE_CALIBRATION_MAX_CURVE_POINTS            16u

typedef enum
{
    BALANCE_CALIBRATION_STATUS_OK = 0,
    BALANCE_CALIBRATION_STATUS_BAD_ARG,
    BALANCE_CALIBRATION_STATUS_BUSY,
    BALANCE_CALIBRATION_STATUS_ERROR,
    BALANCE_CALIBRATION_STATUS_FLASH_ERROR,
    BALANCE_CALIBRATION_STATUS_PRECHECK_FAIL,
    BALANCE_CALIBRATION_STATUS_ABORTED
} balance_calibration_status_t;

typedef enum
{
    BALANCE_CALIBRATION_STATE_IDLE = 0,
    BALANCE_CALIBRATION_STATE_PRECHECK,
    BALANCE_CALIBRATION_STATE_SETTLE,
    BALANCE_CALIBRATION_STATE_L_POS_SEARCH,
    BALANCE_CALIBRATION_STATE_L_NEG_SEARCH,
    BALANCE_CALIBRATION_STATE_R_POS_SEARCH,
    BALANCE_CALIBRATION_STATE_R_NEG_SEARCH,
    BALANCE_CALIBRATION_STATE_L_POS_CURVE,
    BALANCE_CALIBRATION_STATE_L_NEG_CURVE,
    BALANCE_CALIBRATION_STATE_R_POS_CURVE,
    BALANCE_CALIBRATION_STATE_R_NEG_CURVE,
    BALANCE_CALIBRATION_STATE_CALC,
    BALANCE_CALIBRATION_STATE_FLASH_SAVE,
    BALANCE_CALIBRATION_STATE_DONE,
    BALANCE_CALIBRATION_STATE_ABORT,
    BALANCE_CALIBRATION_STATE_ERROR
} balance_calibration_state_t;

typedef enum
{
    BALANCE_CALIBRATION_WHEEL_LEFT = 0,
    BALANCE_CALIBRATION_WHEEL_RIGHT
} balance_calibration_wheel_t;

typedef enum
{
    BALANCE_CALIBRATION_DIR_POS = 0,
    BALANCE_CALIBRATION_DIR_NEG
} balance_calibration_dir_t;

typedef struct
{
    float u_cmd;
    float vel_end;
    float vel_avg;
    uint8_t moved;
    int32_t t_move_ms;
    uint32_t hold_ms;
} balance_calibration_point_t;

typedef struct
{
    float deadzone;
    float breakaway;
    int32_t t_move_ms;
    uint32_t points_count;
    balance_calibration_point_t points[BALANCE_CALIBRATION_MAX_CURVE_POINTS];
} balance_calibration_dir_result_t;

typedef struct
{
    balance_calibration_dir_result_t pos;
    balance_calibration_dir_result_t neg;
} balance_calibration_wheel_result_t;

typedef struct
{
    uint32_t version;
    uint32_t flags;
    float l_deadzone_pos;
    float l_deadzone_neg;
    float r_deadzone_pos;
    float r_deadzone_neg;
    float l_breakaway_pos;
    float l_breakaway_neg;
    float r_breakaway_pos;
    float r_breakaway_neg;
    float l_slope_pos;
    float l_slope_neg;
    float r_slope_pos;
    float r_slope_neg;
    uint32_t timestamp_ms;
} balance_calibration_persist_t;

typedef struct
{
    float u_start_search;
    float u_step_search;
    float u_max_search;
    float curve_margin_u;
    float curve_step_u;
    uint32_t curve_points_count;
    uint32_t settle_ms_before;
    uint32_t settle_ms_between;
    uint32_t hold_ms_search;
    uint32_t hold_ms_curve;
    uint32_t avg_window_ms;
    float vel_thresh_move;
    uint32_t confirm_ms_move;
    float vel_still_thresh;
    uint32_t still_confirm_ms;
} balance_calibration_config_t;

typedef void (*balance_calibration_log_line_fn)(void *ctx, const char *line);
typedef float (*balance_calibration_get_vel_fn)(void *ctx);
typedef balance_calibration_status_t (*balance_calibration_set_torque_fn)(void *ctx, float u_left, float u_right);
typedef uint8_t (*balance_calibration_get_flag_fn)(void *ctx);
typedef balance_calibration_status_t (*balance_calibration_set_flag_fn)(void *ctx, uint8_t value);
typedef balance_calibration_status_t (*balance_calibration_save_persist_fn)(void *ctx, const balance_calibration_persist_t *persist);

typedef struct
{
    void *user_ctx;
    balance_calibration_get_vel_fn get_left_vel;
    balance_calibration_get_vel_fn get_right_vel;
    balance_calibration_set_torque_fn set_torque;
    balance_calibration_get_flag_fn are_axes_ready;
    balance_calibration_get_flag_fn has_errors;
    balance_calibration_get_flag_fn is_control_enabled;
    balance_calibration_set_flag_fn set_control_enabled;
    balance_calibration_log_line_fn log_line;
    balance_calibration_save_persist_fn save_persist;
} balance_calibration_io_t;

typedef struct
{
    balance_calibration_wheel_t wheel;
    balance_calibration_dir_t dir;
    float u_current;
    uint32_t point_start_ms;
    uint32_t avg_window_start_ms;
    uint32_t move_detect_start_ms;
    uint32_t still_detect_start_ms;
    uint32_t next_state_after_settle;
    uint32_t curve_index;
    uint32_t step_index;
    float vel_avg_sum;
    uint32_t vel_avg_count;
    uint8_t moved_latched;
    int32_t t_move_ms;
} balance_calibration_runtime_t;

typedef struct
{
    balance_calibration_config_t cfg;
    balance_calibration_io_t io;
    balance_calibration_state_t state;
    uint32_t state_enter_ms;
    uint8_t active;
    uint8_t abort_requested;
    uint8_t flash_saved;
    uint8_t reserved0;
    balance_calibration_runtime_t rt;
    balance_calibration_wheel_result_t left;
    balance_calibration_wheel_result_t right;
    balance_calibration_persist_t persist;
} balance_calibration_t;

void balance_calibration_init(balance_calibration_t *cal, const balance_calibration_config_t *cfg);
balance_calibration_status_t balance_calibration_start(balance_calibration_t *cal, uint32_t now_ms);
void balance_calibration_abort(balance_calibration_t *cal);
void balance_calibration_process(balance_calibration_t *cal, uint32_t now_ms);
uint8_t balance_calibration_is_active(const balance_calibration_t *cal);
balance_calibration_state_t balance_calibration_get_state(const balance_calibration_t *cal);
const balance_calibration_persist_t *balance_calibration_get_persist(const balance_calibration_t *cal);
void balance_calibration_set_persist(balance_calibration_t *cal, const balance_calibration_persist_t *persist);
void balance_calibration_emit_persist(const balance_calibration_t *cal, const balance_calibration_persist_t *persist);
const char *balance_calibration_state_to_phase(balance_calibration_state_t state);

#ifdef __cplusplus
}
#endif

#endif
