#include "balance_calibration.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>

static void balance_calibration_logf(const balance_calibration_t *cal, const char *fmt, ...)
{
    char line[196];
    va_list args;
    int n;

    if ((cal == NULL) || (cal->io.log_line == NULL) || (fmt == NULL))
    {
        return;
    }

    va_start(args, fmt);
    n = vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    if ((n <= 0) || (n >= (int)sizeof(line)))
    {
        return;
    }

    cal->io.log_line(cal->io.user_ctx, line);
}

const char *balance_calibration_state_to_phase(balance_calibration_state_t state)
{
    switch (state)
    {
    case BALANCE_CALIBRATION_STATE_PRECHECK: return "PRECHECK";
    case BALANCE_CALIBRATION_STATE_SETTLE: return "SETTLE";
    case BALANCE_CALIBRATION_STATE_L_POS_SEARCH: return "L_POS_SEARCH";
    case BALANCE_CALIBRATION_STATE_L_NEG_SEARCH: return "L_NEG_SEARCH";
    case BALANCE_CALIBRATION_STATE_R_POS_SEARCH: return "R_POS_SEARCH";
    case BALANCE_CALIBRATION_STATE_R_NEG_SEARCH: return "R_NEG_SEARCH";
    case BALANCE_CALIBRATION_STATE_L_POS_CURVE: return "L_POS_CURVE";
    case BALANCE_CALIBRATION_STATE_L_NEG_CURVE: return "L_NEG_CURVE";
    case BALANCE_CALIBRATION_STATE_R_POS_CURVE: return "R_POS_CURVE";
    case BALANCE_CALIBRATION_STATE_R_NEG_CURVE: return "R_NEG_CURVE";
    case BALANCE_CALIBRATION_STATE_CALC: return "CALC";
    case BALANCE_CALIBRATION_STATE_FLASH_SAVE: return "FLASH_SAVE";
    case BALANCE_CALIBRATION_STATE_DONE: return "DONE";
    case BALANCE_CALIBRATION_STATE_ABORT: return "ABORT";
    case BALANCE_CALIBRATION_STATE_ERROR: return "ERROR";
    default: return "IDLE";
    }
}

static void balance_calibration_cfg_default(balance_calibration_config_t *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    cfg->u_start_search = BALANCE_CALIBRATION_U_START_SEARCH_DEFAULT;
    cfg->u_step_search = BALANCE_CALIBRATION_U_STEP_SEARCH_DEFAULT;
    cfg->u_max_search = BALANCE_CALIBRATION_U_MAX_SEARCH_DEFAULT;
    cfg->curve_margin_u = BALANCE_CALIBRATION_CURVE_MARGIN_U_DEFAULT;
    cfg->curve_step_u = BALANCE_CALIBRATION_CURVE_STEP_U_DEFAULT;
    cfg->curve_points_count = BALANCE_CALIBRATION_CURVE_POINTS_DEFAULT;
    cfg->settle_ms_before = BALANCE_CALIBRATION_SETTLE_MS_BEFORE_DEFAULT;
    cfg->settle_ms_between = BALANCE_CALIBRATION_SETTLE_MS_BETWEEN_DEFAULT;
    cfg->hold_ms_search = BALANCE_CALIBRATION_HOLD_MS_SEARCH_DEFAULT;
    cfg->hold_ms_curve = BALANCE_CALIBRATION_HOLD_MS_CURVE_DEFAULT;
    cfg->avg_window_ms = BALANCE_CALIBRATION_AVG_WINDOW_MS_DEFAULT;
    cfg->vel_thresh_move = BALANCE_CALIBRATION_VEL_THRESH_MOVE_DEFAULT;
    cfg->confirm_ms_move = BALANCE_CALIBRATION_CONFIRM_MS_MOVE_DEFAULT;
    cfg->vel_still_thresh = BALANCE_CALIBRATION_VEL_STILL_THRESH_DEFAULT;
    cfg->still_confirm_ms = BALANCE_CALIBRATION_STILL_CONFIRM_MS_DEFAULT;
}

static balance_calibration_dir_result_t *balance_calibration_dir_result_ref(balance_calibration_t *cal,
                                                                            balance_calibration_wheel_t wheel,
                                                                            balance_calibration_dir_t dir)
{
    if (cal == NULL)
    {
        return NULL;
    }

    if (wheel == BALANCE_CALIBRATION_WHEEL_LEFT)
    {
        return (dir == BALANCE_CALIBRATION_DIR_POS) ? &cal->left.pos : &cal->left.neg;
    }

    return (dir == BALANCE_CALIBRATION_DIR_POS) ? &cal->right.pos : &cal->right.neg;
}

static float balance_calibration_get_active_vel(balance_calibration_t *cal)
{
    if (cal == NULL)
    {
        return 0.0f;
    }

    if (cal->rt.wheel == BALANCE_CALIBRATION_WHEEL_LEFT)
    {
        if (cal->io.get_left_vel == NULL)
        {
            return 0.0f;
        }
        return cal->io.get_left_vel(cal->io.user_ctx);
    }

    if (cal->io.get_right_vel == NULL)
    {
        return 0.0f;
    }
    return cal->io.get_right_vel(cal->io.user_ctx);
}

static void balance_calibration_apply_torque(balance_calibration_t *cal, float u_cmd)
{
    float u_left;
    float u_right;

    if ((cal == NULL) || (cal->io.set_torque == NULL))
    {
        return;
    }

    u_left = 0.0f;
    u_right = 0.0f;
    if (cal->rt.wheel == BALANCE_CALIBRATION_WHEEL_LEFT)
    {
        u_left = u_cmd;
    }
    else
    {
        u_right = u_cmd;
    }

    (void)cal->io.set_torque(cal->io.user_ctx, u_left, u_right);
}

static void balance_calibration_apply_zero(balance_calibration_t *cal)
{
    if ((cal == NULL) || (cal->io.set_torque == NULL))
    {
        return;
    }

    (void)cal->io.set_torque(cal->io.user_ctx, 0.0f, 0.0f);
}

static void balance_calibration_prepare_step(balance_calibration_t *cal, uint32_t now_ms)
{
    float breakaway;
    float sign;
    balance_calibration_dir_result_t *dir_res;

    if (cal == NULL)
    {
        return;
    }

    cal->rt.point_start_ms = now_ms;
    cal->rt.avg_window_start_ms = now_ms;
    cal->rt.move_detect_start_ms = 0u;
    cal->rt.vel_avg_sum = 0.0f;
    cal->rt.vel_avg_count = 0u;
    cal->rt.moved_latched = 0u;
    cal->rt.t_move_ms = -1;
    cal->rt.step_index++;

    if ((cal->state == BALANCE_CALIBRATION_STATE_L_POS_SEARCH) ||
        (cal->state == BALANCE_CALIBRATION_STATE_L_NEG_SEARCH) ||
        (cal->state == BALANCE_CALIBRATION_STATE_R_POS_SEARCH) ||
        (cal->state == BALANCE_CALIBRATION_STATE_R_NEG_SEARCH))
    {
        if (fabsf(cal->rt.u_current) < 1e-9f)
        {
            sign = (cal->rt.dir == BALANCE_CALIBRATION_DIR_POS) ? 1.0f : -1.0f;
            cal->rt.u_current = sign * cal->cfg.u_start_search;
        }
    }
    else
    {
        dir_res = balance_calibration_dir_result_ref(cal, cal->rt.wheel, cal->rt.dir);
        if (dir_res != NULL)
        {
            breakaway = dir_res->breakaway;
            sign = (cal->rt.dir == BALANCE_CALIBRATION_DIR_POS) ? 1.0f : -1.0f;
            cal->rt.u_current = sign * (breakaway + cal->cfg.curve_margin_u + ((float)cal->rt.curve_index * cal->cfg.curve_step_u));
        }
    }

    balance_calibration_logf(cal,
                             "CALSTAT phase=%s step=%lu state=RUN",
                             balance_calibration_state_to_phase(cal->state),
                             (unsigned long)cal->rt.step_index);
}

static void balance_calibration_set_state(balance_calibration_t *cal,
                                          balance_calibration_state_t new_state,
                                          uint32_t now_ms)
{
    if (cal == NULL)
    {
        return;
    }

    cal->state = new_state;
    cal->state_enter_ms = now_ms;
    cal->rt.step_index = 0u;
    cal->rt.move_detect_start_ms = 0u;
    cal->rt.still_detect_start_ms = 0u;
    cal->rt.vel_avg_sum = 0.0f;
    cal->rt.vel_avg_count = 0u;
    cal->rt.moved_latched = 0u;
    cal->rt.t_move_ms = -1;

    if (new_state == BALANCE_CALIBRATION_STATE_SETTLE)
    {
        balance_calibration_apply_zero(cal);
        balance_calibration_logf(cal, "CALSTAT phase=SETTLE state=RUN");
        return;
    }

    if (new_state == BALANCE_CALIBRATION_STATE_DONE)
    {
        balance_calibration_apply_zero(cal);
        balance_calibration_logf(cal, "CALSTAT phase=DONE state=OK");
        cal->active = 0u;
        return;
    }

    if (new_state == BALANCE_CALIBRATION_STATE_ABORT)
    {
        balance_calibration_apply_zero(cal);
        balance_calibration_logf(cal, "CALERR code=ABORTED reason=user_request");
        balance_calibration_logf(cal, "CALSTAT phase=ABORT state=ABORTED");
        cal->active = 0u;
        return;
    }

    if (new_state == BALANCE_CALIBRATION_STATE_ERROR)
    {
        balance_calibration_apply_zero(cal);
        balance_calibration_logf(cal, "CALSTAT phase=ERROR state=FAIL");
        cal->active = 0u;
        return;
    }

    if (new_state == BALANCE_CALIBRATION_STATE_PRECHECK)
    {
        balance_calibration_logf(cal, "CALSTAT phase=PRECHECK state=RUN");
        return;
    }

    if (new_state == BALANCE_CALIBRATION_STATE_CALC)
    {
        balance_calibration_apply_zero(cal);
        balance_calibration_logf(cal, "CALSTAT phase=CALC state=RUN");
        return;
    }

    if (new_state == BALANCE_CALIBRATION_STATE_FLASH_SAVE)
    {
        balance_calibration_apply_zero(cal);
        balance_calibration_logf(cal, "CALSTAT phase=FLASH_SAVE state=RUN");
        return;
    }

    if (new_state == BALANCE_CALIBRATION_STATE_L_POS_SEARCH)
    {
        cal->rt.wheel = BALANCE_CALIBRATION_WHEEL_LEFT;
        cal->rt.dir = BALANCE_CALIBRATION_DIR_POS;
        cal->rt.u_current = 0.0f;
        balance_calibration_prepare_step(cal, now_ms);
        return;
    }

    if (new_state == BALANCE_CALIBRATION_STATE_L_NEG_SEARCH)
    {
        cal->rt.wheel = BALANCE_CALIBRATION_WHEEL_LEFT;
        cal->rt.dir = BALANCE_CALIBRATION_DIR_NEG;
        cal->rt.u_current = 0.0f;
        balance_calibration_prepare_step(cal, now_ms);
        return;
    }

    if (new_state == BALANCE_CALIBRATION_STATE_R_POS_SEARCH)
    {
        cal->rt.wheel = BALANCE_CALIBRATION_WHEEL_RIGHT;
        cal->rt.dir = BALANCE_CALIBRATION_DIR_POS;
        cal->rt.u_current = 0.0f;
        balance_calibration_prepare_step(cal, now_ms);
        return;
    }

    if (new_state == BALANCE_CALIBRATION_STATE_R_NEG_SEARCH)
    {
        cal->rt.wheel = BALANCE_CALIBRATION_WHEEL_RIGHT;
        cal->rt.dir = BALANCE_CALIBRATION_DIR_NEG;
        cal->rt.u_current = 0.0f;
        balance_calibration_prepare_step(cal, now_ms);
        return;
    }

    if (new_state == BALANCE_CALIBRATION_STATE_L_POS_CURVE)
    {
        cal->rt.wheel = BALANCE_CALIBRATION_WHEEL_LEFT;
        cal->rt.dir = BALANCE_CALIBRATION_DIR_POS;
        cal->rt.curve_index = 0u;
        balance_calibration_prepare_step(cal, now_ms);
        return;
    }

    if (new_state == BALANCE_CALIBRATION_STATE_L_NEG_CURVE)
    {
        cal->rt.wheel = BALANCE_CALIBRATION_WHEEL_LEFT;
        cal->rt.dir = BALANCE_CALIBRATION_DIR_NEG;
        cal->rt.curve_index = 0u;
        balance_calibration_prepare_step(cal, now_ms);
        return;
    }

    if (new_state == BALANCE_CALIBRATION_STATE_R_POS_CURVE)
    {
        cal->rt.wheel = BALANCE_CALIBRATION_WHEEL_RIGHT;
        cal->rt.dir = BALANCE_CALIBRATION_DIR_POS;
        cal->rt.curve_index = 0u;
        balance_calibration_prepare_step(cal, now_ms);
        return;
    }

    if (new_state == BALANCE_CALIBRATION_STATE_R_NEG_CURVE)
    {
        cal->rt.wheel = BALANCE_CALIBRATION_WHEEL_RIGHT;
        cal->rt.dir = BALANCE_CALIBRATION_DIR_NEG;
        cal->rt.curve_index = 0u;
        balance_calibration_prepare_step(cal, now_ms);
        return;
    }
}

static void balance_calibration_finish_dir_search(balance_calibration_t *cal, uint32_t now_ms)
{
    balance_calibration_dir_result_t *dir_res;
    float breakaway;
    float deadzone;

    dir_res = balance_calibration_dir_result_ref(cal, cal->rt.wheel, cal->rt.dir);
    if (dir_res == NULL)
    {
        balance_calibration_logf(cal, "CALERR code=SEARCH_FAIL reason=bad_dir_ref");
        balance_calibration_set_state(cal, BALANCE_CALIBRATION_STATE_ERROR, now_ms);
        return;
    }

    breakaway = fabsf(cal->rt.u_current);
    deadzone = breakaway - (0.5f * cal->cfg.u_step_search);
    if (deadzone < 0.0f)
    {
        deadzone = 0.0f;
    }

    dir_res->breakaway = breakaway;
    dir_res->deadzone = deadzone;
    dir_res->t_move_ms = cal->rt.t_move_ms;

    balance_calibration_logf(cal,
                             "CALDZ wheel=%c dir=%s deadzone=%.4f breakaway=%.4f t_move_ms=%ld",
                             (cal->rt.wheel == BALANCE_CALIBRATION_WHEEL_LEFT) ? 'L' : 'R',
                             (cal->rt.dir == BALANCE_CALIBRATION_DIR_POS) ? "POS" : "NEG",
                             deadzone,
                             breakaway,
                             (long)cal->rt.t_move_ms);

    if (cal->state == BALANCE_CALIBRATION_STATE_L_POS_SEARCH)
    {
        cal->rt.next_state_after_settle = (uint32_t)BALANCE_CALIBRATION_STATE_L_NEG_SEARCH;
    }
    else if (cal->state == BALANCE_CALIBRATION_STATE_L_NEG_SEARCH)
    {
        cal->rt.next_state_after_settle = (uint32_t)BALANCE_CALIBRATION_STATE_R_POS_SEARCH;
    }
    else if (cal->state == BALANCE_CALIBRATION_STATE_R_POS_SEARCH)
    {
        cal->rt.next_state_after_settle = (uint32_t)BALANCE_CALIBRATION_STATE_R_NEG_SEARCH;
    }
    else
    {
        cal->rt.next_state_after_settle = (uint32_t)BALANCE_CALIBRATION_STATE_L_POS_CURVE;
    }

    balance_calibration_set_state(cal, BALANCE_CALIBRATION_STATE_SETTLE, now_ms);
}

static void balance_calibration_finish_dir_curve(balance_calibration_t *cal, uint32_t now_ms)
{
    if (cal->state == BALANCE_CALIBRATION_STATE_L_POS_CURVE)
    {
        cal->rt.next_state_after_settle = (uint32_t)BALANCE_CALIBRATION_STATE_L_NEG_CURVE;
    }
    else if (cal->state == BALANCE_CALIBRATION_STATE_L_NEG_CURVE)
    {
        cal->rt.next_state_after_settle = (uint32_t)BALANCE_CALIBRATION_STATE_R_POS_CURVE;
    }
    else if (cal->state == BALANCE_CALIBRATION_STATE_R_POS_CURVE)
    {
        cal->rt.next_state_after_settle = (uint32_t)BALANCE_CALIBRATION_STATE_R_NEG_CURVE;
    }
    else
    {
        cal->rt.next_state_after_settle = (uint32_t)BALANCE_CALIBRATION_STATE_CALC;
    }

    balance_calibration_set_state(cal, BALANCE_CALIBRATION_STATE_SETTLE, now_ms);
}

static void balance_calibration_calc_slopes(balance_calibration_t *cal)
{
    const balance_calibration_point_t *first_pt;
    const balance_calibration_point_t *last_pt;
    float du;

    if (cal == NULL)
    {
        return;
    }

    cal->persist.version = BALANCE_CALIBRATION_VERSION_U32;
    cal->persist.flags = 0u;
    cal->persist.l_deadzone_pos = cal->left.pos.deadzone;
    cal->persist.l_deadzone_neg = cal->left.neg.deadzone;
    cal->persist.r_deadzone_pos = cal->right.pos.deadzone;
    cal->persist.r_deadzone_neg = cal->right.neg.deadzone;
    cal->persist.l_breakaway_pos = cal->left.pos.breakaway;
    cal->persist.l_breakaway_neg = cal->left.neg.breakaway;
    cal->persist.r_breakaway_pos = cal->right.pos.breakaway;
    cal->persist.r_breakaway_neg = cal->right.neg.breakaway;

    cal->persist.l_slope_pos = 0.0f;
    cal->persist.l_slope_neg = 0.0f;
    cal->persist.r_slope_pos = 0.0f;
    cal->persist.r_slope_neg = 0.0f;

    if (cal->left.pos.points_count >= 2u)
    {
        first_pt = &cal->left.pos.points[0];
        last_pt = &cal->left.pos.points[cal->left.pos.points_count - 1u];
        du = fabsf(last_pt->u_cmd) - fabsf(first_pt->u_cmd);
        if (fabsf(du) > 1e-6f)
        {
            cal->persist.l_slope_pos = (last_pt->vel_avg - first_pt->vel_avg) / du;
            cal->persist.flags |= BALANCE_CALIBRATION_FLAG_L_POS_VALID;
        }
    }

    if (cal->left.neg.points_count >= 2u)
    {
        first_pt = &cal->left.neg.points[0];
        last_pt = &cal->left.neg.points[cal->left.neg.points_count - 1u];
        du = fabsf(last_pt->u_cmd) - fabsf(first_pt->u_cmd);
        if (fabsf(du) > 1e-6f)
        {
            cal->persist.l_slope_neg = (last_pt->vel_avg - first_pt->vel_avg) / du;
            cal->persist.flags |= BALANCE_CALIBRATION_FLAG_L_NEG_VALID;
        }
    }

    if (cal->right.pos.points_count >= 2u)
    {
        first_pt = &cal->right.pos.points[0];
        last_pt = &cal->right.pos.points[cal->right.pos.points_count - 1u];
        du = fabsf(last_pt->u_cmd) - fabsf(first_pt->u_cmd);
        if (fabsf(du) > 1e-6f)
        {
            cal->persist.r_slope_pos = (last_pt->vel_avg - first_pt->vel_avg) / du;
            cal->persist.flags |= BALANCE_CALIBRATION_FLAG_R_POS_VALID;
        }
    }

    if (cal->right.neg.points_count >= 2u)
    {
        first_pt = &cal->right.neg.points[0];
        last_pt = &cal->right.neg.points[cal->right.neg.points_count - 1u];
        du = fabsf(last_pt->u_cmd) - fabsf(first_pt->u_cmd);
        if (fabsf(du) > 1e-6f)
        {
            cal->persist.r_slope_neg = (last_pt->vel_avg - first_pt->vel_avg) / du;
            cal->persist.flags |= BALANCE_CALIBRATION_FLAG_R_NEG_VALID;
        }
    }

    if ((cal->persist.flags & (BALANCE_CALIBRATION_FLAG_L_POS_VALID |
                               BALANCE_CALIBRATION_FLAG_L_NEG_VALID |
                               BALANCE_CALIBRATION_FLAG_R_POS_VALID |
                               BALANCE_CALIBRATION_FLAG_R_NEG_VALID)) != 0u)
    {
        cal->persist.flags |= BALANCE_CALIBRATION_FLAG_VALID;
    }
}

void balance_calibration_emit_persist(const balance_calibration_t *cal, const balance_calibration_persist_t *persist)
{
    if ((cal == NULL) || (persist == NULL))
    {
        return;
    }

    balance_calibration_logf(cal,
                             "CALRES1 l_deadzone_pos=%.4f l_deadzone_neg=%.4f r_deadzone_pos=%.4f r_deadzone_neg=%.4f",
                             persist->l_deadzone_pos,
                             persist->l_deadzone_neg,
                             persist->r_deadzone_pos,
                             persist->r_deadzone_neg);
    balance_calibration_logf(cal,
                             "CALRES2 l_breakaway_pos=%.4f l_breakaway_neg=%.4f r_breakaway_pos=%.4f r_breakaway_neg=%.4f",
                             persist->l_breakaway_pos,
                             persist->l_breakaway_neg,
                             persist->r_breakaway_pos,
                             persist->r_breakaway_neg);
    balance_calibration_logf(cal,
                             "CALRES3 l_slope_pos=%.4f l_slope_neg=%.4f r_slope_pos=%.4f r_slope_neg=%.4f flash=%s",
                             persist->l_slope_pos,
                             persist->l_slope_neg,
                             persist->r_slope_pos,
                             persist->r_slope_neg,
                             (cal->flash_saved != 0u) ? "OK" : "NA");
}

void balance_calibration_init(balance_calibration_t *cal, const balance_calibration_config_t *cfg)
{
    if (cal == NULL)
    {
        return;
    }

    memset(cal, 0, sizeof(*cal));
    balance_calibration_cfg_default(&cal->cfg);
    if (cfg != NULL)
    {
        cal->cfg = *cfg;
    }
    if (cal->cfg.curve_points_count > BALANCE_CALIBRATION_MAX_CURVE_POINTS)
    {
        cal->cfg.curve_points_count = BALANCE_CALIBRATION_MAX_CURVE_POINTS;
    }
    cal->state = BALANCE_CALIBRATION_STATE_IDLE;
    cal->persist.version = BALANCE_CALIBRATION_VERSION_U32;
}

balance_calibration_status_t balance_calibration_start(balance_calibration_t *cal, uint32_t now_ms)
{
    if (cal == NULL)
    {
        return BALANCE_CALIBRATION_STATUS_BAD_ARG;
    }

    if (cal->active != 0u)
    {
        return BALANCE_CALIBRATION_STATUS_BUSY;
    }

    memset(&cal->left, 0, sizeof(cal->left));
    memset(&cal->right, 0, sizeof(cal->right));
    memset(&cal->persist, 0, sizeof(cal->persist));
    memset(&cal->rt, 0, sizeof(cal->rt));
    cal->persist.version = BALANCE_CALIBRATION_VERSION_U32;
    cal->flash_saved = 0u;
    cal->abort_requested = 0u;
    cal->active = 1u;
    balance_calibration_set_state(cal, BALANCE_CALIBRATION_STATE_PRECHECK, now_ms);
    return BALANCE_CALIBRATION_STATUS_OK;
}

void balance_calibration_abort(balance_calibration_t *cal)
{
    if (cal == NULL)
    {
        return;
    }

    cal->abort_requested = 1u;
}

uint8_t balance_calibration_is_active(const balance_calibration_t *cal)
{
    if (cal == NULL)
    {
        return 0u;
    }

    return cal->active;
}

balance_calibration_state_t balance_calibration_get_state(const balance_calibration_t *cal)
{
    if (cal == NULL)
    {
        return BALANCE_CALIBRATION_STATE_IDLE;
    }

    return cal->state;
}

const balance_calibration_persist_t *balance_calibration_get_persist(const balance_calibration_t *cal)
{
    if (cal == NULL)
    {
        return NULL;
    }

    return &cal->persist;
}

void balance_calibration_set_persist(balance_calibration_t *cal, const balance_calibration_persist_t *persist)
{
    if ((cal == NULL) || (persist == NULL))
    {
        return;
    }

    cal->persist = *persist;
}

void balance_calibration_process(balance_calibration_t *cal, uint32_t now_ms)
{
    float vel_cur;
    float vel_abs;
    float vel_avg;
    uint32_t elapsed_ms;
    balance_calibration_dir_result_t *dir_res;
    balance_calibration_point_t *pt;
    const char *reason;

    if ((cal == NULL) || (cal->active == 0u))
    {
        return;
    }

    if (cal->abort_requested != 0u)
    {
        balance_calibration_set_state(cal, BALANCE_CALIBRATION_STATE_ABORT, now_ms);
        return;
    }

    if (cal->state == BALANCE_CALIBRATION_STATE_PRECHECK)
    {
        if ((cal->io.get_left_vel == NULL) || (cal->io.get_right_vel == NULL) ||
            (cal->io.set_torque == NULL) || (cal->io.are_axes_ready == NULL) ||
            (cal->io.has_errors == NULL) || (cal->io.is_control_enabled == NULL) ||
            (cal->io.set_control_enabled == NULL) || (cal->io.save_persist == NULL))
        {
            balance_calibration_logf(cal, "CALERR code=PRECHECK_FAIL reason=missing_callback");
            balance_calibration_set_state(cal, BALANCE_CALIBRATION_STATE_ERROR, now_ms);
            return;
        }
        if (cal->io.has_errors(cal->io.user_ctx) != 0u)
        {
            balance_calibration_logf(cal, "CALERR code=PRECHECK_FAIL reason=odrive_error");
            balance_calibration_set_state(cal, BALANCE_CALIBRATION_STATE_ERROR, now_ms);
            return;
        }
        if (cal->io.are_axes_ready(cal->io.user_ctx) == 0u)
        {
            balance_calibration_logf(cal, "CALERR code=PRECHECK_FAIL reason=axes_not_ready");
            balance_calibration_set_state(cal, BALANCE_CALIBRATION_STATE_ERROR, now_ms);
            return;
        }
        if (cal->io.set_control_enabled(cal->io.user_ctx, 0u) != BALANCE_CALIBRATION_STATUS_OK)
        {
            balance_calibration_logf(cal, "CALERR code=PRECHECK_FAIL reason=disable_control_failed");
            balance_calibration_set_state(cal, BALANCE_CALIBRATION_STATE_ERROR, now_ms);
            return;
        }
        cal->rt.next_state_after_settle = (uint32_t)BALANCE_CALIBRATION_STATE_L_POS_SEARCH;
        balance_calibration_set_state(cal, BALANCE_CALIBRATION_STATE_SETTLE, now_ms);
        return;
    }

    if (cal->state == BALANCE_CALIBRATION_STATE_SETTLE)
    {
        balance_calibration_apply_zero(cal);
        vel_abs = fabsf(cal->io.get_left_vel(cal->io.user_ctx));
        if (fabsf(cal->io.get_right_vel(cal->io.user_ctx)) > vel_abs)
        {
            vel_abs = fabsf(cal->io.get_right_vel(cal->io.user_ctx));
        }
        if (vel_abs <= cal->cfg.vel_still_thresh)
        {
            if (cal->rt.still_detect_start_ms == 0u)
            {
                cal->rt.still_detect_start_ms = now_ms;
            }
            elapsed_ms = now_ms - cal->rt.still_detect_start_ms;
            if (elapsed_ms >= cal->cfg.still_confirm_ms)
            {
                balance_calibration_set_state(cal,
                                              (balance_calibration_state_t)cal->rt.next_state_after_settle,
                                              now_ms);
            }
        }
        else
        {
            cal->rt.still_detect_start_ms = 0u;
        }
        return;
    }

    if (cal->state == BALANCE_CALIBRATION_STATE_CALC)
    {
        cal->persist.timestamp_ms = now_ms;
        balance_calibration_calc_slopes(cal);
        balance_calibration_set_state(cal, BALANCE_CALIBRATION_STATE_FLASH_SAVE, now_ms);
        return;
    }

    if (cal->state == BALANCE_CALIBRATION_STATE_FLASH_SAVE)
    {
        if (cal->io.save_persist(cal->io.user_ctx, &cal->persist) != BALANCE_CALIBRATION_STATUS_OK)
        {
            cal->flash_saved = 0u;
            balance_calibration_logf(cal, "CALERR code=FLASH_WRITE_FAIL reason=save_failed");
            balance_calibration_set_state(cal, BALANCE_CALIBRATION_STATE_ERROR, now_ms);
            return;
        }
        cal->flash_saved = 1u;
        balance_calibration_emit_persist(cal, &cal->persist);
        balance_calibration_set_state(cal, BALANCE_CALIBRATION_STATE_DONE, now_ms);
        return;
    }

    if ((cal->state != BALANCE_CALIBRATION_STATE_L_POS_SEARCH) &&
        (cal->state != BALANCE_CALIBRATION_STATE_L_NEG_SEARCH) &&
        (cal->state != BALANCE_CALIBRATION_STATE_R_POS_SEARCH) &&
        (cal->state != BALANCE_CALIBRATION_STATE_R_NEG_SEARCH) &&
        (cal->state != BALANCE_CALIBRATION_STATE_L_POS_CURVE) &&
        (cal->state != BALANCE_CALIBRATION_STATE_L_NEG_CURVE) &&
        (cal->state != BALANCE_CALIBRATION_STATE_R_POS_CURVE) &&
        (cal->state != BALANCE_CALIBRATION_STATE_R_NEG_CURVE))
    {
        return;
    }

    balance_calibration_apply_torque(cal, cal->rt.u_current);

    vel_cur = balance_calibration_get_active_vel(cal);
    vel_abs = fabsf(vel_cur);
    if ((now_ms - cal->rt.point_start_ms) >= (cal->cfg.hold_ms_search > cal->cfg.avg_window_ms ? (cal->cfg.hold_ms_search - cal->cfg.avg_window_ms) : 0u))
    {
        cal->rt.vel_avg_sum += vel_cur;
        cal->rt.vel_avg_count++;
    }

    if (vel_abs >= cal->cfg.vel_thresh_move)
    {
        if (cal->rt.move_detect_start_ms == 0u)
        {
            cal->rt.move_detect_start_ms = now_ms;
        }
        else if (((now_ms - cal->rt.move_detect_start_ms) >= cal->cfg.confirm_ms_move) && (cal->rt.moved_latched == 0u))
        {
            cal->rt.moved_latched = 1u;
            cal->rt.t_move_ms = (int32_t)(now_ms - cal->rt.point_start_ms);
        }
    }
    else
    {
        cal->rt.move_detect_start_ms = 0u;
    }

    elapsed_ms = now_ms - cal->rt.point_start_ms;
    if ((cal->state == BALANCE_CALIBRATION_STATE_L_POS_SEARCH) ||
        (cal->state == BALANCE_CALIBRATION_STATE_L_NEG_SEARCH) ||
        (cal->state == BALANCE_CALIBRATION_STATE_R_POS_SEARCH) ||
        (cal->state == BALANCE_CALIBRATION_STATE_R_NEG_SEARCH))
    {
        if (elapsed_ms < cal->cfg.hold_ms_search)
        {
            return;
        }

        if (cal->rt.vel_avg_count != 0u)
        {
            vel_avg = cal->rt.vel_avg_sum / (float)cal->rt.vel_avg_count;
        }
        else
        {
            vel_avg = vel_cur;
        }

        balance_calibration_logf(cal,
                                 "CALPT wheel=%c dir=%s phase=SEARCH step=%lu u=%.4f vel=%.4f vel_avg=%.4f moved=%u t_move_ms=%ld hold_ms=%lu",
                                 (cal->rt.wheel == BALANCE_CALIBRATION_WHEEL_LEFT) ? 'L' : 'R',
                                 (cal->rt.dir == BALANCE_CALIBRATION_DIR_POS) ? "POS" : "NEG",
                                 (unsigned long)cal->rt.step_index,
                                 cal->rt.u_current,
                                 vel_cur,
                                 vel_avg,
                                 (unsigned)cal->rt.moved_latched,
                                 (long)cal->rt.t_move_ms,
                                 (unsigned long)cal->cfg.hold_ms_search);

        if (cal->rt.moved_latched != 0u)
        {
            balance_calibration_finish_dir_search(cal, now_ms);
            return;
        }

        if ((fabsf(cal->rt.u_current) + cal->cfg.u_step_search) > cal->cfg.u_max_search)
        {
            if (cal->state == BALANCE_CALIBRATION_STATE_L_POS_SEARCH) { reason = "l_pos_not_moved"; }
            else if (cal->state == BALANCE_CALIBRATION_STATE_L_NEG_SEARCH) { reason = "l_neg_not_moved"; }
            else if (cal->state == BALANCE_CALIBRATION_STATE_R_POS_SEARCH) { reason = "r_pos_not_moved"; }
            else { reason = "r_neg_not_moved"; }
            balance_calibration_logf(cal, "CALERR code=SEARCH_FAIL reason=%s", reason);
            balance_calibration_set_state(cal, BALANCE_CALIBRATION_STATE_ERROR, now_ms);
            return;
        }

        cal->rt.u_current += (cal->rt.dir == BALANCE_CALIBRATION_DIR_POS) ? cal->cfg.u_step_search : -cal->cfg.u_step_search;
        balance_calibration_prepare_step(cal, now_ms);
        return;
    }

    if (elapsed_ms < cal->cfg.hold_ms_curve)
    {
        return;
    }

    if (cal->rt.vel_avg_count != 0u)
    {
        vel_avg = cal->rt.vel_avg_sum / (float)cal->rt.vel_avg_count;
    }
    else
    {
        vel_avg = vel_cur;
    }

    dir_res = balance_calibration_dir_result_ref(cal, cal->rt.wheel, cal->rt.dir);
    if (dir_res == NULL)
    {
        balance_calibration_logf(cal, "CALERR code=CURVE_FAIL reason=bad_dir_ref");
        balance_calibration_set_state(cal, BALANCE_CALIBRATION_STATE_ERROR, now_ms);
        return;
    }
    if (dir_res->points_count >= BALANCE_CALIBRATION_MAX_CURVE_POINTS)
    {
        balance_calibration_logf(cal, "CALERR code=CURVE_FAIL reason=too_many_points");
        balance_calibration_set_state(cal, BALANCE_CALIBRATION_STATE_ERROR, now_ms);
        return;
    }

    pt = &dir_res->points[dir_res->points_count];
    pt->u_cmd = cal->rt.u_current;
    pt->vel_end = vel_cur;
    pt->vel_avg = vel_avg;
    pt->moved = cal->rt.moved_latched;
    pt->t_move_ms = cal->rt.t_move_ms;
    pt->hold_ms = cal->cfg.hold_ms_curve;
    dir_res->points_count++;

    balance_calibration_logf(cal,
                             "CALPT wheel=%c dir=%s phase=CURVE step=%lu u=%.4f vel=%.4f vel_avg=%.4f moved=%u t_move_ms=%ld hold_ms=%lu",
                             (cal->rt.wheel == BALANCE_CALIBRATION_WHEEL_LEFT) ? 'L' : 'R',
                             (cal->rt.dir == BALANCE_CALIBRATION_DIR_POS) ? "POS" : "NEG",
                             (unsigned long)cal->rt.step_index,
                             cal->rt.u_current,
                             vel_cur,
                             vel_avg,
                             (unsigned)cal->rt.moved_latched,
                             (long)cal->rt.t_move_ms,
                             (unsigned long)cal->cfg.hold_ms_curve);

    if ((cal->rt.curve_index + 1u) >= cal->cfg.curve_points_count)
    {
        balance_calibration_finish_dir_curve(cal, now_ms);
        return;
    }

    cal->rt.curve_index++;
    balance_calibration_prepare_step(cal, now_ms);
}
