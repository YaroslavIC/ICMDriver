#include "balance.h"
#include <string.h>
#include <math.h>

static float balance_clamp(float v, float vmin, float vmax)
{
    if (v < vmin)
    {
        return vmin;
    }
    if (v > vmax)
    {
        return vmax;
    }
    return v;
}

static float balance_rate_limit(float cur, float target, float step)
{
    float dv;

    dv = target - cur;
    if (dv > step)
    {
        dv = step;
    }
    else if (dv < -step)
    {
        dv = -step;
    }

    return cur + dv;
}

const char *balance_mode_to_string(balance_mode_t mode)
{
    switch (mode)
    {
    case BALANCE_MODE_IDLE:
        return "IDLE";

    case BALANCE_MODE_CATCH:
        return "CATCH";

    case BALANCE_MODE_BALANCE:
        return "BAL";

    case BALANCE_MODE_SAFE_STOP:
        return "SAFE";

    default:
        return "UNK";
    }
}

balance_status_t balance_init(balance_controller_t *bal)
{
    if (bal == NULL)
    {
        return BALANCE_STATUS_BAD_ARG;
    }

    memset(bal, 0, sizeof(*bal));

    bal->params.control_u_limit = BALANCE_CONTROL_U_LIMIT_DEFAULT;
    bal->params.control_k_pitch = BALANCE_CONTROL_K_PITCH_DEFAULT;
    bal->params.control_k_pitch_rate = BALANCE_CONTROL_K_PITCH_RATE_DEFAULT;
    bal->params.control_k_wheel_vel = BALANCE_CONTROL_K_WHEEL_VEL_DEFAULT;
    bal->params.control_k_wheel_pos = BALANCE_CONTROL_K_WHEEL_POS_DEFAULT;
    bal->params.control_k_sync = BALANCE_CONTROL_K_SYNC_DEFAULT;
    bal->params.control_u_sync_limit = BALANCE_CONTROL_U_SYNC_LIMIT_DEFAULT;
    bal->params.vertical_pitch_thresh_mrad = BALANCE_VERTICAL_PITCH_THRESH_MRAD_DEFAULT;
    bal->params.vertical_rate_thresh_mrads = BALANCE_VERTICAL_RATE_THRESH_MRADS_DEFAULT;
    bal->params.imu_pitch_zero_offset_rad = BALANCE_IMU_PITCH_ZERO_OFFSET_RAD_DEFAULT;
    bal->params.balance_target_pitch_rad = BALANCE_TARGET_PITCH_RAD_DEFAULT;
    bal->params.catch2bal_pitch_th_rad = BALANCE_CATCH2BAL_PITCH_TH_RAD_DEFAULT;
    bal->params.catch2bal_rate_th_rads = BALANCE_CATCH2BAL_RATE_TH_RADS_DEFAULT;
    bal->params.bal2catch_pitch_th_rad = BALANCE_BAL2CATCH_PITCH_TH_RAD_DEFAULT;
    bal->params.catch_hold_ms = BALANCE_CATCH_HOLD_MS_DEFAULT;
    bal->params.catch_u_limit = BALANCE_CATCH_U_LIMIT_DEFAULT;
    bal->params.catch_k_pitch = BALANCE_CATCH_K_PITCH_DEFAULT;
    bal->params.catch_k_pitch_rate = BALANCE_CATCH_K_PITCH_RATE_DEFAULT;
    bal->params.catch_k_wheel_vel = BALANCE_CATCH_K_WHEEL_VEL_DEFAULT;
    bal->params.catch_k_wheel_pos = BALANCE_CATCH_K_WHEEL_POS_DEFAULT;
    bal->params.fall_pitch_pos_th_rad = BALANCE_FALL_PITCH_POS_TH_RAD_DEFAULT;
    bal->params.fall_pitch_neg_th_rad = BALANCE_FALL_PITCH_NEG_TH_RAD_DEFAULT;
    bal->params.motion_pitch_bias_per_cmd_rad = BALANCE_MOTION_PITCH_BIAS_PER_CMD_DEFAULT;
    bal->params.motion_cmd_rate_per_s = BALANCE_MOTION_CMD_RATE_PER_S_DEFAULT;
    bal->params.motion_turn_u_limit = BALANCE_MOTION_TURN_U_LIMIT_DEFAULT;

    bal->state.mode = BALANCE_MODE_IDLE;
    return BALANCE_STATUS_OK;
}

balance_status_t balance_reset(balance_controller_t *bal, float wheel_pos_ref, uint32_t now_ms)
{
    if (bal == NULL)
    {
        return BALANCE_STATUS_BAD_ARG;
    }

    bal->state.mode = BALANCE_MODE_IDLE;
    bal->state.mode_enter_ms = now_ms;
    bal->state.catch_in_band_since_ms = 0u;
    bal->state.last_step_ms = now_ms;
    bal->state.wheel_pos_ref = wheel_pos_ref;
    bal->state.motion_fwd_cmd_cur = 0.0f;
    bal->state.motion_turn_cmd_cur = 0.0f;
    bal->state.catch_in_band_active = 0u;
    return BALANCE_STATUS_OK;
}

static void balance_set_mode(balance_controller_t *bal, balance_mode_t mode, uint32_t now_ms)
{
    if (bal->state.mode == mode)
    {
        return;
    }

    bal->state.mode = mode;
    bal->state.mode_enter_ms = now_ms;
    if (mode != BALANCE_MODE_CATCH)
    {
        bal->state.catch_in_band_since_ms = 0u;
        bal->state.catch_in_band_active = 0u;
    }
}

static void balance_update_mode(balance_controller_t *bal,
                                const balance_input_t *in,
                                float pitch_error,
                                float pitch_rate)
{
    uint8_t in_band;

    if ((in->control_enabled == 0u) || (in->valid == 0u))
    {
        balance_set_mode(bal, BALANCE_MODE_IDLE, in->now_ms);
        return;
    }

    switch (bal->state.mode)
    {
    case BALANCE_MODE_IDLE:
        balance_set_mode(bal, BALANCE_MODE_CATCH, in->now_ms);
        break;

    case BALANCE_MODE_CATCH:
        in_band = 0u;
        if ((fabsf(pitch_error) < bal->params.catch2bal_pitch_th_rad) &&
            (fabsf(pitch_rate) < bal->params.catch2bal_rate_th_rads))
        {
            in_band = 1u;
        }

        if (in_band != 0u)
        {
            if (bal->state.catch_in_band_active == 0u)
            {
                bal->state.catch_in_band_active = 1u;
                bal->state.catch_in_band_since_ms = in->now_ms;
            }

            if ((float)((uint32_t)(in->now_ms - bal->state.catch_in_band_since_ms)) >= bal->params.catch_hold_ms)
            {
                balance_set_mode(bal, BALANCE_MODE_BALANCE, in->now_ms);
            }
        }
        else
        {
            bal->state.catch_in_band_active = 0u;
            bal->state.catch_in_band_since_ms = 0u;
        }
        break;

    case BALANCE_MODE_BALANCE:
        if (fabsf(pitch_error) > bal->params.bal2catch_pitch_th_rad)
        {
            balance_set_mode(bal, BALANCE_MODE_CATCH, in->now_ms);
        }
        break;

    case BALANCE_MODE_SAFE_STOP:
    default:
        balance_set_mode(bal, BALANCE_MODE_IDLE, in->now_ms);
        break;
    }
}

balance_status_t balance_step(balance_controller_t *bal,
                              const balance_input_t *in,
                              const balance_command_t *cmd,
                              balance_output_t *out)
{
    float dt_s;
    float cmd_step;
    float fwd_cmd_target;
    float turn_cmd_target;
    float pitch_corr;
    float target_pitch;
    float pitch_error;
    float wheel_pos_rel;
    float p_term;
    float d_term;
    float v_term;
    float x_term;
    float u_raw;
    float u_base;
    float sync_term;
    float turn_term;
    float left_cmd;
    float right_cmd;

    if ((bal == NULL) || (in == NULL) || (cmd == NULL) || (out == NULL))
    {
        return BALANCE_STATUS_BAD_ARG;
    }

    memset(out, 0, sizeof(*out));

    if (bal->state.last_step_ms == 0u)
    {
        bal->state.last_step_ms = in->now_ms;
    }

    dt_s = (float)((uint32_t)(in->now_ms - bal->state.last_step_ms)) * 0.001f;
    if ((dt_s <= 0.0f) || (dt_s > 0.2f))
    {
        dt_s = 0.02f;
    }
    bal->state.last_step_ms = in->now_ms;

    fwd_cmd_target = balance_clamp(cmd->motion_fwd_cmd,
                                   -BALANCE_MOTION_CMD_LIMIT,
                                   +BALANCE_MOTION_CMD_LIMIT);
    turn_cmd_target = balance_clamp(cmd->motion_turn_cmd,
                                    -BALANCE_MOTION_CMD_LIMIT,
                                    +BALANCE_MOTION_CMD_LIMIT);
    cmd_step = bal->params.motion_cmd_rate_per_s * dt_s;
    if (cmd_step < 0.0f)
    {
        cmd_step = 0.0f;
    }

    bal->state.motion_fwd_cmd_cur = balance_rate_limit(bal->state.motion_fwd_cmd_cur,
                                                       fwd_cmd_target,
                                                       cmd_step);
    bal->state.motion_turn_cmd_cur = balance_rate_limit(bal->state.motion_turn_cmd_cur,
                                                        turn_cmd_target,
                                                        cmd_step);

    pitch_corr = in->pitch_rad + bal->params.imu_pitch_zero_offset_rad;
    target_pitch = bal->params.balance_target_pitch_rad +
                   (bal->params.motion_pitch_bias_per_cmd_rad * bal->state.motion_fwd_cmd_cur);
    pitch_error = pitch_corr - target_pitch;

    if ((pitch_corr > bal->params.fall_pitch_pos_th_rad) ||
        (pitch_corr < bal->params.fall_pitch_neg_th_rad) ||
        (in->control_enabled == 0u) ||
        (in->valid == 0u))
    {
        (void)balance_reset(bal, in->wheel_pos_avg, in->now_ms);
        out->mode = BALANCE_MODE_IDLE;
        out->pitch_corr_rad = pitch_corr;
        out->pitch_error_rad = pitch_error;
        out->target_pitch_rad = target_pitch;
        out->wheel_pos_rel = 0.0f;
        out->valid = 1u;
        return BALANCE_STATUS_OK;
    }

    balance_update_mode(bal, in, pitch_error, in->pitch_rate_rad_s);
    if (bal->state.mode == BALANCE_MODE_IDLE)
    {
        bal->state.wheel_pos_ref = in->wheel_pos_avg;
        out->mode = BALANCE_MODE_IDLE;
        out->pitch_corr_rad = pitch_corr;
        out->pitch_error_rad = pitch_error;
        out->target_pitch_rad = target_pitch;
        out->valid = 1u;
        return BALANCE_STATUS_OK;
    }

    wheel_pos_rel = in->wheel_pos_avg - bal->state.wheel_pos_ref;

    if (bal->state.mode == BALANCE_MODE_CATCH)
    {
        p_term = -bal->params.catch_k_pitch * pitch_error;
        d_term = bal->params.catch_k_pitch_rate * in->pitch_rate_rad_s;
        v_term = -(bal->params.catch_k_wheel_vel * in->wheel_vel_avg);
        x_term = -(bal->params.catch_k_wheel_pos * wheel_pos_rel);
        u_raw = p_term + d_term + v_term + x_term;
        u_base = balance_clamp(u_raw,
                               -bal->params.catch_u_limit,
                               +bal->params.catch_u_limit);

        sync_term = 0.0f;
        turn_term = 0.0f;
        left_cmd = -u_base;
        right_cmd = u_base;
    }
    else
    {
        p_term = -bal->params.control_k_pitch * pitch_error;
        d_term = bal->params.control_k_pitch_rate * in->pitch_rate_rad_s;
        v_term = -(bal->params.control_k_wheel_vel * in->wheel_vel_avg);
        x_term = -(bal->params.control_k_wheel_pos * wheel_pos_rel);
        u_raw = p_term + d_term + v_term + x_term;
        u_base = balance_clamp(u_raw,
                               -(bal->params.control_u_limit - bal->params.control_u_sync_limit),
                               +(bal->params.control_u_limit - bal->params.control_u_sync_limit));


        sync_term = -(bal->params.control_k_sync * in->wheel_vel_diff);
        sync_term = balance_clamp(sync_term,
                                  -bal->params.control_u_sync_limit,
                                  +bal->params.control_u_sync_limit);

        turn_term = bal->state.motion_turn_cmd_cur * bal->params.motion_turn_u_limit;
        turn_term = balance_clamp(turn_term,
                                  -bal->params.motion_turn_u_limit,
                                  +bal->params.motion_turn_u_limit);

        left_cmd = -u_base - sync_term + turn_term;
        right_cmd = +u_base + sync_term + turn_term;
        left_cmd = balance_clamp(left_cmd, -bal->params.control_u_limit, +bal->params.control_u_limit);
        right_cmd = balance_clamp(right_cmd, -bal->params.control_u_limit, +bal->params.control_u_limit);
    }

    out->mode = bal->state.mode;
    out->valid = 1u;
    out->pitch_corr_rad = pitch_corr;
    out->pitch_error_rad = pitch_error;
    out->target_pitch_rad = target_pitch;
    out->wheel_pos_rel = wheel_pos_rel;
    out->p_term = p_term;
    out->d_term = d_term;
    out->v_term = v_term;
    out->x_term = x_term;
    out->sync_term = sync_term;
    out->turn_term = turn_term;
    out->u_raw = u_raw;
    out->u_base = u_base;
    out->u_left = left_cmd;
    out->u_right = right_cmd;
    out->motion_fwd_cmd = bal->state.motion_fwd_cmd_cur;
    out->motion_turn_cmd = bal->state.motion_turn_cmd_cur;

    if ((fabsf(u_raw - u_base) > 1e-6f) ||
        (fabsf(left_cmd) >= (bal->params.control_u_limit - 1e-6f)) ||
        (fabsf(right_cmd) >= (bal->params.control_u_limit - 1e-6f)))
    {
        out->saturated = 1u;
    }

    return BALANCE_STATUS_OK;
}
