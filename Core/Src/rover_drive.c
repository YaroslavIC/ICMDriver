#include "rover_drive.h"
#include <string.h>
#include <math.h>

static float rover_absf(float v)
{
    return (v >= 0.0f) ? v : -v;
}

static float rover_clampf(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

static float rover_drive_sanitize_dir(float dir)
{
    return (dir < 0.0f) ? -1.0f : 1.0f;
}

static float rover_drive_apply_ramp(float current, float target, float max_delta)
{
    float delta;

    if (max_delta <= 0.0f)
    {
        return target;
    }

    delta = target - current;
    if (delta > max_delta)
    {
        return current + max_delta;
    }
    if (delta < -max_delta)
    {
        return current - max_delta;
    }
    return target;
}

rover_drive_status_t rover_drive_set_defaults(rover_drive_params_t *params)
{
    if (params == NULL)
    {
        return ROVER_DRIVE_STATUS_BAD_ARG;
    }

    params->max_vel_rev_s = 1.0f;
    params->max_turn_rev_s = 1.0f;
    params->accel_rev_s2 = 2.0f;
    params->left_dir = 1.0f;
    params->right_dir = -1.0f;
    params->cmd_timeout_ms = 1000.0f;

    return ROVER_DRIVE_STATUS_OK;
}

rover_drive_status_t rover_drive_validate_params(const rover_drive_params_t *params)
{
    if (params == NULL)
    {
        return ROVER_DRIVE_STATUS_BAD_ARG;
    }

    if ((params->max_vel_rev_s <= 0.0f) || (params->max_turn_rev_s < 0.0f) ||
        (params->accel_rev_s2 < 0.0f) || (params->cmd_timeout_ms < 0.0f))
    {
        return ROVER_DRIVE_STATUS_BAD_VALUE;
    }

    return ROVER_DRIVE_STATUS_OK;
}

rover_drive_status_t rover_drive_init(rover_drive_t *drv)
{
    if (drv == NULL)
    {
        return ROVER_DRIVE_STATUS_BAD_ARG;
    }

    memset(drv, 0, sizeof(*drv));
    (void)rover_drive_set_defaults(&drv->params);
    drv->initialized = 1u;
    return ROVER_DRIVE_STATUS_OK;
}

rover_drive_status_t rover_drive_set_command(rover_drive_command_t *cmd, float forward, float turn, uint32_t now_ms)
{
    return rover_drive_set_command_boost(cmd,
                                         forward,
                                         turn,
                                         ROVER_DRIVE_DEFAULT_BOOST_VEL_REV_S,
                                         ROVER_DRIVE_DEFAULT_BOOST_MS,
                                         now_ms);
}

rover_drive_status_t rover_drive_set_command_boost(rover_drive_command_t *cmd,
                                                   float forward,
                                                   float turn,
                                                   float boost_vel_rev_s,
                                                   float boost_ms,
                                                   uint32_t now_ms)
{
    if (cmd == NULL)
    {
        return ROVER_DRIVE_STATUS_BAD_ARG;
    }

    if ((forward < -ROVER_DRIVE_CMD_LIMIT) || (forward > ROVER_DRIVE_CMD_LIMIT) ||
        (turn < -ROVER_DRIVE_CMD_LIMIT) || (turn > ROVER_DRIVE_CMD_LIMIT) ||
        (boost_vel_rev_s < 0.0f) || (boost_vel_rev_s > ROVER_DRIVE_MAX_BOOST_VEL_REV_S) ||
        (boost_ms < 0.0f) || (boost_ms > ROVER_DRIVE_MAX_BOOST_MS))
    {
        return ROVER_DRIVE_STATUS_BAD_VALUE;
    }

    cmd->forward_cmd = forward;
    cmd->turn_cmd = turn;
    cmd->boost_vel_rev_s = boost_vel_rev_s;
    cmd->boost_ms = boost_ms;
    cmd->last_cmd_ms = now_ms;
    return ROVER_DRIVE_STATUS_OK;
}

void rover_drive_stop(rover_drive_command_t *cmd, uint32_t now_ms)
{
    if (cmd == NULL)
    {
        return;
    }

    cmd->forward_cmd = 0.0f;
    cmd->turn_cmd = 0.0f;
    cmd->boost_vel_rev_s = ROVER_DRIVE_DEFAULT_BOOST_VEL_REV_S;
    cmd->boost_ms = ROVER_DRIVE_DEFAULT_BOOST_MS;
    cmd->last_cmd_ms = now_ms;
}

void rover_drive_reset_outputs(rover_drive_t *drv, uint32_t now_ms)
{
    if (drv == NULL)
    {
        return;
    }

    memset(&drv->out, 0, sizeof(drv->out));
    drv->out.mode = ROVER_DRIVE_MODE_IDLE;
    drv->last_step_ms = now_ms;
}

rover_drive_status_t rover_drive_step(rover_drive_t *drv,
                                      uint8_t enabled,
                                      uint8_t fault,
                                      const rover_drive_command_t *cmd,
                                      uint32_t now_ms,
                                      rover_drive_output_t *out)
{
    float forward;
    float turn;
    float left_target;
    float right_target;
    float max_abs;
    float max_vel;
    float max_delta;
    float boost_vel;
    float boost_ms;
    float dt_s;
    uint32_t dt_ms;
    uint8_t timeout;
    uint8_t boost_active;

    if ((drv == NULL) || (cmd == NULL))
    {
        return ROVER_DRIVE_STATUS_BAD_ARG;
    }

    if (rover_drive_validate_params(&drv->params) != ROVER_DRIVE_STATUS_OK)
    {
        return ROVER_DRIVE_STATUS_BAD_VALUE;
    }

    if (drv->initialized == 0u)
    {
        (void)rover_drive_init(drv);
    }

    dt_ms = (drv->last_step_ms == 0u) ? 0u : (uint32_t)(now_ms - drv->last_step_ms);
    drv->last_step_ms = now_ms;
    dt_s = ((float)dt_ms) * 0.001f;
    max_delta = drv->params.accel_rev_s2 * dt_s;

    timeout = 0u;
    drv->out.saturated = 0u;
    if ((drv->params.cmd_timeout_ms > 0.0f) &&
        ((uint32_t)(now_ms - cmd->last_cmd_ms) > (uint32_t)drv->params.cmd_timeout_ms))
    {
        timeout = 1u;
    }

    forward = rover_clampf(cmd->forward_cmd, -ROVER_DRIVE_CMD_LIMIT, ROVER_DRIVE_CMD_LIMIT);
    turn = rover_clampf(cmd->turn_cmd, -ROVER_DRIVE_CMD_LIMIT, ROVER_DRIVE_CMD_LIMIT);
    boost_vel = rover_clampf(cmd->boost_vel_rev_s, 0.0f, ROVER_DRIVE_MAX_BOOST_VEL_REV_S);
    boost_ms = rover_clampf(cmd->boost_ms, 0.0f, ROVER_DRIVE_MAX_BOOST_MS);
    boost_active = 0u;

    if ((enabled == 0u) || (fault != 0u) || (timeout != 0u))
    {
        left_target = 0.0f;
        right_target = 0.0f;
    }
    else
    {
        left_target = (forward * drv->params.max_vel_rev_s) - (turn * drv->params.max_turn_rev_s);
        right_target = (forward * drv->params.max_vel_rev_s) + (turn * drv->params.max_turn_rev_s);

        max_abs = rover_absf(left_target);
        if (rover_absf(right_target) > max_abs)
        {
            max_abs = rover_absf(right_target);
        }

        max_vel = drv->params.max_vel_rev_s;
        if (max_abs > max_vel)
        {
            left_target = (left_target / max_abs) * max_vel;
            right_target = (right_target / max_abs) * max_vel;
            drv->out.saturated = 1u;
        }
        else
        {
            drv->out.saturated = 0u;
        }

        left_target *= rover_drive_sanitize_dir(drv->params.left_dir);
        right_target *= rover_drive_sanitize_dir(drv->params.right_dir);

        if ((boost_vel > drv->params.max_vel_rev_s) && (drv->params.max_vel_rev_s > 0.0f))
        {
            boost_vel = drv->params.max_vel_rev_s;
        }

        if ((boost_vel > 0.0f) && (boost_ms > 0.0f) &&
            ((uint32_t)(now_ms - cmd->last_cmd_ms) <= (uint32_t)boost_ms) &&
            ((rover_absf(left_target) > 0.0001f) || (rover_absf(right_target) > 0.0001f)))
        {
            boost_active = 1u;
            if ((rover_absf(left_target) > 0.0001f) && (rover_absf(left_target) < boost_vel))
            {
                left_target = (left_target < 0.0f) ? -boost_vel : boost_vel;
            }
            if ((rover_absf(right_target) > 0.0001f) && (rover_absf(right_target) < boost_vel))
            {
                right_target = (right_target < 0.0f) ? -boost_vel : boost_vel;
            }
        }
    }

    drv->out.left_target_rev_s = left_target;
    drv->out.right_target_rev_s = right_target;
    drv->out.left_output_rev_s = rover_drive_apply_ramp(drv->out.left_output_rev_s, left_target, max_delta);
    drv->out.right_output_rev_s = rover_drive_apply_ramp(drv->out.right_output_rev_s, right_target, max_delta);
    drv->out.forward_cmd = forward;
    drv->out.turn_cmd = turn;
    drv->out.boost_vel_rev_s = boost_vel;
    drv->out.boost_ms = boost_ms;
    drv->out.boost_active = boost_active;
    drv->out.enabled = (enabled != 0u) ? 1u : 0u;
    drv->out.timeout = timeout;
    drv->out.valid = 1u;

    if (fault != 0u)
    {
        drv->out.mode = ROVER_DRIVE_MODE_FAULT;
    }
    else if (enabled == 0u)
    {
        drv->out.mode = ROVER_DRIVE_MODE_IDLE;
    }
    else if ((rover_absf(forward) < 0.0001f) && (rover_absf(turn) < 0.0001f))
    {
        drv->out.mode = ROVER_DRIVE_MODE_READY;
    }
    else
    {
        drv->out.mode = ROVER_DRIVE_MODE_DRIVE;
    }

    if (out != NULL)
    {
        *out = drv->out;
    }

    return ROVER_DRIVE_STATUS_OK;
}

const char *rover_drive_mode_to_string(rover_drive_mode_t mode)
{
    switch (mode)
    {
    case ROVER_DRIVE_MODE_IDLE: return "IDLE";
    case ROVER_DRIVE_MODE_READY: return "READY";
    case ROVER_DRIVE_MODE_DRIVE: return "DRIVE";
    case ROVER_DRIVE_MODE_FAULT: return "FAULT";
    default: return "UNKNOWN";
    }
}
