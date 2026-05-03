#include "hardwareinit.h"
#include <string.h>

static void hardwareinit_runtime_to_flash(const app_runtime_t *app, flash_cfg_data_t *data)
{
    if ((app == NULL) || (data == NULL))
    {
        return;
    }

    memset(data, 0, sizeof(*data));
    data->rover_params = app->drive.params;
}

static void hardwareinit_runtime_from_flash(app_runtime_t *app, const flash_cfg_data_t *data)
{
    if ((app == NULL) || (data == NULL))
    {
        return;
    }

    if (rover_drive_validate_params(&data->rover_params) == ROVER_DRIVE_STATUS_OK)
    {
        app->drive.params = data->rover_params;
    }
}

static uint8_t hardwareinit_param_is_persistent(app_serial_param_id_t id)
{
    switch (id)
    {
    case APP_SERIAL_PARAM_FORWARD_CMD:
    case APP_SERIAL_PARAM_TURN_CMD:
        return 0u;

    default:
        return 1u;
    }
}

hardwareinit_status_t hardwareinit_runtime_init(app_runtime_t *app)
{
    flash_cfg_data_t flash_data;

    if (app == NULL)
    {
        return HARDWAREINIT_STATUS_BAD_ARG;
    }

    memset(app, 0, sizeof(*app));

    if (rover_drive_init(&app->drive) != ROVER_DRIVE_STATUS_OK)
    {
        return HARDWAREINIT_STATUS_ROVER_ERROR;
    }

    rover_drive_stop(&app->command, 0u);
    app->control_enabled = 0u;

    if (flash_cfg_store_init(&app->flash_store) != FLASH_CFG_STORE_STATUS_OK)
    {
        return HARDWAREINIT_STATUS_FLASH_ERROR;
    }

    if (flash_cfg_store_load(&app->flash_store, &flash_data) == FLASH_CFG_STORE_STATUS_OK)
    {
        hardwareinit_runtime_from_flash(app, &flash_data);
        app->flash_cfg_loaded = 1u;
    }

    app->serial.get_param = hardwareinit_serial_get_param;
    app->serial.set_param = hardwareinit_serial_set_param;
    app->serial.get_enable = hardwareinit_serial_get_enable;
    app->serial.set_enable = hardwareinit_serial_set_enable;
    app->serial.user_ctx = app;
    if (app_serial_init(&app->serial) != APP_SERIAL_STATUS_OK)
    {
        return HARDWAREINIT_STATUS_SERIAL_ERROR;
    }

    return HARDWAREINIT_STATUS_OK;
}

hardwareinit_status_t hardwareinit_save_to_flash(app_runtime_t *app)
{
    flash_cfg_data_t data;

    if (app == NULL)
    {
        return HARDWAREINIT_STATUS_BAD_ARG;
    }

    hardwareinit_runtime_to_flash(app, &data);
    if (flash_cfg_store_save(&app->flash_store, &data) != FLASH_CFG_STORE_STATUS_OK)
    {
        return HARDWAREINIT_STATUS_FLASH_ERROR;
    }

    return HARDWAREINIT_STATUS_OK;
}

app_serial_status_t hardwareinit_serial_get_param(void *ctx, app_serial_param_id_t id, float *value)
{
    app_runtime_t *app;

    if ((ctx == NULL) || (value == NULL))
    {
        return APP_SERIAL_STATUS_BAD_ARG;
    }

    app = (app_runtime_t *)ctx;

    switch (id)
    {
    case APP_SERIAL_PARAM_MAX_VEL_REV_S: *value = app->drive.params.max_vel_rev_s; break;
    case APP_SERIAL_PARAM_MAX_TURN_REV_S: *value = app->drive.params.max_turn_rev_s; break;
    case APP_SERIAL_PARAM_ACCEL_REV_S2: *value = app->drive.params.accel_rev_s2; break;
    case APP_SERIAL_PARAM_LEFT_DIR: *value = app->drive.params.left_dir; break;
    case APP_SERIAL_PARAM_RIGHT_DIR: *value = app->drive.params.right_dir; break;
    case APP_SERIAL_PARAM_CMD_TIMEOUT_MS: *value = app->drive.params.cmd_timeout_ms; break;
    case APP_SERIAL_PARAM_FORWARD_CMD: *value = app->command.forward_cmd; break;
    case APP_SERIAL_PARAM_TURN_CMD: *value = app->command.turn_cmd; break;
    default:
        return APP_SERIAL_STATUS_BAD_PARAM;
    }

    return APP_SERIAL_STATUS_OK;
}

app_serial_status_t hardwareinit_serial_set_param(void *ctx, app_serial_param_id_t id, float value)
{
    app_runtime_t *app;
    rover_drive_params_t prev_params;
    rover_drive_command_t prev_cmd;

    if (ctx == NULL)
    {
        return APP_SERIAL_STATUS_BAD_ARG;
    }

    app = (app_runtime_t *)ctx;
    prev_params = app->drive.params;
    prev_cmd = app->command;

    switch (id)
    {
    case APP_SERIAL_PARAM_MAX_VEL_REV_S:
        if (value <= 0.0f) { return APP_SERIAL_STATUS_BAD_VALUE; }
        app->drive.params.max_vel_rev_s = value;
        break;

    case APP_SERIAL_PARAM_MAX_TURN_REV_S:
        if (value < 0.0f) { return APP_SERIAL_STATUS_BAD_VALUE; }
        app->drive.params.max_turn_rev_s = value;
        break;

    case APP_SERIAL_PARAM_ACCEL_REV_S2:
        if (value < 0.0f) { return APP_SERIAL_STATUS_BAD_VALUE; }
        app->drive.params.accel_rev_s2 = value;
        break;

    case APP_SERIAL_PARAM_LEFT_DIR:
        app->drive.params.left_dir = (value < 0.0f) ? -1.0f : 1.0f;
        break;

    case APP_SERIAL_PARAM_RIGHT_DIR:
        app->drive.params.right_dir = (value < 0.0f) ? -1.0f : 1.0f;
        break;

    case APP_SERIAL_PARAM_CMD_TIMEOUT_MS:
        if (value < 0.0f) { return APP_SERIAL_STATUS_BAD_VALUE; }
        app->drive.params.cmd_timeout_ms = value;
        break;

    case APP_SERIAL_PARAM_FORWARD_CMD:
        if (rover_drive_set_command(&app->command, value, app->command.turn_cmd, HAL_GetTick()) != ROVER_DRIVE_STATUS_OK)
        {
            return APP_SERIAL_STATUS_BAD_VALUE;
        }
        break;

    case APP_SERIAL_PARAM_TURN_CMD:
        if (rover_drive_set_command(&app->command, app->command.forward_cmd, value, HAL_GetTick()) != ROVER_DRIVE_STATUS_OK)
        {
            return APP_SERIAL_STATUS_BAD_VALUE;
        }
        break;

    default:
        return APP_SERIAL_STATUS_BAD_PARAM;
    }

    if (rover_drive_validate_params(&app->drive.params) != ROVER_DRIVE_STATUS_OK)
    {
        app->drive.params = prev_params;
        app->command = prev_cmd;
        return APP_SERIAL_STATUS_BAD_VALUE;
    }

    if (hardwareinit_param_is_persistent(id) != 0u)
    {
        if (hardwareinit_save_to_flash(app) != HARDWAREINIT_STATUS_OK)
        {
            app->drive.params = prev_params;
            app->command = prev_cmd;
            return APP_SERIAL_STATUS_ERROR;
        }
    }

    return APP_SERIAL_STATUS_OK;
}

app_serial_status_t hardwareinit_serial_get_enable(void *ctx, uint8_t *enabled)
{
    app_runtime_t *app;

    if ((ctx == NULL) || (enabled == NULL))
    {
        return APP_SERIAL_STATUS_BAD_ARG;
    }

    app = (app_runtime_t *)ctx;
    *enabled = app->control_enabled;
    return APP_SERIAL_STATUS_OK;
}

app_serial_status_t hardwareinit_serial_set_enable(void *ctx, uint8_t enabled)
{
    app_runtime_t *app;

    if (ctx == NULL)
    {
        return APP_SERIAL_STATUS_BAD_ARG;
    }

    app = (app_runtime_t *)ctx;
    app->control_enabled = (enabled != 0u) ? 1u : 0u;
    if (app->control_enabled == 0u)
    {
        rover_drive_stop(&app->command, HAL_GetTick());
        rover_drive_reset_outputs(&app->drive, HAL_GetTick());
    }
    return APP_SERIAL_STATUS_OK;
}
