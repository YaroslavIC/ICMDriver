#include "hardwareinit.h"
#include <string.h>

static void hardwareinit_runtime_to_flash(const app_runtime_t *app, flash_cfg_data_t *data)
{
    if ((app == NULL) || (data == NULL))
    {
        return;
    }

    memset(data, 0, sizeof(*data));
    data->balance_params = app->balance.params;
    data->calibration = app->calib_data;
}

static void hardwareinit_runtime_from_flash(app_runtime_t *app, const flash_cfg_data_t *data)
{
    if ((app == NULL) || (data == NULL))
    {
        return;
    }

    app->balance.params = data->balance_params;
    app->calib_data = data->calibration;
    app->calib_loaded = ((app->calib_data.flags & BALANCE_CALIBRATION_FLAG_VALID) != 0u) ? 1u : 0u;
}

static uint8_t hardwareinit_param_is_persistent(app_serial_param_id_t id)
{
    switch (id)
    {
    case APP_SERIAL_PARAM_MOTION_FWD_CMD:
    case APP_SERIAL_PARAM_MOTION_TURN_CMD:
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

    if (balance_init(&app->balance) != BALANCE_STATUS_OK)
    {
        return HARDWAREINIT_STATUS_BALANCE_ERROR;
    }

    app->command.motion_fwd_cmd = 0.0f;
    app->command.motion_turn_cmd = 0.0f;
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
    case APP_SERIAL_PARAM_CONTROL_U_LIMIT: *value = app->balance.params.control_u_limit; break;
    case APP_SERIAL_PARAM_CONTROL_K_PITCH: *value = app->balance.params.control_k_pitch; break;
    case APP_SERIAL_PARAM_CONTROL_K_PITCH_RATE: *value = app->balance.params.control_k_pitch_rate; break;
    case APP_SERIAL_PARAM_CONTROL_K_WHEEL_VEL: *value = app->balance.params.control_k_wheel_vel; break;
    case APP_SERIAL_PARAM_CONTROL_K_WHEEL_POS: *value = app->balance.params.control_k_wheel_pos; break;
    case APP_SERIAL_PARAM_CONTROL_K_SYNC: *value = app->balance.params.control_k_sync; break;
    case APP_SERIAL_PARAM_CONTROL_U_SYNC_LIMIT: *value = app->balance.params.control_u_sync_limit; break;
    case APP_SERIAL_PARAM_VERTICAL_PITCH_THRESH_MRAD: *value = app->balance.params.vertical_pitch_thresh_mrad; break;
    case APP_SERIAL_PARAM_VERTICAL_RATE_THRESH_MRADS: *value = app->balance.params.vertical_rate_thresh_mrads; break;
    case APP_SERIAL_PARAM_IMU_PITCH_ZERO_OFFSET_RAD: *value = app->balance.params.imu_pitch_zero_offset_rad; break;
    case APP_SERIAL_PARAM_BALANCE_TARGET_PITCH_RAD: *value = app->balance.params.balance_target_pitch_rad; break;
    case APP_SERIAL_PARAM_CATCH2BAL_PITCH_TH_RAD: *value = app->balance.params.catch2bal_pitch_th_rad; break;
    case APP_SERIAL_PARAM_CATCH2BAL_RATE_TH_RADS: *value = app->balance.params.catch2bal_rate_th_rads; break;
    case APP_SERIAL_PARAM_BAL2CATCH_PITCH_TH_RAD: *value = app->balance.params.bal2catch_pitch_th_rad; break;
    case APP_SERIAL_PARAM_CATCH_HOLD_MS: *value = app->balance.params.catch_hold_ms; break;
    case APP_SERIAL_PARAM_CATCH_U_LIMIT: *value = app->balance.params.catch_u_limit; break;
    case APP_SERIAL_PARAM_CATCH_K_PITCH: *value = app->balance.params.catch_k_pitch; break;
    case APP_SERIAL_PARAM_CATCH_K_PITCH_RATE: *value = app->balance.params.catch_k_pitch_rate; break;
    case APP_SERIAL_PARAM_CATCH_K_WHEEL_VEL: *value = app->balance.params.catch_k_wheel_vel; break;
    case APP_SERIAL_PARAM_CATCH_K_WHEEL_POS: *value = app->balance.params.catch_k_wheel_pos; break;
    case APP_SERIAL_PARAM_FALL_PITCH_POS_TH_RAD: *value = app->balance.params.fall_pitch_pos_th_rad; break;
    case APP_SERIAL_PARAM_FALL_PITCH_NEG_TH_RAD: *value = app->balance.params.fall_pitch_neg_th_rad; break;
    case APP_SERIAL_PARAM_MOTION_PITCH_BIAS_PER_CMD_RAD: *value = app->balance.params.motion_pitch_bias_per_cmd_rad; break;
    case APP_SERIAL_PARAM_MOTION_CMD_RATE_PER_S: *value = app->balance.params.motion_cmd_rate_per_s; break;
    case APP_SERIAL_PARAM_MOTION_TURN_U_LIMIT: *value = app->balance.params.motion_turn_u_limit; break;
    case APP_SERIAL_PARAM_MOTION_FWD_CMD: *value = app->command.motion_fwd_cmd; break;
    case APP_SERIAL_PARAM_MOTION_TURN_CMD: *value = app->command.motion_turn_cmd; break;
    default:
        return APP_SERIAL_STATUS_BAD_PARAM;
    }

    return APP_SERIAL_STATUS_OK;
}

app_serial_status_t hardwareinit_serial_set_param(void *ctx, app_serial_param_id_t id, float value)
{
    app_runtime_t *app;
    balance_params_t prev_params;
    balance_command_t prev_cmd;

    if (ctx == NULL)
    {
        return APP_SERIAL_STATUS_BAD_ARG;
    }

    app = (app_runtime_t *)ctx;
    prev_params = app->balance.params;
    prev_cmd = app->command;

    switch (id)
    {
    case APP_SERIAL_PARAM_CONTROL_U_LIMIT:
        if (value <= 0.0f) { return APP_SERIAL_STATUS_BAD_VALUE; }
        app->balance.params.control_u_limit = value;
        break;
    case APP_SERIAL_PARAM_CONTROL_K_PITCH: app->balance.params.control_k_pitch = value; break;
    case APP_SERIAL_PARAM_CONTROL_K_PITCH_RATE: app->balance.params.control_k_pitch_rate = value; break;
    case APP_SERIAL_PARAM_CONTROL_K_WHEEL_VEL: app->balance.params.control_k_wheel_vel = value; break;
    case APP_SERIAL_PARAM_CONTROL_K_WHEEL_POS: app->balance.params.control_k_wheel_pos = value; break;
    case APP_SERIAL_PARAM_CONTROL_K_SYNC: app->balance.params.control_k_sync = value; break;
    case APP_SERIAL_PARAM_CONTROL_U_SYNC_LIMIT:
        if (value < 0.0f) { return APP_SERIAL_STATUS_BAD_VALUE; }
        app->balance.params.control_u_sync_limit = value;
        break;
    case APP_SERIAL_PARAM_VERTICAL_PITCH_THRESH_MRAD:
        if (value < 0.0f) { return APP_SERIAL_STATUS_BAD_VALUE; }
        app->balance.params.vertical_pitch_thresh_mrad = value;
        break;
    case APP_SERIAL_PARAM_VERTICAL_RATE_THRESH_MRADS:
        if (value < 0.0f) { return APP_SERIAL_STATUS_BAD_VALUE; }
        app->balance.params.vertical_rate_thresh_mrads = value;
        break;
    case APP_SERIAL_PARAM_IMU_PITCH_ZERO_OFFSET_RAD: app->balance.params.imu_pitch_zero_offset_rad = value; break;
    case APP_SERIAL_PARAM_BALANCE_TARGET_PITCH_RAD: app->balance.params.balance_target_pitch_rad = value; break;
    case APP_SERIAL_PARAM_CATCH2BAL_PITCH_TH_RAD:
        if (value < 0.0f) { return APP_SERIAL_STATUS_BAD_VALUE; }
        app->balance.params.catch2bal_pitch_th_rad = value;
        break;
    case APP_SERIAL_PARAM_CATCH2BAL_RATE_TH_RADS:
        if (value < 0.0f) { return APP_SERIAL_STATUS_BAD_VALUE; }
        app->balance.params.catch2bal_rate_th_rads = value;
        break;
    case APP_SERIAL_PARAM_BAL2CATCH_PITCH_TH_RAD:
        if (value < 0.0f) { return APP_SERIAL_STATUS_BAD_VALUE; }
        app->balance.params.bal2catch_pitch_th_rad = value;
        break;
    case APP_SERIAL_PARAM_CATCH_HOLD_MS:
        if (value < 0.0f) { return APP_SERIAL_STATUS_BAD_VALUE; }
        app->balance.params.catch_hold_ms = value;
        break;
    case APP_SERIAL_PARAM_CATCH_U_LIMIT:
        if (value <= 0.0f) { return APP_SERIAL_STATUS_BAD_VALUE; }
        app->balance.params.catch_u_limit = value;
        break;
    case APP_SERIAL_PARAM_CATCH_K_PITCH: app->balance.params.catch_k_pitch = value; break;
    case APP_SERIAL_PARAM_CATCH_K_PITCH_RATE: app->balance.params.catch_k_pitch_rate = value; break;
    case APP_SERIAL_PARAM_CATCH_K_WHEEL_VEL: app->balance.params.catch_k_wheel_vel = value; break;
    case APP_SERIAL_PARAM_CATCH_K_WHEEL_POS: app->balance.params.catch_k_wheel_pos = value; break;
    case APP_SERIAL_PARAM_FALL_PITCH_POS_TH_RAD: app->balance.params.fall_pitch_pos_th_rad = value; break;
    case APP_SERIAL_PARAM_FALL_PITCH_NEG_TH_RAD: app->balance.params.fall_pitch_neg_th_rad = value; break;
    case APP_SERIAL_PARAM_MOTION_PITCH_BIAS_PER_CMD_RAD: app->balance.params.motion_pitch_bias_per_cmd_rad = value; break;
    case APP_SERIAL_PARAM_MOTION_CMD_RATE_PER_S:
        if (value < 0.0f) { return APP_SERIAL_STATUS_BAD_VALUE; }
        app->balance.params.motion_cmd_rate_per_s = value;
        break;
    case APP_SERIAL_PARAM_MOTION_TURN_U_LIMIT:
        if (value < 0.0f) { return APP_SERIAL_STATUS_BAD_VALUE; }
        app->balance.params.motion_turn_u_limit = value;
        break;
    case APP_SERIAL_PARAM_MOTION_FWD_CMD:
        if ((value < -BALANCE_MOTION_CMD_LIMIT) || (value > BALANCE_MOTION_CMD_LIMIT))
        {
            return APP_SERIAL_STATUS_BAD_VALUE;
        }
        app->command.motion_fwd_cmd = value;
        break;
    case APP_SERIAL_PARAM_MOTION_TURN_CMD:
        if ((value < -BALANCE_MOTION_CMD_LIMIT) || (value > BALANCE_MOTION_CMD_LIMIT))
        {
            return APP_SERIAL_STATUS_BAD_VALUE;
        }
        app->command.motion_turn_cmd = value;
        break;
    default:
        return APP_SERIAL_STATUS_BAD_PARAM;
    }

    if (hardwareinit_param_is_persistent(id) != 0u)
    {
        if (hardwareinit_save_to_flash(app) != HARDWAREINIT_STATUS_OK)
        {
            app->balance.params = prev_params;
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
        app->command.motion_fwd_cmd = 0.0f;
        app->command.motion_turn_cmd = 0.0f;
    }
    return APP_SERIAL_STATUS_OK;
}
