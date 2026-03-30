#include "app_serial.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static app_serial_t *g_app_serial_target = NULL;

static const char *app_serial_param_name(app_serial_param_id_t id)
{
    static const char * const names[APP_SERIAL_PARAM_COUNT] =
    {
        "control_u_limit",
        "control_k_pitch",
        "control_k_pitch_rate",
        "control_k_wheel_vel",
        "control_k_wheel_pos",
        "control_k_sync",
        "control_u_sync_limit",
        "vertical_pitch_thresh_mrad",
        "vertical_rate_thresh_mrads",
        "imu_pitch_zero_offset_rad",
        "balance_target_pitch_rad",
        "catch2bal_pitch_th_rad",
        "catch2bal_rate_th_rads",
        "bal2catch_pitch_th_rad",
        "catch_hold_ms",
        "catch_u_limit",
        "catch_k_pitch",
        "catch_k_pitch_rate",
        "catch_k_wheel_vel",
        "catch_k_wheel_pos",
        "fall_pitch_pos_th_rad",
        "fall_pitch_neg_th_rad",
        "motion_pitch_bias_per_cmd_rad",
        "motion_cmd_rate_per_s",
        "motion_turn_u_limit",
        "motion_fwd_cmd",
        "motion_turn_cmd"
    };

    if ((uint32_t)id >= (uint32_t)APP_SERIAL_PARAM_COUNT)
    {
        return NULL;
    }

    return names[(uint32_t)id];
}

static app_serial_status_t app_serial_try_send(app_serial_t *serial, const char *text)
{
    uint16_t len;
    uint8_t tx_status;

    if ((serial == NULL) || (text == NULL))
    {
        return APP_SERIAL_STATUS_BAD_ARG;
    }

    len = (uint16_t)strlen(text);
    tx_status = CDC_Transmit_FS((uint8_t *)text, len);
    if (tx_status == USBD_OK)
    {
        return APP_SERIAL_STATUS_OK;
    }

    if (tx_status == USBD_BUSY)
    {
        return APP_SERIAL_STATUS_BUSY;
    }

    return APP_SERIAL_STATUS_ERROR;
}

static app_serial_status_t app_serial_send_param(app_serial_t *serial, app_serial_param_id_t id)
{
    float value;
    const char *name;
    int n;

    if ((serial == NULL) || (serial->get_param == NULL))
    {
        return APP_SERIAL_STATUS_BAD_ARG;
    }

    if (serial->get_param(serial->user_ctx, id, &value) != APP_SERIAL_STATUS_OK)
    {
        return APP_SERIAL_STATUS_BAD_PARAM;
    }

    name = app_serial_param_name(id);
    if (name == NULL)
    {
        return APP_SERIAL_STATUS_BAD_PARAM;
    }

    n = snprintf(serial->tx_line, sizeof(serial->tx_line), "RSP %s=%.6f\r\n", name, value);
    if ((n <= 0) || (n >= (int)sizeof(serial->tx_line)))
    {
        return APP_SERIAL_STATUS_ERROR;
    }

    return app_serial_try_send(serial, serial->tx_line);
}

static app_serial_status_t app_serial_send_enable(app_serial_t *serial)
{
    uint8_t enabled;
    int n;

    if ((serial == NULL) || (serial->get_enable == NULL))
    {
        return APP_SERIAL_STATUS_BAD_ARG;
    }

    if (serial->get_enable(serial->user_ctx, &enabled) != APP_SERIAL_STATUS_OK)
    {
        return APP_SERIAL_STATUS_ERROR;
    }

    n = snprintf(serial->tx_line, sizeof(serial->tx_line), "RSP en=%u\r\n", (unsigned)enabled);
    if ((n <= 0) || (n >= (int)sizeof(serial->tx_line)))
    {
        return APP_SERIAL_STATUS_ERROR;
    }

    return app_serial_try_send(serial, serial->tx_line);
}

static app_serial_param_id_t app_serial_find_param(const char *name)
{
    uint32_t i;
    const char *param_name;

    if (name == NULL)
    {
        return APP_SERIAL_PARAM_COUNT;
    }

    for (i = 0u; i < (uint32_t)APP_SERIAL_PARAM_COUNT; i++)
    {
        param_name = app_serial_param_name((app_serial_param_id_t)i);
        if ((param_name != NULL) && (strcmp(name, param_name) == 0))
        {
            return (app_serial_param_id_t)i;
        }
    }

    return APP_SERIAL_PARAM_COUNT;
}

static app_serial_status_t app_serial_handle_line(app_serial_t *serial)
{
    char cmd[16];
    char name[APP_SERIAL_TOKEN_NAME_SIZE];
    float value;
    unsigned en_value;
    int parsed;
    app_serial_param_id_t id;
    int n;

    if (serial == NULL)
    {
        return APP_SERIAL_STATUS_BAD_ARG;
    }

    parsed = sscanf(serial->line_buf, "%15s", cmd);
    if (parsed != 1)
    {
        return APP_SERIAL_STATUS_EMPTY;
    }

    if (strcmp(cmd, "help") == 0)
    {
        n = snprintf(serial->tx_line, sizeof(serial->tx_line), "RSP help: get all | get <name> | set <name> <value> | en <0|1>\r\n");
        if ((n <= 0) || (n >= (int)sizeof(serial->tx_line)))
        {
            return APP_SERIAL_STATUS_ERROR;
        }
        return app_serial_try_send(serial, serial->tx_line);
    }

    if (strcmp(cmd, "get") == 0)
    {
        parsed = sscanf(serial->line_buf, "%15s %47s", cmd, name);
        if (parsed != 2)
        {
            return app_serial_printf(serial, "ERR usage get <name>|all\r\n");
        }

        if (strcmp(name, "all") == 0)
        {
            serial->dump_active = 1u;
            serial->dump_index = 0u;
            return APP_SERIAL_STATUS_OK;
        }

        if (strcmp(name, "en") == 0)
        {
            return app_serial_send_enable(serial);
        }

        id = app_serial_find_param(name);
        if (id == APP_SERIAL_PARAM_COUNT)
        {
            return app_serial_printf(serial, "ERR unknown_param\r\n");
        }

        return app_serial_send_param(serial, id);
    }

    if (strcmp(cmd, "set") == 0)
    {
        parsed = sscanf(serial->line_buf, "%15s %47s %f", cmd, name, &value);
        if (parsed != 3)
        {
            return app_serial_printf(serial, "ERR usage set <name> <value>\r\n");
        }

        id = app_serial_find_param(name);
        if (id == APP_SERIAL_PARAM_COUNT)
        {
            return app_serial_printf(serial, "ERR unknown_param\r\n");
        }

        if ((serial->set_param == NULL) || (serial->set_param(serial->user_ctx, id, value) != APP_SERIAL_STATUS_OK))
        {
            return app_serial_printf(serial, "ERR bad_value\r\n");
        }

        return app_serial_send_param(serial, id);
    }

    if (strcmp(cmd, "en") == 0)
    {
        parsed = sscanf(serial->line_buf, "%15s %u", cmd, &en_value);
        if (parsed != 2)
        {
            return app_serial_printf(serial, "ERR usage en <0|1>\r\n");
        }

        if ((serial->set_enable == NULL) || (serial->set_enable(serial->user_ctx, (uint8_t)((en_value != 0u) ? 1u : 0u)) != APP_SERIAL_STATUS_OK))
        {
            return app_serial_printf(serial, "ERR en_failed\r\n");
        }

        return app_serial_send_enable(serial);
    }

    return app_serial_printf(serial, "ERR unknown_cmd\r\n");
}

app_serial_status_t app_serial_init(app_serial_t *serial)
{
    if (serial == NULL)
    {
        return APP_SERIAL_STATUS_BAD_ARG;
    }

    serial->rx_wr = 0u;
    serial->rx_rd = 0u;
    serial->line_len = 0u;
    serial->dump_active = 0u;
    serial->tx_busy = 0u;
    serial->dump_index = 0u;
    memset(serial->rx_fifo, 0, sizeof(serial->rx_fifo));
    memset(serial->line_buf, 0, sizeof(serial->line_buf));
    memset(serial->tx_line, 0, sizeof(serial->tx_line));
    app_serial_cdc_set_target(serial);
    return APP_SERIAL_STATUS_OK;
}

app_serial_status_t app_serial_rx_bytes(app_serial_t *serial, const uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint16_t next_wr;

    if ((serial == NULL) || (data == NULL))
    {
        return APP_SERIAL_STATUS_BAD_ARG;
    }

    for (i = 0u; i < len; i++)
    {
        next_wr = (uint16_t)((serial->rx_wr + 1u) % APP_SERIAL_RX_FIFO_SIZE);
        if (next_wr == serial->rx_rd)
        {
            return APP_SERIAL_STATUS_FULL;
        }

        serial->rx_fifo[serial->rx_wr] = data[i];
        serial->rx_wr = next_wr;
    }

    return APP_SERIAL_STATUS_OK;
}

app_serial_status_t app_serial_printf(app_serial_t *serial, const char *fmt, ...)
{
    va_list args;
    int n;

    if ((serial == NULL) || (fmt == NULL))
    {
        return APP_SERIAL_STATUS_BAD_ARG;
    }

    va_start(args, fmt);
    n = vsnprintf(serial->tx_line, sizeof(serial->tx_line), fmt, args);
    va_end(args);

    if ((n <= 0) || (n >= (int)sizeof(serial->tx_line)))
    {
        return APP_SERIAL_STATUS_ERROR;
    }

    return app_serial_try_send(serial, serial->tx_line);
}

app_serial_status_t app_serial_process(app_serial_t *serial)
{
    app_serial_status_t status;
    uint8_t ch;

    if (serial == NULL)
    {
        return APP_SERIAL_STATUS_BAD_ARG;
    }

    if (serial->dump_active != 0u)
    {
        if (serial->dump_index < (uint32_t)APP_SERIAL_PARAM_COUNT)
        {
            status = app_serial_send_param(serial, (app_serial_param_id_t)serial->dump_index);
            if (status == APP_SERIAL_STATUS_BUSY)
            {
                return APP_SERIAL_STATUS_BUSY;
            }
            if (status != APP_SERIAL_STATUS_OK)
            {
                serial->dump_active = 0u;
                return status;
            }
            serial->dump_index++;
            return APP_SERIAL_STATUS_OK;
        }

        serial->dump_active = 0u;
        return app_serial_send_enable(serial);
    }

    while (serial->rx_rd != serial->rx_wr)
    {
        ch = serial->rx_fifo[serial->rx_rd];
        serial->rx_rd = (uint16_t)((serial->rx_rd + 1u) % APP_SERIAL_RX_FIFO_SIZE);

        if (ch == (uint8_t)'\r')
        {
            continue;
        }

        if (ch == (uint8_t)'\n')
        {
            serial->line_buf[serial->line_len] = 0;
            serial->line_len = 0u;
            return app_serial_handle_line(serial);
        }

        if (serial->line_len < (uint16_t)(APP_SERIAL_LINE_SIZE - 1u))
        {
            serial->line_buf[serial->line_len] = (char)ch;
            serial->line_len++;
        }
        else
        {
            serial->line_len = 0u;
            return app_serial_printf(serial, "ERR line_too_long\r\n");
        }
    }

    return APP_SERIAL_STATUS_EMPTY;
}

void app_serial_cdc_set_target(app_serial_t *serial)
{
    g_app_serial_target = serial;
}

void app_serial_cdc_rx_callback(uint8_t *buf, uint32_t len)
{
    if ((g_app_serial_target == NULL) || (buf == NULL) || (len == 0u))
    {
        return;
    }

    (void)app_serial_rx_bytes(g_app_serial_target, buf, len);
}
