// balance_lqr.c
// LQR-регулятор балансировки для STM32F401CCU6
// Версия драйвера: 0.1.1

#include "balance_lqr.h"

#include <string.h>

#define BALANCE_LQR_INVERT_OUTPUT       (1u)

static float balance_lqr_clamp(float x, float lo, float hi)
{
    if (x < lo)
    {
        return lo;
    }
    if (x > hi)
    {
        return hi;
    }
    return x;
}

static void balance_lqr_set_default_config(balance_lqr_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));

    cfg->enabled = 1u;
    cfg->K[0] = 0.0f;
    cfg->K[1] = 0.0f;
    cfg->K[2] = 0.0f;
    cfg->K[3] = 0.0f;
    cfg->u_min = -0.3f;
    cfg->u_max = +0.3f;
}

balance_lqr_status_t balance_lqr_reset(balance_lqr_t *dev)
{
    if (dev == NULL)
    {
        return BALANCE_LQR_STATUS_BAD_PARAM;
    }

    memset(&dev->state, 0, sizeof(dev->state));
    return BALANCE_LQR_STATUS_OK;
}

balance_lqr_status_t balance_lqr_set_config(balance_lqr_t *dev, const balance_lqr_config_t *cfg)
{
    if ((dev == NULL) || (cfg == NULL))
    {
        return BALANCE_LQR_STATUS_BAD_PARAM;
    }

    dev->cfg = *cfg;

    if (dev->cfg.u_min > dev->cfg.u_max)
    {
        float tmp;
        tmp = dev->cfg.u_min;
        dev->cfg.u_min = dev->cfg.u_max;
        dev->cfg.u_max = tmp;
    }

    return BALANCE_LQR_STATUS_OK;
}

balance_lqr_status_t balance_lqr_init(balance_lqr_t *dev, const balance_lqr_config_t *cfg)
{
    balance_lqr_config_t local_cfg;

    if (dev == NULL)
    {
        return BALANCE_LQR_STATUS_BAD_PARAM;
    }

    memset(dev, 0, sizeof(*dev));

    if (cfg == NULL)
    {
        balance_lqr_set_default_config(&local_cfg);
        cfg = &local_cfg;
    }

    (void)balance_lqr_set_config(dev, cfg);
    (void)balance_lqr_reset(dev);

    return BALANCE_LQR_STATUS_OK;
}

balance_lqr_status_t balance_lqr_step(balance_lqr_t *dev, const balance_lqr_state_input_t *in, float *u_out)
{
    float u;

    if ((dev == NULL) || (in == NULL) || (u_out == NULL))
    {
        return BALANCE_LQR_STATUS_BAD_PARAM;
    }

    dev->state.x[0] = in->pitch_rad;
    dev->state.x[1] = in->pitch_rate_rad_s;
    dev->state.x[2] = in->wheel_pos_avg;
    dev->state.x[3] = in->wheel_vel_avg;

    u = 0.0f;
    u -= dev->cfg.K[0] * dev->state.x[0];
    u -= dev->cfg.K[1] * dev->state.x[1];
    u -= dev->cfg.K[2] * dev->state.x[2];
    u -= dev->cfg.K[3] * dev->state.x[3];

#if (BALANCE_LQR_INVERT_OUTPUT != 0u)
    u = -u;
#endif

    dev->state.u_unsat = u;

    if (dev->cfg.enabled == 0u)
    {
        dev->state.u_sat = 0.0f;
        dev->state.valid = 1u;
        dev->state.step_count++;
        *u_out = 0.0f;
        return BALANCE_LQR_STATUS_OK;
    }

    dev->state.u_sat = balance_lqr_clamp(u, dev->cfg.u_min, dev->cfg.u_max);
    dev->state.valid = 1u;
    dev->state.step_count++;
    *u_out = dev->state.u_sat;

    return BALANCE_LQR_STATUS_OK;
}
