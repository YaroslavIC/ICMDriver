// ekf_driver.c
// EKF-драйвер для STM32F401CCU6
// Источники данных: ICM20948 + ODrive(MCP2515)
// Версия драйвера: 0.1.0

#include "ekf_driver.h"

#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define EKF_MIN_DT_US                 (100u)
#define EKF_MAX_DT_US                 (50000u)
#define EKF_EPS                       (1.0e-9f)

typedef enum
{
    EKF_INPUT_UPD_NONE  = 0u,
    EKF_INPUT_UPD_IMU   = (1u << 0),
    EKF_INPUT_UPD_ODRV1 = (1u << 1),
    EKF_INPUT_UPD_ODRV2 = (1u << 2),
    EKF_INPUT_UPD_PAIR  = (1u << 3)
} ekf_input_update_mask_t;

static float ekf_wrap_pi(float x)
{
    while (x > (float)M_PI)
    {
        x -= 2.0f * (float)M_PI;
    }
    while (x < -(float)M_PI)
    {
        x += 2.0f * (float)M_PI;
    }
    return x;
}

static uint64_t ekf_abs_diff_u64(uint64_t a, uint64_t b)
{
    if (a >= b)
    {
        return a - b;
    }
    return b - a;
}

static float ekf_lerp_f32(float a, float b, float alpha)
{
    return a + (b - a) * alpha;
}

static void ekf_set_default_config(ekf_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));

    cfg->pitch_gyro_axis = 1u;
    cfg->pitch_acc_forward_axis = 0u;
    cfg->pitch_acc_up_axis = 2u;

    cfg->pitch_gyro_sign = +1;
    cfg->pitch_acc_forward_sign = +1;
    cfg->pitch_acc_up_sign = +1;

    cfg->wheel1_sign = +1;
    cfg->wheel2_sign = +1;

    cfg->imu_stale_timeout_us = 5000u;
    cfg->odrv_stale_timeout_us = 12000u;

    cfg->q_pitch = 0.5f;
    cfg->q_gyro_bias = 0.01f;
    cfg->q_wheel_pos = 0.1f;
    cfg->q_wheel_vel = 3.0f;
    cfg->q_diff_pos = 0.1f;
    cfg->q_diff_vel = 3.0f;

    cfg->r_pitch_acc = 0.03f;
    cfg->r_wheel_pos = 0.001f;
    cfg->r_wheel_vel = 0.01f;
    cfg->r_diff_pos = 0.001f;
    cfg->r_diff_vel = 0.01f;

    cfg->p0_pitch = 0.3f;
    cfg->p0_gyro_bias = 0.2f;
    cfg->p0_wheel_pos = 1.0f;
    cfg->p0_wheel_vel = 1.0f;
    cfg->p0_diff_pos = 1.0f;
    cfg->p0_diff_vel = 1.0f;
}

static void ekf_hist_shift_imu(ekf_imu_track_t *trk, const ICM20948_PhysSample_t *s)
{
    trk->prev = trk->curr;

    trk->curr.ts_us = s->timestamp_us;
    trk->curr.valid = 1u;
    trk->curr.ax = s->accel_mps2[0];
    trk->curr.ay = s->accel_mps2[1];
    trk->curr.az = s->accel_mps2[2];
    trk->curr.gx = s->gyro_rads[0];
    trk->curr.gy = s->gyro_rads[1];
    trk->curr.gz = s->gyro_rads[2];
    trk->seq++;
}

static void ekf_hist_shift_odrv(ekf_odrv_track_t *trk, float pos, float vel, uint64_t ts_us)
{
    trk->prev = trk->curr;

    trk->curr.ts_us = ts_us;
    trk->curr.valid = 1u;
    trk->curr.pos = pos;
    trk->curr.vel = vel;
    trk->seq++;
}

static float ekf_get_imu_axis_acc(const ekf_imu_sample_t *s, uint8_t axis)
{
    if (axis == 0u)
    {
        return s->ax;
    }
    if (axis == 1u)
    {
        return s->ay;
    }
    return s->az;
}

static float ekf_get_imu_axis_gyro(const ekf_imu_sample_t *s, uint8_t axis)
{
    if (axis == 0u)
    {
        return s->gx;
    }
    if (axis == 1u)
    {
        return s->gy;
    }
    return s->gz;
}

static float ekf_compute_pitch_acc_rad(const ekf_config_t *cfg, const ekf_imu_sample_t *s)
{
    float acc_fwd;
    float acc_up;

    acc_fwd = (float)cfg->pitch_acc_forward_sign * ekf_get_imu_axis_acc(s, cfg->pitch_acc_forward_axis);
    acc_up = (float)cfg->pitch_acc_up_sign * ekf_get_imu_axis_acc(s, cfg->pitch_acc_up_axis);

    return atan2f(-acc_fwd, acc_up);
}

static ekf_status_t ekf_interpolate_imu(const ekf_imu_track_t *trk,
                                        uint64_t t_us,
                                        uint32_t stale_timeout_us,
                                        ekf_imu_sample_t *out,
                                        uint8_t *is_interp,
                                        uint8_t *is_stale)
{
    uint64_t age_us;

    memset(out, 0, sizeof(*out));
    *is_interp = 0u;
    *is_stale = 0u;

    if ((trk->curr.valid == 0u) || (trk->curr.ts_us == 0ull))
    {
        return EKF_STATUS_NOT_READY;
    }

    age_us = (t_us >= trk->curr.ts_us) ? (t_us - trk->curr.ts_us) : 0ull;
    if (age_us > (uint64_t)stale_timeout_us)
    {
        *is_stale = 1u;
    }

    if ((trk->prev.valid != 0u) &&
        (trk->prev.ts_us < trk->curr.ts_us) &&
        (t_us >= trk->prev.ts_us) &&
        (t_us <= trk->curr.ts_us))
    {
        float alpha;
        uint64_t dt_us;

        dt_us = trk->curr.ts_us - trk->prev.ts_us;
        if (dt_us == 0ull)
        {
            return EKF_STATUS_MATH_ERROR;
        }

        alpha = (float)(t_us - trk->prev.ts_us) / (float)dt_us;

        out->ts_us = t_us;
        out->valid = 1u;
        out->ax = ekf_lerp_f32(trk->prev.ax, trk->curr.ax, alpha);
        out->ay = ekf_lerp_f32(trk->prev.ay, trk->curr.ay, alpha);
        out->az = ekf_lerp_f32(trk->prev.az, trk->curr.az, alpha);
        out->gx = ekf_lerp_f32(trk->prev.gx, trk->curr.gx, alpha);
        out->gy = ekf_lerp_f32(trk->prev.gy, trk->curr.gy, alpha);
        out->gz = ekf_lerp_f32(trk->prev.gz, trk->curr.gz, alpha);
        *is_interp = 1u;
        return (*is_stale != 0u) ? EKF_STATUS_STALE_DATA : EKF_STATUS_OK;
    }

    *out = trk->curr;
    return (*is_stale != 0u) ? EKF_STATUS_STALE_DATA : EKF_STATUS_OK;
}

static ekf_status_t ekf_interpolate_odrv(const ekf_odrv_track_t *trk,
                                         uint64_t t_us,
                                         uint32_t stale_timeout_us,
                                         ekf_odrv_sample_t *out,
                                         uint8_t *is_interp,
                                         uint8_t *is_stale)
{
    uint64_t age_us;

    memset(out, 0, sizeof(*out));
    *is_interp = 0u;
    *is_stale = 0u;

    if ((trk->curr.valid == 0u) || (trk->curr.ts_us == 0ull))
    {
        return EKF_STATUS_NOT_READY;
    }

    age_us = (t_us >= trk->curr.ts_us) ? (t_us - trk->curr.ts_us) : 0ull;
    if (age_us > (uint64_t)stale_timeout_us)
    {
        *is_stale = 1u;
    }

    if ((trk->prev.valid != 0u) &&
        (trk->prev.ts_us < trk->curr.ts_us) &&
        (t_us >= trk->prev.ts_us) &&
        (t_us <= trk->curr.ts_us))
    {
        float alpha;
        uint64_t dt_us;

        dt_us = trk->curr.ts_us - trk->prev.ts_us;
        if (dt_us == 0ull)
        {
            return EKF_STATUS_MATH_ERROR;
        }

        alpha = (float)(t_us - trk->prev.ts_us) / (float)dt_us;

        out->ts_us = t_us;
        out->valid = 1u;
        out->pos = ekf_lerp_f32(trk->prev.pos, trk->curr.pos, alpha);
        out->vel = ekf_lerp_f32(trk->prev.vel, trk->curr.vel, alpha);
        *is_interp = 1u;
        return (*is_stale != 0u) ? EKF_STATUS_STALE_DATA : EKF_STATUS_OK;
    }

    *out = trk->curr;
    return (*is_stale != 0u) ? EKF_STATUS_STALE_DATA : EKF_STATUS_OK;
}

static void ekf_state_set_identity_cov(ekf_state_t *st, const ekf_config_t *cfg)
{
    uint32_t i;
    uint32_t j;

    memset(st->P, 0, sizeof(st->P));

    for (i = 0u; i < EKF_STATE_DIM; i++)
    {
        for (j = 0u; j < EKF_STATE_DIM; j++)
        {
            st->P[i][j] = 0.0f;
        }
    }

    st->P[0][0] = cfg->p0_pitch;
    st->P[1][1] = cfg->p0_gyro_bias;
    st->P[2][2] = cfg->p0_wheel_pos;
    st->P[3][3] = cfg->p0_wheel_vel;
    st->P[4][4] = cfg->p0_diff_pos;
    st->P[5][5] = cfg->p0_diff_vel;
}

static void ekf_predict(ekf_driver_t *dev, float dt_s, const ekf_measurement_t *meas)
{
    float x_prev[EKF_STATE_DIM];
    float F[EKF_STATE_DIM][EKF_STATE_DIM];
    float FP[EKF_STATE_DIM][EKF_STATE_DIM];
    float Pn[EKF_STATE_DIM][EKF_STATE_DIM];
    float q_diag[EKF_STATE_DIM];
    uint32_t i;
    uint32_t j;
    uint32_t k;
    float gyro_pitch;

    memcpy(x_prev, dev->state.x, sizeof(x_prev));

    if (dev->cfg.pitch_gyro_axis == 0u)
    {
        gyro_pitch = (float)dev->cfg.pitch_gyro_sign * meas->gx;
    }
    else if (dev->cfg.pitch_gyro_axis == 1u)
    {
        gyro_pitch = (float)dev->cfg.pitch_gyro_sign * meas->gy;
    }
    else
    {
        gyro_pitch = (float)dev->cfg.pitch_gyro_sign * meas->gz;
    }

    dev->state.x[0] = ekf_wrap_pi(x_prev[0] + dt_s * (gyro_pitch - x_prev[1]));
    dev->state.x[1] = x_prev[1];
    dev->state.x[2] = x_prev[2] + dt_s * x_prev[3];
    dev->state.x[3] = x_prev[3];
    dev->state.x[4] = x_prev[4] + dt_s * x_prev[5];
    dev->state.x[5] = x_prev[5];

    memset(F, 0, sizeof(F));
    F[0][0] = 1.0f;
    F[0][1] = -dt_s;
    F[1][1] = 1.0f;
    F[2][2] = 1.0f;
    F[2][3] = dt_s;
    F[3][3] = 1.0f;
    F[4][4] = 1.0f;
    F[4][5] = dt_s;
    F[5][5] = 1.0f;

    for (i = 0u; i < EKF_STATE_DIM; i++)
    {
        for (j = 0u; j < EKF_STATE_DIM; j++)
        {
            FP[i][j] = 0.0f;
            for (k = 0u; k < EKF_STATE_DIM; k++)
            {
                FP[i][j] += F[i][k] * dev->state.P[k][j];
            }
        }
    }

    for (i = 0u; i < EKF_STATE_DIM; i++)
    {
        for (j = 0u; j < EKF_STATE_DIM; j++)
        {
            Pn[i][j] = 0.0f;
            for (k = 0u; k < EKF_STATE_DIM; k++)
            {
                Pn[i][j] += FP[i][k] * F[j][k];
            }
        }
    }

    q_diag[0] = dev->cfg.q_pitch * dt_s;
    q_diag[1] = dev->cfg.q_gyro_bias * dt_s;
    q_diag[2] = dev->cfg.q_wheel_pos * dt_s;
    q_diag[3] = dev->cfg.q_wheel_vel * dt_s;
    q_diag[4] = dev->cfg.q_diff_pos * dt_s;
    q_diag[5] = dev->cfg.q_diff_vel * dt_s;

    for (i = 0u; i < EKF_STATE_DIM; i++)
    {
        Pn[i][i] += q_diag[i];
    }

    memcpy(dev->state.P, Pn, sizeof(Pn));
}

static ekf_status_t ekf_update_scalar(ekf_driver_t *dev,
                                      uint32_t state_idx,
                                      float z,
                                      float R,
                                      float *innovation_out)
{
    float HPH;
    float S;
    float K[EKF_STATE_DIM];
    float y;
    float newP[EKF_STATE_DIM][EKF_STATE_DIM];
    uint32_t i;
    uint32_t j;

    HPH = dev->state.P[state_idx][state_idx];
    S = HPH + R;
    if (S <= EKF_EPS)
    {
        return EKF_STATUS_MATH_ERROR;
    }

    y = z - dev->state.x[state_idx];
    if (state_idx == 0u)
    {
        y = ekf_wrap_pi(y);
    }

    for (i = 0u; i < EKF_STATE_DIM; i++)
    {
        K[i] = dev->state.P[i][state_idx] / S;
        dev->state.x[i] += K[i] * y;
    }
    dev->state.x[0] = ekf_wrap_pi(dev->state.x[0]);

    for (i = 0u; i < EKF_STATE_DIM; i++)
    {
        for (j = 0u; j < EKF_STATE_DIM; j++)
        {
            newP[i][j] = dev->state.P[i][j] - K[i] * dev->state.P[state_idx][j];
        }
    }

    memcpy(dev->state.P, newP, sizeof(newP));
    *innovation_out = y;
    return EKF_STATUS_OK;
}

static void ekf_bootstrap_state(ekf_driver_t *dev, const ekf_measurement_t *meas)
{
    if ((meas->flags & EKF_FLAG_IMU_VALID) != 0u)
    {
        dev->state.x[0] = meas->pitch_acc_rad;
        dev->state.x[1] = 0.0f;
    }

    if (((meas->flags & EKF_FLAG_ODRV1_VALID) != 0u) &&
        ((meas->flags & EKF_FLAG_ODRV2_VALID) != 0u))
    {
        dev->state.x[2] = meas->wheel_pos_avg;
        dev->state.x[3] = meas->wheel_vel_avg;
        dev->state.x[4] = meas->wheel_pos_diff;
        dev->state.x[5] = meas->wheel_vel_diff;
    }
}

ekf_status_t ekf_driver_init(ekf_driver_t *dev, const ekf_config_t *cfg)
{
    if (dev == NULL)
    {
        return EKF_STATUS_BAD_PARAM;
    }

    memset(dev, 0, sizeof(*dev));

    if (cfg != NULL)
    {
        dev->cfg = *cfg;
    }
    else
    {
        ekf_set_default_config(&dev->cfg);
    }

    if ((dev->cfg.pitch_gyro_axis > 2u) ||
        (dev->cfg.pitch_acc_forward_axis > 2u) ||
        (dev->cfg.pitch_acc_up_axis > 2u))
    {
        return EKF_STATUS_BAD_PARAM;
    }

    if ((dev->cfg.wheel1_sign != +1) && (dev->cfg.wheel1_sign != -1))
    {
        return EKF_STATUS_BAD_PARAM;
    }

    if ((dev->cfg.wheel2_sign != +1) && (dev->cfg.wheel2_sign != -1))
    {
        return EKF_STATUS_BAD_PARAM;
    }

    return ekf_driver_reset(dev);
}

ekf_status_t ekf_driver_reset(ekf_driver_t *dev)
{
    if (dev == NULL)
    {
        return EKF_STATUS_BAD_PARAM;
    }

    memset(&dev->hist, 0, sizeof(dev->hist));
    memset(&dev->last_meas, 0, sizeof(dev->last_meas));
    memset(&dev->state, 0, sizeof(dev->state));
    dev->step_pending = 0u;
    dev->pending_step_ts_us = 0ull;

    ekf_state_set_identity_cov(&dev->state, &dev->cfg);
    return EKF_STATUS_OK;
}

ekf_status_t ekf_driver_request_step_isr(ekf_driver_t *dev, uint64_t step_time_us)
{
    if (dev == NULL)
    {
        return EKF_STATUS_BAD_PARAM;
    }

    dev->pending_step_ts_us = step_time_us;
    dev->step_pending = 1u;
    return EKF_STATUS_OK;
}

ekf_status_t ekf_driver_update_inputs_from_drivers(ekf_driver_t *dev, ICM20948_t *imu, can_mcp2515_odrive_t *odrv)
{
    uint32_t updates;
    ICM20948_PhysSample_t imu_sample;

    if ((dev == NULL) || (imu == NULL) || (odrv == NULL))
    {
        return EKF_STATUS_BAD_PARAM;
    }

    updates = EKF_INPUT_UPD_NONE;

    if (ICM20948_GetLatestPhys(imu, &imu_sample) == ICM20948_OK)
    {
        if ((dev->hist.imu.curr.valid == 0u) ||
            (imu_sample.timestamp_us != dev->hist.imu.curr.ts_us))
        {
            ekf_hist_shift_imu(&dev->hist.imu, &imu_sample);
            updates |= EKF_INPUT_UPD_IMU;
        }
    }

    if ((odrv->pair.ready != 0u) &&
        (odrv->pair.pair_complete_ts_us != 0ull) &&
        ((dev->hist.pair.valid == 0u) ||
         (odrv->pair.pair_complete_ts_us != dev->hist.pair.pair_complete_ts_us)))
    {
        ekf_hist_shift_odrv(&dev->hist.odrv1, odrv->pair.pos1, odrv->pair.vel1, odrv->pair.ts1_us);
        ekf_hist_shift_odrv(&dev->hist.odrv2, odrv->pair.pos2, odrv->pair.vel2, odrv->pair.ts2_us);

        dev->hist.pair.valid = 1u;
        dev->hist.pair.pair_complete_ts_us = odrv->pair.pair_complete_ts_us;
        dev->hist.pair.pair_dt_us = odrv->pair.pair_dt_us;
        dev->hist.pair.seq++;

        updates |= EKF_INPUT_UPD_ODRV1;
        updates |= EKF_INPUT_UPD_ODRV2;
        updates |= EKF_INPUT_UPD_PAIR;
    }
    else
    {
        if ((odrv->node1.encoder.valid != 0u) &&
            (odrv->node1.encoder.rsp_ts_us != 0ull) &&
            ((dev->hist.odrv1.curr.valid == 0u) ||
             (odrv->node1.encoder.rsp_ts_us != dev->hist.odrv1.curr.ts_us)))
        {
            ekf_hist_shift_odrv(&dev->hist.odrv1,
                                odrv->node1.encoder.pos_estimate,
                                odrv->node1.encoder.vel_estimate,
                                odrv->node1.encoder.rsp_ts_us);
            updates |= EKF_INPUT_UPD_ODRV1;
        }

        if ((odrv->node2.encoder.valid != 0u) &&
            (odrv->node2.encoder.rsp_ts_us != 0ull) &&
            ((dev->hist.odrv2.curr.valid == 0u) ||
             (odrv->node2.encoder.rsp_ts_us != dev->hist.odrv2.curr.ts_us)))
        {
            ekf_hist_shift_odrv(&dev->hist.odrv2,
                                odrv->node2.encoder.pos_estimate,
                                odrv->node2.encoder.vel_estimate,
                                odrv->node2.encoder.rsp_ts_us);
            updates |= EKF_INPUT_UPD_ODRV2;
        }
    }

    if (updates == EKF_INPUT_UPD_NONE)
    {
        return EKF_STATUS_NO_NEW_DATA;
    }
    return EKF_STATUS_OK;
}

ekf_status_t ekf_driver_build_measurement_at_time(ekf_driver_t *dev, uint64_t t_us, ekf_measurement_t *out)
{
    ekf_status_t st_imu;
    ekf_status_t st_odrv1;
    ekf_status_t st_odrv2;
    ekf_imu_sample_t imu_s;
    ekf_odrv_sample_t od1_s;
    ekf_odrv_sample_t od2_s;
    uint8_t imu_interp;
    uint8_t imu_stale;
    uint8_t od1_interp;
    uint8_t od1_stale;
    uint8_t od2_interp;
    uint8_t od2_stale;

    if ((dev == NULL) || (out == NULL))
    {
        return EKF_STATUS_BAD_PARAM;
    }

    memset(out, 0, sizeof(*out));
    out->ts_us = t_us;

    st_imu = ekf_interpolate_imu(&dev->hist.imu,
                                 t_us,
                                 dev->cfg.imu_stale_timeout_us,
                                 &imu_s,
                                 &imu_interp,
                                 &imu_stale);

    st_odrv1 = ekf_interpolate_odrv(&dev->hist.odrv1,
                                    t_us,
                                    dev->cfg.odrv_stale_timeout_us,
                                    &od1_s,
                                    &od1_interp,
                                    &od1_stale);

    st_odrv2 = ekf_interpolate_odrv(&dev->hist.odrv2,
                                    t_us,
                                    dev->cfg.odrv_stale_timeout_us,
                                    &od2_s,
                                    &od2_interp,
                                    &od2_stale);

    if ((st_imu == EKF_STATUS_OK) || (st_imu == EKF_STATUS_STALE_DATA))
    {
        out->ax = imu_s.ax;
        out->ay = imu_s.ay;
        out->az = imu_s.az;
        out->gx = imu_s.gx;
        out->gy = imu_s.gy;
        out->gz = imu_s.gz;
        out->pitch_acc_rad = ekf_compute_pitch_acc_rad(&dev->cfg, &imu_s);
        out->flags |= EKF_FLAG_IMU_VALID;
        out->imu_dt_us = ekf_abs_diff_u64(t_us, imu_s.ts_us);
        if (imu_interp != 0u)
        {
            out->flags |= EKF_FLAG_IMU_INTERP;
        }
        if (imu_stale != 0u)
        {
            out->flags |= EKF_FLAG_IMU_STALE;
        }
    }

    if ((st_odrv1 == EKF_STATUS_OK) || (st_odrv1 == EKF_STATUS_STALE_DATA))
    {
        out->pos1 = (float)dev->cfg.wheel1_sign * od1_s.pos;
        out->vel1 = (float)dev->cfg.wheel1_sign * od1_s.vel;
        out->flags |= EKF_FLAG_ODRV1_VALID;
        out->odrv1_dt_us = ekf_abs_diff_u64(t_us, od1_s.ts_us);
        if (od1_interp != 0u)
        {
            out->flags |= EKF_FLAG_ODRV1_INTERP;
        }
        if (od1_stale != 0u)
        {
            out->flags |= EKF_FLAG_ODRV1_STALE;
        }
    }

    if ((st_odrv2 == EKF_STATUS_OK) || (st_odrv2 == EKF_STATUS_STALE_DATA))
    {
        out->pos2 = (float)dev->cfg.wheel2_sign * od2_s.pos;
        out->vel2 = (float)dev->cfg.wheel2_sign * od2_s.vel;
        out->flags |= EKF_FLAG_ODRV2_VALID;
        out->odrv2_dt_us = ekf_abs_diff_u64(t_us, od2_s.ts_us);
        if (od2_interp != 0u)
        {
            out->flags |= EKF_FLAG_ODRV2_INTERP;
        }
        if (od2_stale != 0u)
        {
            out->flags |= EKF_FLAG_ODRV2_STALE;
        }
    }

    if (((out->flags & EKF_FLAG_ODRV1_VALID) != 0u) &&
        ((out->flags & EKF_FLAG_ODRV2_VALID) != 0u))
    {
        out->wheel_pos_avg = 0.5f * (out->pos1 + out->pos2);
        out->wheel_vel_avg = 0.5f * (out->vel1 + out->vel2);
        out->wheel_pos_diff = 0.5f * (out->pos2 - out->pos1);
        out->wheel_vel_diff = 0.5f * (out->vel2 - out->vel1);
        out->flags |= EKF_FLAG_ODRV_PAIR_VALID;

        if (dev->hist.pair.valid != 0u)
        {
            out->pair_dt_us = dev->hist.pair.pair_dt_us;
        }
        else if ((od1_s.ts_us != 0ull) && (od2_s.ts_us != 0ull))
        {
            out->pair_dt_us = ekf_abs_diff_u64(od1_s.ts_us, od2_s.ts_us);
        }
    }

    if (out->flags == 0u)
    {
        return EKF_STATUS_NOT_READY;
    }

    return EKF_STATUS_OK;
}

ekf_status_t ekf_driver_step_at_time(ekf_driver_t *dev, uint64_t t_us)
{
    ekf_status_t st;
    ekf_measurement_t meas;
    float dt_s;
    uint64_t dt_us;

    if (dev == NULL)
    {
        return EKF_STATUS_BAD_PARAM;
    }

    st = ekf_driver_build_measurement_at_time(dev, t_us, &meas);
    if ((st != EKF_STATUS_OK) && (st != EKF_STATUS_STALE_DATA))
    {
        return st;
    }

    if (dev->state.valid == 0u)
    {
        ekf_bootstrap_state(dev, &meas);
        dev->state.valid = 1u;
        dev->state.last_step_ts_us = t_us;
        dev->state.last_predict_dt_us = 0ull;
        dev->state.step_count = 1u;
        dev->last_meas = meas;
        return EKF_STATUS_OK;
    }

    if (t_us <= dev->state.last_step_ts_us)
    {
        dt_us = (uint64_t)EKF_MIN_DT_US;
    }
    else
    {
        dt_us = t_us - dev->state.last_step_ts_us;
    }
    if (dt_us < (uint64_t)EKF_MIN_DT_US)
    {
        dt_us = (uint64_t)EKF_MIN_DT_US;
    }
    if (dt_us > (uint64_t)EKF_MAX_DT_US)
    {
        dt_us = (uint64_t)EKF_MAX_DT_US;
    }

    dt_s = (float)dt_us * 1.0e-6f;

    ekf_predict(dev, dt_s, &meas);

    if (((meas.flags & EKF_FLAG_IMU_VALID) != 0u) &&
        ((meas.flags & EKF_FLAG_IMU_STALE) == 0u))
    {
        st = ekf_update_scalar(dev, 0u, meas.pitch_acc_rad, dev->cfg.r_pitch_acc, &dev->state.last_innovation_pitch);
        if (st != EKF_STATUS_OK)
        {
            return st;
        }
    }

    if (((meas.flags & EKF_FLAG_ODRV_PAIR_VALID) != 0u) &&
        ((meas.flags & EKF_FLAG_ODRV1_STALE) == 0u) &&
        ((meas.flags & EKF_FLAG_ODRV2_STALE) == 0u))
    {
        st = ekf_update_scalar(dev, 2u, meas.wheel_pos_avg, dev->cfg.r_wheel_pos, &dev->state.last_innovation_wheel_pos);
        if (st != EKF_STATUS_OK)
        {
            return st;
        }

        st = ekf_update_scalar(dev, 3u, meas.wheel_vel_avg, dev->cfg.r_wheel_vel, &dev->state.last_innovation_wheel_vel);
        if (st != EKF_STATUS_OK)
        {
            return st;
        }

        st = ekf_update_scalar(dev, 4u, meas.wheel_pos_diff, dev->cfg.r_diff_pos, &dev->state.last_innovation_diff_pos);
        if (st != EKF_STATUS_OK)
        {
            return st;
        }

        st = ekf_update_scalar(dev, 5u, meas.wheel_vel_diff, dev->cfg.r_diff_vel, &dev->state.last_innovation_diff_vel);
        if (st != EKF_STATUS_OK)
        {
            return st;
        }
    }

    dev->state.last_step_ts_us = t_us;
    dev->state.last_predict_dt_us = dt_us;
    dev->state.step_count++;
    dev->last_meas = meas;

    return EKF_STATUS_OK;
}

ekf_status_t ekf_driver_process_pending(ekf_driver_t *dev)
{
    ekf_status_t st;
    uint64_t t_us;

    if (dev == NULL)
    {
        return EKF_STATUS_BAD_PARAM;
    }

    if (dev->step_pending == 0u)
    {
        return EKF_STATUS_NO_NEW_DATA;
    }

    t_us = dev->pending_step_ts_us;
    dev->step_pending = 0u;

    st = ekf_driver_step_at_time(dev, t_us);
    return st;
}

ekf_status_t ekf_driver_get_state_snapshot(ekf_driver_t *dev, ekf_state_snapshot_t *out)
{
    if ((dev == NULL) || (out == NULL))
    {
        return EKF_STATUS_BAD_PARAM;
    }

    memset(out, 0, sizeof(*out));

    out->ts_us = dev->state.last_step_ts_us;
    out->pitch_rad = dev->state.x[0];
    out->pitch_rate_rad_s = (dev->last_meas.gy - dev->state.x[1]);
    out->gyro_bias_rads = dev->state.x[1];
    out->wheel_pos_avg = dev->state.x[2];
    out->wheel_vel_avg = dev->state.x[3];
    out->wheel_pos_diff = dev->state.x[4];
    out->wheel_vel_diff = dev->state.x[5];
    out->wheel_pos_left_raw = dev->last_meas.pos1;
    out->wheel_vel_left_raw = dev->last_meas.vel1;
    out->wheel_pos_right_raw = dev->last_meas.pos2;
    out->wheel_vel_right_raw = dev->last_meas.vel2;
    out->wheel_left_raw_ts_us = dev->hist.odrv1.curr.ts_us;
    out->wheel_right_raw_ts_us = dev->hist.odrv2.curr.ts_us;
    out->step_count = dev->state.step_count;
    out->valid = dev->state.valid;

    return EKF_STATUS_OK;
}
