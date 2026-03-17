// ekf_driver.h
// EKF-драйвер для STM32F401CCU6
// Источники данных: ICM20948 + ODrive(MCP2515)
// Версия драйвера: 0.1.0
//
// ПРАВИЛА:
// - Все данные хранятся в структурах и передаются через структуры.
// - Все функции возвращают только статус выполнения.
// - Все комментарии только через //.
// - Драйвер не использует динамическую память.
// - EKF запускается по внешнему таймеру с передачей точного времени шага в микросекундах.

#ifndef EKF_DRIVER_H
#define EKF_DRIVER_H

#include "main.h"
#include <stdint.h>
#include "icm20948_driver.h"
#include "can_mcp2515_odrive.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EKF_DRIVER_VERSION_MAJOR        (0u)
#define EKF_DRIVER_VERSION_MINOR        (1u)
#define EKF_DRIVER_VERSION_PATCH        (0u)

#define EKF_STATE_DIM                   (6u)

#define EKF_FLAG_IMU_VALID              (1u << 0)
#define EKF_FLAG_ODRV1_VALID            (1u << 1)
#define EKF_FLAG_ODRV2_VALID            (1u << 2)
#define EKF_FLAG_IMU_INTERP             (1u << 3)
#define EKF_FLAG_ODRV1_INTERP           (1u << 4)
#define EKF_FLAG_ODRV2_INTERP           (1u << 5)
#define EKF_FLAG_IMU_STALE              (1u << 6)
#define EKF_FLAG_ODRV1_STALE            (1u << 7)
#define EKF_FLAG_ODRV2_STALE            (1u << 8)
#define EKF_FLAG_ODRV_PAIR_VALID        (1u << 9)

typedef enum
{
    EKF_STATUS_OK = 0,
    EKF_STATUS_NO_NEW_DATA,
    EKF_STATUS_NOT_READY,
    EKF_STATUS_BAD_PARAM,
    EKF_STATUS_STALE_DATA,
    EKF_STATUS_MATH_ERROR
} ekf_status_t;

typedef struct
{
    uint64_t ts_us;
    uint8_t valid;

    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;
} ekf_imu_sample_t;

typedef struct
{
    ekf_imu_sample_t prev;
    ekf_imu_sample_t curr;
    uint32_t seq;
} ekf_imu_track_t;

typedef struct
{
    uint64_t ts_us;
    uint8_t valid;

    float pos;
    float vel;
} ekf_odrv_sample_t;

typedef struct
{
    ekf_odrv_sample_t prev;
    ekf_odrv_sample_t curr;
    uint32_t seq;
} ekf_odrv_track_t;

typedef struct
{
    uint64_t pair_complete_ts_us;
    uint64_t pair_dt_us;
    uint8_t valid;
    uint32_t seq;
} ekf_odrv_pair_meta_t;

typedef struct
{
    ekf_imu_track_t imu;
    ekf_odrv_track_t odrv1;
    ekf_odrv_track_t odrv2;
    ekf_odrv_pair_meta_t pair;
} ekf_input_history_t;

typedef struct
{
    uint64_t ts_us;
    uint32_t flags;

    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    float pitch_acc_rad;

    float pos1;
    float vel1;
    float pos2;
    float vel2;

    float wheel_pos_avg;
    float wheel_vel_avg;
    float wheel_pos_diff;
    float wheel_vel_diff;

    uint64_t imu_dt_us;
    uint64_t odrv1_dt_us;
    uint64_t odrv2_dt_us;
    uint64_t pair_dt_us;
} ekf_measurement_t;

typedef struct
{
    uint8_t pitch_gyro_axis;
    uint8_t pitch_acc_forward_axis;
    uint8_t pitch_acc_up_axis;

    int8_t pitch_gyro_sign;
    int8_t pitch_acc_forward_sign;
    int8_t pitch_acc_up_sign;

    int8_t wheel1_sign;
    int8_t wheel2_sign;

    uint32_t imu_stale_timeout_us;
    uint32_t odrv_stale_timeout_us;

    float q_pitch;
    float q_gyro_bias;
    float q_wheel_pos;
    float q_wheel_vel;
    float q_diff_pos;
    float q_diff_vel;

    float r_pitch_acc;
    float r_wheel_pos;
    float r_wheel_vel;
    float r_diff_pos;
    float r_diff_vel;

    float p0_pitch;
    float p0_gyro_bias;
    float p0_wheel_pos;
    float p0_wheel_vel;
    float p0_diff_pos;
    float p0_diff_vel;
} ekf_config_t;

typedef struct
{
    uint8_t valid;
    uint32_t step_count;
    uint64_t last_step_ts_us;
    uint64_t last_predict_dt_us;

    float x[EKF_STATE_DIM];
    float P[EKF_STATE_DIM][EKF_STATE_DIM];

    float last_innovation_pitch;
    float last_innovation_wheel_pos;
    float last_innovation_wheel_vel;
    float last_innovation_diff_pos;
    float last_innovation_diff_vel;
} ekf_state_t;

typedef struct
{
    ekf_config_t cfg;
    ekf_input_history_t hist;
    ekf_measurement_t last_meas;
    ekf_state_t state;

    uint8_t step_pending;
    uint64_t pending_step_ts_us;
} ekf_driver_t;

typedef struct
{
    uint64_t ts_us;
    float pitch_rad;
    float gyro_bias_rads;
    float wheel_pos_avg;
    float wheel_vel_avg;
    float wheel_pos_diff;
    float wheel_vel_diff;
    uint32_t step_count;
    uint8_t valid;
} ekf_state_snapshot_t;

ekf_status_t ekf_driver_init(ekf_driver_t *dev, const ekf_config_t *cfg);
ekf_status_t ekf_driver_reset(ekf_driver_t *dev);
ekf_status_t ekf_driver_request_step_isr(ekf_driver_t *dev, uint64_t step_time_us);
ekf_status_t ekf_driver_update_inputs_from_drivers(ekf_driver_t *dev, ICM20948_t *imu, can_mcp2515_odrive_t *odrv);
ekf_status_t ekf_driver_build_measurement_at_time(ekf_driver_t *dev, uint64_t t_us, ekf_measurement_t *out);
ekf_status_t ekf_driver_step_at_time(ekf_driver_t *dev, uint64_t t_us);
ekf_status_t ekf_driver_process_pending(ekf_driver_t *dev);
ekf_status_t ekf_driver_get_state_snapshot(ekf_driver_t *dev, ekf_state_snapshot_t *out);

#ifdef __cplusplus
}
#endif

#endif
