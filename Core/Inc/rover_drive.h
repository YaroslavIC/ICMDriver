#ifndef ROVER_DRIVE_H
#define ROVER_DRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ROVER_DRIVE_MODULE_VERSION       "R02"
#define ROVER_DRIVE_CMD_LIMIT            (1.0f)
#define ROVER_DRIVE_DEFAULT_BOOST_VEL_REV_S  (0.50f)
#define ROVER_DRIVE_DEFAULT_BOOST_MS         (250.0f)
#define ROVER_DRIVE_MAX_BOOST_VEL_REV_S      (5.0f)
#define ROVER_DRIVE_MAX_BOOST_MS             (3000.0f)

// Версия: R02 03.05.26
// Назначение:
// Дифференциальное управление rover с двумя задними ведущими колесами.
// node_id_1 = левое заднее колесо, node_id_2 = правое заднее колесо.
// Модуль содержит только rover-drive управление задней осью.

typedef enum
{
    ROVER_DRIVE_STATUS_OK = 0,
    ROVER_DRIVE_STATUS_BAD_ARG,
    ROVER_DRIVE_STATUS_BAD_VALUE
} rover_drive_status_t;

typedef enum
{
    ROVER_DRIVE_MODE_IDLE = 0,
    ROVER_DRIVE_MODE_READY,
    ROVER_DRIVE_MODE_DRIVE,
    ROVER_DRIVE_MODE_FAULT
} rover_drive_mode_t;

typedef struct
{
    float max_vel_rev_s;       // Максимальная скорость колеса, rev/s
    float max_turn_rev_s;      // Максимальная добавка скорости на поворот, rev/s
    float accel_rev_s2;        // Ограничение ускорения команды, rev/s^2; 0 = без ramp
    float left_dir;            // +1 или -1, node_id_1, левое заднее колесо
    float right_dir;           // +1 или -1, node_id_2, правое заднее колесо
    float cmd_timeout_ms;      // 0 = timeout отключен
} rover_drive_params_t;

typedef struct
{
    float forward_cmd;         // -1.0 ... +1.0
    float turn_cmd;            // -1.0 ... +1.0
    float boost_vel_rev_s;      // Скорость пускового boost, rev/s; 0 = boost отключен
    float boost_ms;             // Время boost после команды drive, ms; 0 = boost отключен
    uint32_t last_cmd_ms;
} rover_drive_command_t;

typedef struct
{
    uint8_t valid;
    uint8_t enabled;
    uint8_t saturated;
    uint8_t timeout;
    rover_drive_mode_t mode;

    float forward_cmd;
    float turn_cmd;
    float boost_vel_rev_s;
    float boost_ms;
    uint8_t boost_active;
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t reserved2;

    float left_target_rev_s;
    float right_target_rev_s;
    float left_output_rev_s;
    float right_output_rev_s;
} rover_drive_output_t;

typedef struct
{
    rover_drive_params_t params;
    rover_drive_output_t out;
    uint32_t last_step_ms;
    uint8_t initialized;
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t reserved2;
} rover_drive_t;

rover_drive_status_t rover_drive_init(rover_drive_t *drv);
rover_drive_status_t rover_drive_set_defaults(rover_drive_params_t *params);
rover_drive_status_t rover_drive_validate_params(const rover_drive_params_t *params);
rover_drive_status_t rover_drive_set_command(rover_drive_command_t *cmd, float forward, float turn, uint32_t now_ms);
rover_drive_status_t rover_drive_set_command_boost(rover_drive_command_t *cmd,
                                                   float forward,
                                                   float turn,
                                                   float boost_vel_rev_s,
                                                   float boost_ms,
                                                   uint32_t now_ms);
void rover_drive_stop(rover_drive_command_t *cmd, uint32_t now_ms);
void rover_drive_reset_outputs(rover_drive_t *drv, uint32_t now_ms);
rover_drive_status_t rover_drive_step(rover_drive_t *drv,
                                      uint8_t enabled,
                                      uint8_t fault,
                                      const rover_drive_command_t *cmd,
                                      uint32_t now_ms,
                                      rover_drive_output_t *out);
const char *rover_drive_mode_to_string(rover_drive_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif
