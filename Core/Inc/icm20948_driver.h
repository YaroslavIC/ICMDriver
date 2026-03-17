// icm20948_driver.h
// Драйвер ICM-20948 (только Accel+Gyro) по SPI + INT + DMA для STM32F401CCU6 (HAL)
// Версия драйвера: 1.1.0
//
// ПРАВИЛА:
// - Все данные (включая параметры инициализации и выходные данные) хранятся в структурах и передаются через структуры.
// - Все функции возвращают только статус выполнения.
// - В коде и примерах не использовать последовательности /* или */. Все комментарии только через //.
// - Комментарии на русском в кодировке Windows-1251.
// - Константы из документации держать в #define.
// - Для переменных, используемых и в main loop, и в прерываниях, использовать защиту (критические секции).
//
// Пример использования (можно вставить в main.c):
//
// // Глобальные хэндлы (создаются в main.c / CubeMX):
// SPI_HandleTypeDef hspi1;
// DMA_HandleTypeDef hdma_spi1_rx;
// DMA_HandleTypeDef hdma_spi1_tx;
// ICM20948_t imu;
//
// void app_init(void)
// {
//     ICM20948_InitParams_t p;
//     p.hspi = &hspi1;
//     p.hdma_rx = &hdma_spi1_rx;
//     p.hdma_tx = &hdma_spi1_tx;
//     p.cs_port = GPIOB;
//     p.cs_pin  = GPIO_PIN_6; // CS -> PB6
//     p.int_port = GPIOB;
//     p.int_pin  = GPIO_PIN_7; // INT -> PB7 (EXTI7)
//     p.spi_timeout_ms = ICM20948_SPI_TIMEOUT_MS_DEFAULT;
//     (void)ICM20948_Init(&imu, &p);
// }
//
// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//     (void)ICM20948_IrqHandler(&imu, GPIO_Pin);
// }
//
// void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
// {
//     (void)ICM20948_SpiDmaCpltHandler(&imu, hspi);
// }
//
// // В stm32f4xx_it.c должны быть IRQ обработчики HAL:
// // void EXTI9_5_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7); }
// // void DMA1_Stream2_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_spi3_rx); }
// // void DMA1_Stream5_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_spi3_tx); }
//
// Примечание по GPIO:
// - Пины SPI3 (SCK/MISO/MOSI) настраиваются ВНЕ драйвера (CubeMX).
// - EXTI для PB7 настраивается ВНЕ драйвера (CubeMX).
// - CS (PB6) драйвер настраивает как GPIO output и держит в 1 в простое.
//
// Примечание по времени:
// - Драйвер использует DWT->CYCCNT и расширяет его до 64 бит.
// - Если DWT не запускается или не тикает, инициализация завершается с ошибкой (fallback не используется).

#ifndef ICM20948_DRIVER_H
#define ICM20948_DRIVER_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Версия драйвера (увеличивать при любом изменении кода)
#define ICM20948_DRIVER_VERSION_MAJOR          (1u)
#define ICM20948_DRIVER_VERSION_MINOR          (1u)
#define ICM20948_DRIVER_VERSION_PATCH          (0u)

// DMA IRQ по умолчанию для варианта SPI1 на STM32F401
// RX: DMA2_Stream0 Channel 3, TX: DMA2_Stream3 Channel 3
// Эти define используются только для включения NVIC на DMA
#define ICM20948_DMA_RX_STREAM_INSTANCE        (DMA2_Stream0)
#define ICM20948_DMA_TX_STREAM_INSTANCE        (DMA2_Stream3)
#define ICM20948_DMA_CHANNEL_DEFAULT           (DMA_CHANNEL_3)

#define ICM20948_DMA_RX_IRQn                   (DMA2_Stream0_IRQn)
#define ICM20948_DMA_TX_IRQn                   (DMA2_Stream3_IRQn)

#define ICM20948_DMA_RX_IRQ_PRIO               (5u)
#define ICM20948_DMA_TX_IRQ_PRIO               (5u)

// Таймаут для blocking операций SPI в инициализации
#define ICM20948_SPI_TIMEOUT_MS_DEFAULT        (10u)

// Настройки SPI по умолчанию (можно менять под проект)
// Для ICM-20948 обычно подходит SPI Mode 0 (CPOL=0, CPHA=1EDGE)
#define ICM20948_SPI_BAUDRATE_PRESCALER_DEFAULT   (SPI_BAUDRATEPRESCALER_8)
#define ICM20948_SPI_CPOL_DEFAULT                 (SPI_POLARITY_LOW)
#define ICM20948_SPI_CPHA_DEFAULT                 (SPI_PHASE_1EDGE)


// Dummy byte для SPI чтения
#define ICM20948_SPI_DUMMY_BYTE_DEFAULT        (0x00u)

// Burst формат: 1 байт адрес + 16 байт payload = 17 байт транзакция
#define ICM20948_BURST_PAYLOAD_BYTES_DEFAULT   (16u)
#define ICM20948_BURST_TOTAL_BYTES             (1u + ICM20948_BURST_PAYLOAD_BYTES_DEFAULT)

// Используемые данные внутри payload: 14 байт (accel 6 + temp 2 + gyro 6)
#define ICM20948_IMU_DATA_BYTES_USED           (14u)

// Частота по умолчанию: около 1125 Гц (делители = 0)
// Пример расчета: F = 1125 / (1 + div). Для div=0: F=1125 Гц.
#define ICM20948_RATE_GYRO_SMPLRT_DIV_DEFAULT  (0u)
// Для акселя используется 16-битный делитель: div = (div1<<8)|div2. Для 0: ~1125 Гц.
#define ICM20948_RATE_ACCEL_SMPLRT_DIV_DEFAULT (0u)

// Дефолтные диапазоны для балансирующего робота
#define ICM20948_ACCEL_FS_G_DEFAULT            (2u)     // +/-2g
#define ICM20948_GYRO_FS_DPS_DEFAULT           (250u)   // +/-250 dps

// Дефолтные фильтры (частоты задаются как ориентиры; точное кодирование зависит от таблицы DLPF)
#define ICM20948_GYRO_LPF_HZ_DEFAULT           (120u)
#define ICM20948_ACCEL_LPF_HZ_DEFAULT          (80u)

// Для получения ODR = 1125/(1+DIV) по документации DLPF должен быть включен.
// Выбираем ближайшие режимы по таблицам даташита:
// gyro DLPFCFG=2 -> 119.5 Hz
// accel DLPFCFG=2 -> 111.4 Hz
#define ICM20948_GYRO_DLPFCFG_DEFAULT         (2u)
#define ICM20948_ACCEL_DLPFCFG_DEFAULT        (2u)
#define ICM20948_ODR_ALIGN_EN_DEFAULT         (1u)

// Константы пересчета
#define ICM20948_G_TO_MPS2                     (9.80665f)
#define ICM20948_DEG_TO_RAD                    (0.017453292519943295f)

// Поведение при занятости SPI: INT игнорируется
#define ICM20948_DROP_SAMPLE_WHEN_BUSY         (1u)

// Настройка INT1 по умолчанию
#define ICM20948_INT_ACTIVE_HIGH_DEFAULT        (1u)
#define ICM20948_INT_PUSH_PULL_DEFAULT          (1u)
#define ICM20948_INT_LATCHED_DEFAULT            (0u)   // 0 = pulse, 1 = latched

// Калибровка 6 шагов
#define ICM20948_CAL_SAMPLES_PER_STEP_DEFAULT  (2048u)
#define ICM20948_GYRO_CAL_SAMPLES_DEFAULT      (1024u)
#define ICM20948_CAL_GYRO_MOVING_THR_RADS      (0.035f)  // ~2 dps
#define ICM20948_CAL_ACCEL_DELTA_THR_G         (0.02f)
#define ICM20948_CAL_DOM_AXIS_G_MIN            (0.8f)
#define ICM20948_CAL_DOM_AXIS_G_MAX            (1.2f)
// 1 retry => всего 2 попытки, затем принять данные
#define ICM20948_CAL_MAX_RETRY_PER_STEP        (1u)
#define ICM20948_CAL_ACCEPT_AFTER_SECOND_FAIL  (1u)
// Если шаг уже был собран, SetStep перезаписывает
#define ICM20948_CAL_OVERWRITE_EXISTING_STEP   (1u)
// Когда все 6 шагов готовы, SetStep ничего не делает до Reset
#define ICM20948_CAL_SETSTEP_LOCK_WHEN_DONE    (1u)

// Дефолтные калибровки-заглушки
// В Init() копируются в поля структуры:
// dev->cal.acc_bias_g[0..2], dev->cal.acc_scale[0..2], dev->cal.gyro_bias_rads[0..2]
#define ICM20948_ACC_BIAS_X_G_DEFAULT          (-0.0041218996f)  // -> dev->cal.acc_bias_g[0]
#define ICM20948_ACC_BIAS_Y_G_DEFAULT          (-0.0117365122f)  // -> dev->cal.acc_bias_g[1]
#define ICM20948_ACC_BIAS_Z_G_DEFAULT          (0.00749456882f)  // -> dev->cal.acc_bias_g[2]

#define ICM20948_ACC_SCALE_X_DEFAULT           (0.998735726f)  // -> dev->cal.acc_scale[0]
#define ICM20948_ACC_SCALE_Y_DEFAULT           (0.99713397f)  // -> dev->cal.acc_scale[1]
#define ICM20948_ACC_SCALE_Z_DEFAULT           (0.991317391f)  // -> dev->cal.acc_scale[2]

#define ICM20948_GYRO_BIAS_X_RADS_DEFAULT      (0.0f)  // -> dev->cal.gyro_bias_rads[0]
#define ICM20948_GYRO_BIAS_Y_RADS_DEFAULT      (0.0f)  // -> dev->cal.gyro_bias_rads[1]
#define ICM20948_GYRO_BIAS_Z_RADS_DEFAULT      (0.0f)  // -> dev->cal.gyro_bias_rads[2]

// Банки регистров
#define ICM20948_REG_BANK_SEL                  (0x7Fu)
#define ICM20948_BANK_0                        (0x00u)
#define ICM20948_BANK_2                        (0x20u)

// Bank 0 регистры
#define ICM20948_REG_WHO_AM_I                  (0x00u)
#define ICM20948_WHO_AM_I_VALUE                (0xEAu)

#define ICM20948_REG_USER_CTRL                 (0x03u)
#define ICM20948_REG_PWR_MGMT_1                (0x06u)
#define ICM20948_REG_PWR_MGMT_2                (0x07u)
#define ICM20948_REG_INT_PIN_CFG               (0x0Fu)
#define ICM20948_REG_INT_ENABLE                (0x10u)
#define ICM20948_REG_INT_ENABLE_1              (0x11u)

#define ICM20948_REG_INT_STATUS_1              (0x1Au)
#define ICM20948_REG_ACCEL_XOUT_H              (0x2Du)

// Bank 2 регистры
#define ICM20948_REG_GYRO_SMPLRT_DIV           (0x00u)
#define ICM20948_REG_GYRO_CONFIG_1             (0x01u)
#define ICM20948_REG_ODR_ALIGN_EN             (0x09u)

#define ICM20948_REG_ACCEL_SMPLRT_DIV_1        (0x10u)
#define ICM20948_REG_ACCEL_SMPLRT_DIV_2        (0x11u)
#define ICM20948_REG_ACCEL_CONFIG              (0x14u)

// Биты
#define ICM20948_SPI_READ_BIT                  (0x80u)

#define ICM20948_PWR1_DEVICE_RESET             (0x80u)
#define ICM20948_PWR1_SLEEP                    (0x40u)
#define ICM20948_PWR1_CLKSEL_AUTO              (0x01u)

#define ICM20948_INT1_RAW_RDY_EN               (0x01u)
#define ICM20948_USER_CTRL_I2C_IF_DIS          (0x10u)

// Статусы
typedef enum
{
    ICM20948_OK = 0,
    ICM20948_NO_NEW_SAMPLE,
    ICM20948_BUSY,

    ICM20948_ERR_NULL_PTR,
    ICM20948_ERR_PARAM,
    ICM20948_ERR_SPI,
    ICM20948_ERR_DMA,

    ICM20948_ERR_DWT_NOT_RUNNING,
    ICM20948_ERR_DEVICE_ID

} ICM20948_Status_t;

// Шаги калибровки
typedef enum
{
    ICM20948_CAL_STEP_POS_X = 0,
    ICM20948_CAL_STEP_NEG_X = 1,
    ICM20948_CAL_STEP_POS_Y = 2,
    ICM20948_CAL_STEP_NEG_Y = 3,
    ICM20948_CAL_STEP_POS_Z = 4,
    ICM20948_CAL_STEP_NEG_Z = 5,

    ICM20948_CAL_STEP_COUNT = 6,
    ICM20948_CAL_STEP_NONE = 0xFF

} ICM20948_CalStep_t;

// Состояния
typedef enum
{
    ICM20948_STATE_UNINIT = 0,
    ICM20948_STATE_IDLE,
    ICM20948_STATE_DMA_ACTIVE

} ICM20948_State_t;

// Состояние калибровки
typedef enum
{
    ICM20948_CAL_STATE_IDLE = 0,
    ICM20948_CAL_STATE_ACCUM,
    ICM20948_CAL_STATE_DONE

} ICM20948_CalState_t;

// Причина сброса шага
typedef enum
{
    ICM20948_CAL_RESET_NONE = 0,
    ICM20948_CAL_RESET_GYRO_MOVING,
    ICM20948_CAL_RESET_ACCEL_DELTA,
    ICM20948_CAL_RESET_BAD_GRAVITY

} ICM20948_CalResetReason_t;

// Параметры инициализации
typedef struct
{
    SPI_HandleTypeDef* hspi;
    DMA_HandleTypeDef* hdma_rx;
    DMA_HandleTypeDef* hdma_tx;

    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;

    GPIO_TypeDef* int_port;
    uint16_t int_pin;

    uint32_t spi_timeout_ms;

} ICM20948_InitParams_t;

// Типы данных
typedef struct
{
    uint64_t timestamp_us;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];

} ICM20948_RawSample_t;

typedef struct
{
    uint64_t timestamp_us;
    float accel_mps2[3];
    float gyro_rads[3];

} ICM20948_PhysSample_t;

typedef struct
{
    float acc_bias_g[3];
    float acc_scale[3];
    float gyro_bias_rads[3];

} ICM20948_Calibration_t;

typedef struct
{
    uint8_t done_mask;
    uint8_t cal_done;

    uint8_t active_step;
    uint16_t progress;

    uint8_t retry_count;
    uint8_t last_auto_step;

    uint8_t last_reset_reason;

} ICM20948_CalStatus_t;

typedef struct
{
    uint8_t valid;
    float acc_bias_g[3];
    float acc_scale[3];

} ICM20948_CalResult_t;

typedef struct
{
    uint8_t reg_bank_sel;
    uint8_t user_ctrl;
    uint8_t int_pin_cfg;
    uint8_t int_enable_1;
    uint8_t int_status_1;

    uint8_t gyro_smplrt_div;
    uint8_t gyro_config_1;
    uint8_t odr_align_en;

    uint8_t accel_smplrt_div_1;
    uint8_t accel_smplrt_div_2;
    uint8_t accel_config;

} ICM20948_DebugRegs_t;

// Главная структура драйвера
typedef struct
{
    struct
    {
        SPI_HandleTypeDef* hspi;
        DMA_HandleTypeDef* hdma_rx;
        DMA_HandleTypeDef* hdma_tx;

        GPIO_TypeDef* cs_port;
        uint16_t cs_pin;

        GPIO_TypeDef* int_port;
        uint16_t int_pin;

        uint32_t spi_timeout_ms;

    } cfg;

    ICM20948_State_t state;
    volatile uint8_t spi_busy;
    volatile uint8_t new_sample;

    volatile uint32_t dwt_last;
    volatile uint32_t dwt_hi;

    uint8_t tx_buf[ICM20948_BURST_TOTAL_BYTES];
    uint8_t rx_buf[ICM20948_BURST_TOTAL_BYTES];

    ICM20948_RawSample_t raw;
    uint8_t raw_valid;

    ICM20948_Calibration_t cal;

    struct
    {
        ICM20948_CalState_t state;
        uint8_t done_mask;
        uint8_t retry_count;
        ICM20948_CalStep_t active_step;
        ICM20948_CalStep_t last_auto_step;
        ICM20948_CalResetReason_t last_reset_reason;

        uint16_t progress;

        double sum_ax_g;
        double sum_ay_g;
        double sum_az_g;

        float mean_step_g[ICM20948_CAL_STEP_COUNT][3];

        float last_accel_g[3];
        uint8_t last_accel_valid;

        uint8_t gyro_cal_done;
        uint8_t gyro_cal_started;
        uint8_t gyro_cal_active;
        ICM20948_CalStep_t gyro_cal_step;
        uint16_t gyro_cal_progress;
        double gyro_sum_rads[3];

    } cal6;

    // Коэффициенты пересчета (зависят от FS)
    float accel_lsb_per_g;
    float gyro_lsb_per_dps;

    // Оценка реальной частоты прихода данных акселерометра
    uint64_t real_samples_prev_timestamp_us;
    float real_samples_hz;
    float real_samples_period_us;

    // Диагностическое чтение-back ключевых регистров
    ICM20948_DebugRegs_t dbg;

    volatile uint8_t cal_start;   // поставить 1 чтобы начать калибровку
    volatile uint8_t cal_step;   // поставить 1 когда датчик в нужном положении
    volatile uint8_t cal_reset;   // сбросить калибровку
    uint8_t gyro_cal_start;

    ICM20948_InitParams_t p;


} ICM20948_t;

// Публичный API
// Инициализация (SPI и DMA должны быть настроены снаружи, например CubeMX; драйвер использует переданные handle)
ICM20948_Status_t ICM20948_Init(ICM20948_t* dev);

// Обработка INT (вызывать из HAL_GPIO_EXTI_Callback)
ICM20948_Status_t ICM20948_IrqHandler(ICM20948_t* dev, uint16_t gpio_pin);

// Обработка завершения SPI DMA (вызывать из HAL_SPI_TxRxCpltCallback)
ICM20948_Status_t ICM20948_SpiDmaCpltHandler(ICM20948_t* dev, SPI_HandleTypeDef* hspi);

// Сервис (вызывать в main loop)
ICM20948_Status_t ICM20948_Service(ICM20948_t* dev);

// Данные
ICM20948_Status_t ICM20948_GetLatestRaw(ICM20948_t* dev, ICM20948_RawSample_t* out);
ICM20948_Status_t ICM20948_GetLatestPhys(ICM20948_t* dev, ICM20948_PhysSample_t* out);

// Вычисление и хранение реальной частоты прихода данных акселерометра
ICM20948_Status_t ICM20948_RealSamples(ICM20948_t* dev);

// Диагностика регистров
ICM20948_Status_t ICM20948_ReadDebugRegs(ICM20948_t* dev);

// Калибровка
ICM20948_Status_t ICM20948_CalGyro(ICM20948_t* dev);
ICM20948_Status_t ICM20948_CalReset(ICM20948_t* dev);
ICM20948_Status_t ICM20948_CalSetStep(ICM20948_t* dev);
ICM20948_Status_t ICM20948_CalGetStatus(ICM20948_t* dev, ICM20948_CalStatus_t* out);
ICM20948_Status_t ICM20948_CalGetResult(ICM20948_t* dev, ICM20948_CalResult_t* out);

uint64_t icm_micros64(ICM20948_t* dev);

#ifdef __cplusplus
}
#endif

#endif
