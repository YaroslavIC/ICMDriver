# ICM20948 Driver — summary for transfer to a new chat

This file summarizes the agreed architecture, behavior, API, calibration logic, and debugging conclusions for the current ICM20948 driver line.  
Baseline for future edits: **`icm20948_driver_1_1_0.zip`**.

## 1. Project context

Target:
- STM32F401CCU6
- ICM-20948 over SPI
- INT pin connected to MCU EXTI
- magnetometer is **not used**
- temperature is **not used**
- data is needed for a balancing robot

Main design rules agreed during development:
- all data and parameters are passed through structures
- public functions return status only
- no dynamic memory
- code comments only with `//`
- driver version must be incremented on **every code change**, even minimal
- current file `icm20948_driver_1_1_0.zip` is the base version for future modifications

## 2. Hardware assumptions

Current known connection set used during debugging:
- **SPI1**
- **INT -> PA3**
- **CS -> PA4**

Important:
- SPI GPIO alternate-function pins are configured **outside** the driver, normally by CubeMX
- EXTI for INT pin is configured **outside** the driver
- driver handles CS as normal GPIO output

## 3. High-level data path

Working chain:
- ICM20948 asserts **INT1**
- STM32 gets EXTI interrupt
- EXTI callback calls driver IRQ handler
- driver starts SPI DMA burst read
- DMA complete callback calls driver DMA-complete handler
- handler parses accel/gyro raw data
- latest sample becomes available to application

Confirmed working during debugging:
- `WHO_AM_I` reads correctly
- INT pin works
- DMA transfer works
- raw data updates correctly
- physical values from `GetLatestPhys()` are valid

## 4. Timestamping

Driver uses:
- `DWT->CYCCNT`
- software extension to 64-bit
- timestamp stored in microseconds

Important conclusions:
- timestamp conversion depends on `SystemCoreClock`
- after `SystemClock_Config()` it is required that runtime clock values are correct
- later debugging confirmed `SystemCoreClock = 84000000` and DWT conversion was correct

## 5. Sample rate / INT behavior

Important final conclusion from debugging:
- `real_samples_hz` measures the **real interrupt/data-ready rate**
- after proper read-back diagnostics, the register writes were confirmed correct
- measured real rate matched register configuration

Observed and verified:
- with sample-rate divider `div = 3`, measured rate was about **281 Hz**
- with sample-rate divider `div = 0`, measured rate was about **1125 Hz**

Final mathematical conclusion:
- actual formula in the configured filtered mode is:
  - `ODR = 1125 / (1 + div)`

So:
- `div = 0` -> about **1125 Hz**
- `div = 3` -> about **281.25 Hz**

## 6. Interrupt configuration conclusions

A major earlier bug was fixed by configuring interrupt registers according to the datasheet/register map.

Important points learned:
- in SPI mode, `USER_CTRL.I2C_IF_DIS = 1` must be set
- data-ready interrupt is enabled through **`INT_ENABLE_1`**
- data-ready flag is in **`INT_STATUS_1`**
- interrupt pin mode can be configured pulse/latched
- the version that finally worked used proper interrupt register setup and read-back verification

## 7. DLPF / ODR configuration conclusions

Another important debugging stage:
- writing only sample-rate dividers was not enough
- DLPF/FCHOICE/ODR alignment had to be configured explicitly
- read-back of configuration registers was added to verify actual register values

Verified useful register set:
- `GYRO_SMPLRT_DIV`
- `GYRO_CONFIG_1`
- `ODR_ALIGN_EN`
- `ACCEL_SMPLRT_DIV_1`
- `ACCEL_SMPLRT_DIV_2`
- `ACCEL_CONFIG`

When correctly configured and read back:
- `gyro_smplrt_div = 0` and `accel_smplrt_div = 0` gave about **1125 Hz**
- `gyro_smplrt_div = 3` and `accel_smplrt_div = 3` gave about **281 Hz**

## 8. Units and data conventions

Output units agreed for physical values:
- accelerometer: **m/s²**
- gyroscope: **rad/s**

Raw sample:
- `int16_t accel_raw[3]`
- `int16_t gyro_raw[3]`
- timestamp in microseconds

Physical sample:
- `float accel_mps2[3]`
- `float gyro_rads[3]`
- same timestamp

Axes:
- all orientation/correction logic refers to **ICM20948 chip axes**, not board-level abstract axes

## 9. Public API that was defined during design

Main public functions discussed and used:

### Initialization / runtime
- `ICM20948_Init(...)`
- `ICM20948_IrqHandler(...)`
- `ICM20948_SpiDmaCpltHandler(...)`
- `ICM20948_Service(...)`

### Data access
- `ICM20948_GetLatestRaw(...)`
- `ICM20948_GetLatestPhys(...)`
- `ICM20948_RealSamples(...)`

### Accelerometer calibration
- `ICM20948_CalReset(...)`
- `ICM20948_CalSetStep(...)`
- `ICM20948_CalGetStatus(...)`
- `ICM20948_CalGetResult(...)`

### Gyroscope calibration
- `ICM20948_CalGyro(...)`

### Diagnostics
- `ICM20948_ReadDebugRegs(...)`

## 10. Callbacks expected from main.c

Expected glue from application:

### EXTI callback
Application callback should call:
- `ICM20948_IrqHandler(&imu, GPIO_Pin);`

### SPI DMA complete callback
Application callback should call driver only for the correct SPI instance/handle:
- typically check `hspi == &hspi1`
- then call `ICM20948_SpiDmaCpltHandler(&imu, hspi);`

## 11. Accelerometer calibration logic

Implemented logic was a **6-step calibration**.

### Orientation handling
Driver auto-detects current step from accelerometer:
- `+X`
- `-X`
- `+Y`
- `-Y`
- `+Z`
- `-Z`

Order:
- **arbitrary**
- all six positions must be collected

### Sampling per step
Agreed default:
- **2048 samples** per step

### Step overwrite behavior
Agreed:
- if a step is repeated, that step is overwritten

### When all 6 steps are ready
Driver computes:
- `acc_bias_g[3]`
- `acc_scale[3]`

### Acceptance rule for gravity
For accepted orientation data:
- dominant axis mean must be close to ±1 g
- range used during design:
  - `0.8 g .. 1.2 g`

### Retry rule
Agreed:
- up to 2 attempts
- if still not good, use the last valid data

### Motion rejection
Final agreed simplification:
- **gyro-based motion rejection was removed**
- calibration stability check uses only **accelerometer delta**
- this was necessary because gyro offset caused false resets

## 12. Gyroscope calibration logic

Important final decision:
- gyroscope calibration should be a **separate explicit function**
- not implicitly coupled to the first accelerometer step

Current intended use:
1. place the IMU at rest
2. wait until raw data is already arriving
3. call:
   - `ICM20948_CalGyro(&imu);`
4. function averages gyro raw samples
5. stores result to:
   - `dev->cal.gyro_bias_rads[0..2]`

Important practical note:
- `ICM20948_CalGyro()` must be called only **after valid data starts arriving**
- during one debug step it was called too early and failed because `raw_valid == 0`

## 13. Accelerometer default calibration values agreed by user

The following accelerometer defaults were explicitly provided by the user and inserted into the driver line at that time:

```c
#define ICM20948_ACC_BIAS_X_G_DEFAULT   (-0.0041218996f)
#define ICM20948_ACC_BIAS_Y_G_DEFAULT   (-0.0117365122f)
#define ICM20948_ACC_BIAS_Z_G_DEFAULT   ( 0.00749456882f)

#define ICM20948_ACC_SCALE_X_DEFAULT    (0.998735726f)
#define ICM20948_ACC_SCALE_Y_DEFAULT    (0.99713397f)
#define ICM20948_ACC_SCALE_Z_DEFAULT    (0.991317391f)
```

These were intended to be copied into the driver's default calibration defines.

## 14. Debugging helpers that proved useful

Useful runtime fields observed during debugging:
- `imu.real_samples_hz`
- `imu.real_samples_period_us`
- `imu.raw_valid`
- `imu.raw.timestamp_us`

Calibration status fields:
- `imu.cal6.progress`
- `imu.cal6.done_mask`
- `imu.cal6.last_auto_step`
- `imu.cal6.last_reset_reason`

Register read-back diagnostics:
- `imu.dbg.reg_bank_sel`
- `imu.dbg.user_ctrl`
- `imu.dbg.int_pin_cfg`
- `imu.dbg.int_enable_1`
- `imu.dbg.int_status_1`
- `imu.dbg.gyro_smplrt_div`
- `imu.dbg.gyro_config_1`
- `imu.dbg.odr_align_en`
- `imu.dbg.accel_smplrt_div_1`
- `imu.dbg.accel_smplrt_div_2`
- `imu.dbg.accel_config`

These read-back diagnostics were important to prove that the device actually accepted the intended configuration.

## 15. Known issues / lessons learned from this development

### 15.1 INT pin issues
At one stage:
- SPI/DMA worked
- data changed on manual read
- but INT pin stayed silent

This was eventually fixed by:
- correct interrupt register usage
- disabling I2C interface in SPI mode
- enabling the proper INT source
- reading/clearing the proper interrupt status register

### 15.2 Real sample rate confusion
At another stage:
- measured rate looked wrong by ~4x
- this turned out not to be MCU clock error
- after read-back it was confirmed that the measured rate actually matched the configured divider

### 15.3 Gyro motion rejection caused false calibration resets
The user explicitly requested:
- remove gyro usage for calibration-motion control

Final result:
- accel calibration uses only accel-delta motion check

### 15.4 Some generated intermediate archives contained inconsistent code
During iterative editing, a few intermediate archives had:
- undeclared local variables
- removed declarations with leftover calculations
- mismatched version/content

The current baseline for future work is explicitly:
- **`icm20948_driver_1_1_0.zip`**

Use this as the clean base for the next modification cycle.

## 16. Recommended order of use in firmware

Typical intended flow:

1. `ICM20948_Init(&imu, &params);`
2. wait until data is flowing (`raw_valid == 1`)
3. call `ICM20948_CalGyro(&imu);`
4. reset/start accel calibration:
   - `ICM20948_CalReset(&imu);`
5. perform 6 positions:
   - call `ICM20948_CalSetStep(&imu);` once per stable orientation
6. when `cal_done` is true, driver uses the calculated accel correction
7. in runtime loop:
   - handle service
   - read latest physical values
   - use `imu.real_samples_hz` for confirmation of real data rate

## 17. Instruction for the next chat

If starting a new chat, use this exact statement:

> Current base driver version is `icm20948_driver_1_1_0.zip`.  
> Continue modifying this version only.  
> Keep version increments on every code change.  
> Magnetometer is not used.  
> SPI + INT + DMA are used.  
> INT/ODR/debug conclusions are documented in `ICM20948_DRIVER_CHAT_SUMMARY.md`.
