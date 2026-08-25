/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdbool.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FAULT_STATUS1_REG_ADDR   0x00U
#define VGS_STATUS2_REG_ADDR     0x01U
#define DRIVER_CONTROL_REG_ADDR  0x02U
#define GATE_DRIVE_HS_REG_ADDR   0x03U
#define GATE_DRIVE_LS_REG_ADDR   0x04U
#define OCP_CONTROL_REG_ADDR     0x05U
#define CSA_CONTROL_REG_ADDR     0x06U
#define DRIVER_CALIBRATION_REG_ADDR 0x07U

#define DRV_CTRL_OCP_ACT         (1U << 10)
#define DRV_CTRL_CLR_FLT         (1U << 0)
#define DRV_CTRL_BRAKE           (1U << 1)
#define DRV_CTRL_COAST           (1U << 2)
#define DRV_CTRL_PWM_MODE_MASK   (3U << 5)
#define DRV_CTRL_PWM_MODE_6X     (0U << 5)

#define ADC_VREF                 3.3f
#define CURRENT_SHUNT_OHMS       0.001f
#define CURRENT_AMP_GAIN_V_PER_V 20.0f
#define CURRENT_GAIN             (CURRENT_SHUNT_OHMS * CURRENT_AMP_GAIN_V_PER_V)
#define CURRENT_ADC_OFFSET_V     (ADC_VREF * 0.5f)
#define CURRENT_OFFSET_CAL_SAMPLES 512U
#define CURRENT_OFFSET_CALIBRATION_ENABLE 1U
#define CURRENT_FIXED_OFFSET_MV 1650U
#define CURRENT_OFFSET_DISCARD_SAMPLES 16U
#define CURRENT_OFFSET_VALID_MIN_ADC 512U
#define CURRENT_OFFSET_VALID_MAX_ADC 3583U
#define CURRENT_SAMPLE_BEFORE_TIM1_PEAK_TICKS 80U
#define CURRENT_MIN_VALID_WINDOW_TICKS 150U
#define CURRENT_ADC_FULL_SCALE 4095.0f
#define CURRENT_ADC_SATURATION_COUNTS 8U
#define CURRENT_ADC_SAMPLE_CYCLES 12.5f
#define CURRENT_ADC_CONVERSION_CYCLES 12.5f
#define CURRENT_DIAGNOSTIC_FILTER_ALPHA (1.0f / 16.0f)
#define CURRENT_SENSE_SAMPLE_DEBUG_GPIO 0U
#define CURRENT_SENSE_MOVING_AVERAGE_SAMPLES 8U
#define CURRENT_SENSE_SAMPLE_DELAY_US 10U
#define CURRENT_SENSE_DEBUG_PRINT 1U
#define MOTOR_PHASE_CURRENT_LIMIT_ENABLE 0U
#define MOTOR_PHASE_CURRENT_LIMIT_A 8.0f
#define CURRENT_LED_DIAGNOSTIC_ENABLE 0U
#define CURRENT_LED_FULL_SCALE_A 60.0f
#define CURRENT_LED_UPDATE_MS 1000U
#define DRV_FAULT_POLL_ENABLE 0U
#define DRV_SPI_CONTINUOUS_READ_TEST 0U
#define DRV_SPI_BITBANG_GPIO_TEST 1U
#define DRV_SPI_CONFIGURATION_ENABLE 0U
#define DRV_SPI_CSA_CALIBRATION_TRIAL 1U
#define LED_FLASH_TEST_ENABLE 0U
#define SVPWM_CURRENT_LOG_ENABLE 1U
#define UART_OUTPUT_ENABLE 1U
#define MOTOR_SVPWM_CONTINUOUS_RUN 0U
#define MOTOR_FIELD_STEP_DEMO_ENABLE 0U
#define MOTOR_FOC_DEMO_ENABLE        1U
#define MOTOR_ENCODER_TEST_ENABLE    0U
#define FOC_POSITION_DEMO_ENABLE     1U
#define FOC_CURRENT_STEP_TEST_ENABLE 0U
#define FOC_LOW_SPEED_VELOCITY_TEST_ENABLE 0U

#define AS5048A_CS_Pin             GPIO_PIN_15
#define AS5048A_CS_GPIO_Port       GPIOA
#define AS5048A_REG_NOP            0x0000U
#define AS5048A_REG_CLEAR_ERROR    0x0001U
#define AS5048A_REG_DIAGNOSTICS    0x3FFDU
#define AS5048A_REG_MAGNITUDE      0x3FFEU
#define AS5048A_REG_ANGLE          0x3FFFU

/* PCB data lines are crossed: PA6 reaches DRV SDI; PA7 reaches DRV SDO. */
#define DRV_BB_SDI_Pin             DRV_MISO_Pin
#define DRV_BB_SDI_GPIO_Port       DRV_MISO_GPIO_Port
#define DRV_BB_SDO_Pin             DRV_MOSI_Pin
#define DRV_BB_SDO_GPIO_Port       DRV_MOSI_GPIO_Port
#define DRV_BB_EDGE_DELAY_US        5U

#define MOTOR_POLE_PAIRS         14U
#define MOTOR_DUTY_TICKS         1200U
#define DESIRED_MOTOR_DUTY       0.6f
#define MOTOR_START_ELECTRICAL_HZ 0.5f
#define MOTOR_HIGH_ELECTRICAL_HZ 500.0f
#define MOTOR_LOW_ELECTRICAL_HZ  10.0f
#define MOTOR_RAMP_EHZ_PER_SEC   2.0f
#define MOTOR_ALIGNMENT_HOLD_MS  1000U
#define MOTOR_ALIGNMENT_ANGLE_RAD 0.0f
#define MOTOR_DEBUG_PRINT_MS     10000000U
#define MOTOR_SVPWM_UPDATE_US    50U
#define MOTOR_SVPWM_RUN_TIME_MS  50000U
#define SVPWM_CAPTURE_START_MS   5000U
#define SVPWM_LOG_INTERVAL_MS    1U
#define SVPWM_LOG_CAPACITY       500U
#define MOTOR_SVPWM_MAX_INDEX    0.57735026919f
#define MOTOR_OPEN_LOOP_MODULATION_START 0.020f
#define MOTOR_OPEN_LOOP_MODULATION_MID   0.040f
#define MOTOR_OPEN_LOOP_MODULATION_END   0.100f
#define MOTOR_OPEN_LOOP_MODULATION_STAGE1_MS 4000U
#define MOTOR_FIELD_STEP_MODULATION      0.020f
#define MOTOR_FIELD_STEP_HOLD_MS         2000U
#define MOTOR_FIELD_STEPS_PER_ELECTRICAL_REV 6U
#define FOC_ALIGNMENT_MODULATION          0.030f
#define FOC_ALIGNMENT_HOLD_MS             1000U
#define FOC_ALIGNMENT_CURRENT_AVG_SAMPLES   32U
#define FOC_VERIFIED_ENCODER_DIRECTION         1
#define FOC_VERIFIED_CURRENT_POLARITY         1
#define FOC_IQ_TARGET_A                   4.0f
#define FOC_ID_TARGET_A                   0.0f
#define FOC_CURRENT_KP                    0.0030f
#define FOC_CURRENT_KI                    1.0f
#define FOC_CURRENT_LOOP_HZ              20000U
#define FOC_CURRENT_LOOP_DT_S             (1.0f / (float)FOC_CURRENT_LOOP_HZ)
#define FOC_TIM1_PERIOD_TICKS              2799U
#define FOC_SINE_LUT_SIZE                  1024U
#define FOC_SINE_LUT_MASK                  (FOC_SINE_LUT_SIZE - 1U)
#define FOC_MAX_MODULATION                0.470f
#define FOC_HARD_CURRENT_LIMIT_A          50.0f
#define FOC_DQ_FAULT_LIMIT_A              25.0f
#define FOC_OVERCURRENT_CONFIRM_SAMPLES       1U
#define FOC_MAX_MECHANICAL_RPM           5200.0f
#define FOC_SPEED_TARGET_RPM              4800.0f
#define FOC_SPEED_REFERENCE_RAMP_RPM_S    1500.0f
#define FOC_SPEED_KP_A_PER_RPM             0.004f
#define FOC_SPEED_KI_A_PER_RPM_S           0.030f
#define FOC_LOW_SPEED_TARGET_RPM           10.0f
#define FOC_LOW_SPEED_KP_A_PER_RPM          0.025f
#define FOC_LOW_SPEED_KI_A_PER_RPM_S        0.500f
#define FOC_LOW_SPEED_INTEGRAL_LIMIT_A      9.500f
#define FOC_LOW_SPEED_IQ_LIMIT_A           10.000f
#define FOC_LOW_SPEED_OVERSPEED_RPM       100.0f
#define FOC_LOW_SPEED_TEST_DURATION_MS    1600U
#define FOC_LOW_SPEED_LOG_INTERVAL_MS       10U
#define FOC_LOW_SPEED_MIN_CURRENT_SAMPLES  150U
#define FOC_CURRENT_STEP_TEST_DURATION_MS   100U
#define FOC_CURRENT_STEP_LOG_INTERVAL_MS      1U
#define FOC_CURRENT_STEP_MIN_CURRENT_SAMPLES 15U
#define FOC_CURRENT_STEP_1_START_MS          10U
#define FOC_CURRENT_STEP_2_START_MS          35U
#define FOC_CURRENT_STEP_ZERO_START_MS       70U
#define FOC_CURRENT_STEP_1_A                  1.0f
#define FOC_CURRENT_STEP_2_A                  2.0f
#define FOC_CURRENT_STEP_IQ_LIMIT_A           2.0f
#define FOC_CURRENT_STEP_OVERSPEED_RPM      100.0f
#define FOC_CURRENT_STEP_HARD_CURRENT_LIMIT_A 20.0f
#define FOC_CURRENT_STEP_DQ_FAULT_LIMIT_A    15.0f
#define FOC_POSITION_MAX_SPEED_RPM         100.0f
#define FOC_POSITION_KP_RPM_PER_DEG         2.0f
#define FOC_POSITION_SPEED_KP_A_PER_RPM     0.025f
#define FOC_POSITION_BREAKAWAY_CURRENT_A    2.0f
#define FOC_POSITION_BREAKAWAY_MAX_MS        50U
/* Motor-side AS5048A revolutions per output-shaft revolution.  For an
 * 11:1 reduction, an output move of 360 degrees commands 11 motor turns. */
#define FOC_MOTOR_TO_OUTPUT_GEAR_RATIO       11.0f
#define FOC_OUTPUT_DIRECTION_SIGN             1.0f
/* Position targets, trajectory limits, and outer PD gains are expressed at
 * the gearbox output. The motor reference is multiplied by the ratio only at
 * the boundary to the motor-side encoder/current FOC. */
#define FOC_POSITION_KP_A_PER_OUTPUT_DEG     0.450f
#define FOC_POSITION_KD_A_PER_OUTPUT_RPM     0.156f
#define FOC_POSITION_TRAJECTORY_MAX_OUTPUT_RPM      200.0f
#define FOC_POSITION_TRAJECTORY_ACCEL_OUTPUT_RPM_S  80.0f
#define FOC_POSITION_TRAJECTORY_DECEL_OUTPUT_RPM_S  80.0f
#define FOC_POSITION_OVERSPEED_RPM          0.0f
#define FOC_POSITION_IQ_LIMIT_A            10.0f
#define FOC_POSITION_HARD_CURRENT_LIMIT_A  50.0f
#define FOC_POSITION_DQ_FAULT_LIMIT_A       25.0f
#define FOC_OUTPUT_POSITION_TOLERANCE_DEG    3.0f
#define FOC_OUTPUT_SPEED_TOLERANCE_RPM       3.0f
#define FOC_POSITION_HOLD_MS              500U
#define FOC_POSITION_STEP_TIMEOUT_MS      4000U
#define FOC_POSITION_TEST_DURATION_MS    25000U
#define FOC_TEST_DURATION_MS             12000U
#define FOC_ENCODER_UPDATE_US             125U
#define FOC_ENCODER_SENSOR_DELAY_US        200U
#define FOC_ENCODER_PREDICTION_MAX_US     1200U
#define FOC_ENCODER_MAX_INNOVATION_RAD    3.141593f
#define FOC_ENCODER_CORRECTION_GAIN       0.200f
#define FOC_ENCODER_MAX_CORRECTION_RAD    0.020f
#define FOC_ENCODER_VELOCITY_GAIN         0.020f
#define FOC_ENCODER_MAX_VELOCITY_STEP_RAD_S 200.0f
#define FOC_ENCODER_MAX_CONSECUTIVE_REJECTIONS 10U
#if FOC_LOW_SPEED_VELOCITY_TEST_ENABLE
#define FOC_SPEED_WINDOW_US              20000U
#define FOC_SPEED_FILTER_ALPHA              0.250f
#else
#define FOC_SPEED_WINDOW_US               5000U
#define FOC_SPEED_FILTER_ALPHA              0.500f
#endif
#define FOC_ENCODER_MAX_CONSECUTIVE_ERRORS 10U
#define FOC_LOG_INTERVAL_MS                60U
#define FOC_POSITION_LOG_INTERVAL_MS      200U
#if FOC_CURRENT_STEP_TEST_ENABLE
#define FOC_LOG_CAPACITY                  105U
#elif FOC_POSITION_DEMO_ENABLE
#define FOC_LOG_CAPACITY                  130U
#elif FOC_LOW_SPEED_VELOCITY_TEST_ENABLE
#define FOC_LOG_CAPACITY                  160U
#else
#define FOC_LOG_CAPACITY                  202U
#endif
#define FOC_MIN_CURRENT_SAMPLES_PER_LOG   900U
#define FOC_POSITION_MIN_CURRENT_SAMPLES_PER_LOG 3000U
#define FOC_PREFAULT_CAPTURE_ENABLE         0U
#define FOC_PREFAULT_LOG_CAPACITY           32U
#define ENCODER_TEST_INTERVAL_MS           20U
#define ENCODER_TEST_PRINT_INTERVAL_MS     100U
#define MOTOR_DEADTIME_100NS_TICKS 11U
#define MOTOR_DEADTIME_500NS_TICKS 56U
#define MOTOR_DEADTIME_1US_TICKS   112U
#define MOTOR_TIM1_DEADTIME_TICKS  MOTOR_DEADTIME_500NS_TICKS
#define MOTOR_TEST_COOLDOWN_SEC    120U

#define TIM1_ALL_OUTPUTS         (TIM_CCER_CC1E | TIM_CCER_CC1NE | \
                                  TIM_CCER_CC2E | TIM_CCER_CC2NE | \
                                  TIM_CCER_CC3E | TIM_CCER_CC3NE)

#define DRV_GATE_LOCK_UNLOCK     (3U << 8)
#define DRV_IDRIVEP_100MA        (2U << 4)
#define DRV_IDRIVEN_200MA        (2U << 0)
#define DRV_TDRIVE_500NS         (0U << 8)
#define DRV_OCP_DEADTIME_200NS   (2U << 8)
#define DRV_OCP_MODE_LATCHED     (0U << 6)
#define DRV_OCP_DEG_4US          (2U << 4)
#define DRV_VDS_LVL_0600MV       (9U << 0)
#define DRV_CSA_VREF_DIV2        (1U << 9)
#define DRV_CSA_GAIN_40VV        (3U << 6)
#define DRV_CSA_SEN_LVL_025V     (0U << 0)
#define DRV_CSA_CAL_ALL           ((1U << 4) | (1U << 3) | (1U << 2))
#define DRV_CAL_MODE_MANUAL       (1U << 0)
#define DRV_CSA_CONFIG_MASK       ((1U << 10) | (1U << 9) | (1U << 8) | \
                                   (3U << 6) | (1U << 5) | (7U << 2) | (3U << 0))
#define DRV_CSA_DESIRED_CONFIG    (DRV_CSA_VREF_DIV2 | \
                                   DRV_CSA_GAIN_40VV | \
                                   DRV_CSA_SEN_LVL_025V)

#define PI_F                     3.14159265359f
#define TWO_PI_F                 6.28318530718f
#define SQRT3_F                  1.73205080757f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
typedef struct
{
  uint16_t time_ms;
  uint16_t electrical_hz_x10;
  uint16_t electrical_angle_mrad;
  uint16_t modulation_x10000;
  uint16_t phase_a_adc;
  uint16_t phase_b_adc;
  uint16_t phase_c_adc;
  uint8_t valid_a;
  uint8_t valid_b;
  uint8_t valid_c;
  int32_t phase_a_filtered_ma;
  int32_t phase_b_filtered_ma;
  int32_t phase_c_filtered_ma;
} SVPWM_LogSample;

typedef struct
{
  uint16_t time_ms;
  uint16_t raw_a;
  uint16_t raw_b;
  uint16_t raw_c;
  int32_t ia_ma;
  int32_t ib_ma;
  int32_t ib_sensed_ma;
  int32_t ib_sensed_corrected_ma;
  int32_t ic_ma;
  int32_t current_sum_ma;
  int32_t sensed_current_sum_ma;
  int32_t id_ma;
  int32_t iq_ma;
  int32_t electrical_angle_mrad;
  int32_t electrical_velocity_mrad_s;
  int32_t mechanical_rpm_x100;
  int32_t mechanical_rpm_window_x100;
  int32_t speed_reference_rpm_x100;
  int32_t speed_error_rpm_x100;
  int32_t mechanical_position_mdeg;
  int32_t output_position_mdeg;
  int32_t trajectory_position_mdeg;
  int32_t position_error_mdeg;
  int32_t position_target_mdeg;
  int32_t encoder_window_counts;
  int32_t encoder_innovation_mrad;
  int32_t encoder_correction_mrad;
  uint32_t encoder_age_us;
  int32_t id_reference_ma;
  int32_t iq_reference_ma;
  int32_t speed_integrator_ma;
  int32_t breakaway_current_ma;
  int32_t vd_x10000;
  int32_t vq_x10000;
  int32_t id_average_ma;
  int32_t iq_average_ma;
  int32_t phase_rms_ma;
  int32_t phase_peak_ma;
  uint32_t current_samples;
  uint32_t current_isr_max_cycles;
  uint8_t valid_a;
  uint8_t valid_b;
  uint8_t valid_c;
  uint8_t iq_saturated;
} FOC_LogSample;

#if FOC_PREFAULT_CAPTURE_ENABLE
typedef struct
{
  uint32_t sequence;
  uint16_t raw_a;
  uint16_t raw_b;
  uint16_t raw_c;
  uint16_t ccr_a;
  uint16_t ccr_b;
  uint16_t ccr_c;
  uint16_t ccr_trigger;
  float ia;
  float ib;
  float ib_sensed_corrected;
  float ic;
  float id;
  float iq;
  float electrical_angle_rad;
  float encoder_raw_angle_rad;
  float encoder_innovation_rad;
  float encoder_correction_rad;
  float encoder_velocity_rad_s;
  float vd_previous;
  float vq_previous;
  uint8_t valid_mask;
} FOC_PreFaultSample;
#endif

static uint8_t prev_high = 0;
static uint8_t prev_low = 0;
volatile float g_target_electrical_hz = MOTOR_START_ELECTRICAL_HZ;
volatile float g_actual_electrical_hz = MOTOR_START_ELECTRICAL_HZ;
volatile uint16_t g_pwm_duty_ticks = MOTOR_DUTY_TICKS;
volatile uint16_t g_drv_fault_status1 = 0U;
volatile uint16_t g_drv_vgs_status2 = 0U;
volatile uint16_t g_drv_driver_control = 0U;
volatile uint8_t g_current_limit_fault = 0U;
static SVPWM_LogSample g_svpwm_log[SVPWM_LOG_CAPACITY];
static uint16_t g_svpwm_log_count = 0U;
static uint32_t g_svpwm_log_start_tick = 0U;
static uint32_t g_svpwm_log_last_tick = 0U;
static uint16_t g_drv_csa_gain_v_per_v = 20U;
static float g_current_amps_per_adc_count =
    ADC_VREF / (CURRENT_ADC_FULL_SCALE * CURRENT_SHUNT_OHMS * 20.0f);
static uint16_t g_current_offset_adc[3] = {2048U, 2048U, 2048U};
static volatile PhaseCurrents_t g_phase_currents = {0};
static uint32_t g_current_sample_sync_timeouts = 0U;
static int32_t g_current_low_phase_buffer_ma[3][CURRENT_SENSE_MOVING_AVERAGE_SAMPLES] = {{0}};
static int32_t g_current_low_phase_sum_ma[3] = {0, 0, 0};
static uint8_t g_current_low_phase_index[3] = {0, 0, 0};
static uint16_t g_current_low_phase_count[3] = {0, 0, 0};
static float g_open_loop_modulation = MOTOR_OPEN_LOOP_MODULATION_START;
static uint8_t g_motor_deadtime_ticks = MOTOR_TIM1_DEADTIME_TICKS;
static volatile uint16_t g_drv_spi_last_tx = 0U;
static volatile uint16_t g_drv_spi_last_rx = 0U;
static volatile uint8_t g_foc_enabled = 0U;
static volatile uint8_t g_foc_fault = 0U;
static volatile float g_foc_electrical_angle_rad = 0.0f;
static volatile float g_foc_id_reference_a = 0.0f;
static volatile float g_foc_iq_reference_a = 0.0f;
static volatile float g_foc_id_a = 0.0f;
static volatile float g_foc_iq_a = 0.0f;
static volatile float g_foc_vd_modulation = 0.0f;
static volatile float g_foc_vq_modulation = 0.0f;
static volatile float g_foc_encoder_observed_angle_rad = 0.0f;
static volatile float g_foc_electrical_velocity_rad_s = 0.0f;
#if FOC_PREFAULT_CAPTURE_ENABLE
static volatile float g_foc_encoder_raw_angle_rad = 0.0f;
static volatile float g_foc_encoder_innovation_rad = 0.0f;
static volatile float g_foc_encoder_correction_rad = 0.0f;
#endif
static volatile uint32_t g_foc_encoder_observation_cycles = 0U;
static volatile uint32_t g_foc_encoder_observation_sequence = 0U;
static float g_foc_id_integrator = 0.0f;
static float g_foc_iq_integrator = 0.0f;
static uint16_t g_foc_encoder_zero_count = 0U;
static int8_t g_foc_encoder_direction = FOC_VERIFIED_ENCODER_DIRECTION;
static int8_t g_foc_current_polarity = 1;
static volatile float g_foc_b_pwm_bias_a = 0.0f;
static volatile uint8_t g_foc_b_bias_correction_enabled = 0U;
static volatile float g_foc_speed_reference_rpm = 0.0f;
static volatile float g_foc_active_hard_current_limit_a =
    FOC_HARD_CURRENT_LIMIT_A;
static volatile float g_foc_active_dq_fault_limit_a =
    FOC_DQ_FAULT_LIMIT_A;
static int32_t g_foc_mechanical_position_counts = 0;
static float g_foc_output_position_target_deg = 0.0f;
static float g_foc_output_trajectory_position_deg = 0.0f;
static float g_foc_debug_mechanical_rpm_window = 0.0f;
static float g_foc_debug_speed_error_rpm = 0.0f;
static float g_foc_debug_position_error_deg = 0.0f;
static float g_foc_debug_encoder_innovation_rad = 0.0f;
static float g_foc_debug_encoder_correction_rad = 0.0f;
static int32_t g_foc_debug_encoder_window_counts = 0;
static uint8_t g_foc_debug_iq_saturated = 0U;
static float g_foc_debug_speed_integrator_a = 0.0f;
static float g_foc_debug_breakaway_current_a = 0.0f;
#if FOC_POSITION_DEMO_ENABLE
static const float g_foc_output_position_demo_targets_deg[] =
{
  0.0f, 90.0f, 180.0f, 270.0f, 360.0f,
  270.0f, 180.0f, 90.0f, 0.0f
};
#endif
static volatile float g_foc_telemetry_id_sum = 0.0f;
static volatile float g_foc_telemetry_iq_sum = 0.0f;
static volatile float g_foc_telemetry_phase_square_sum = 0.0f;
static volatile float g_foc_telemetry_phase_peak_a = 0.0f;
static volatile uint32_t g_foc_telemetry_sample_count = 0U;
static volatile uint32_t g_foc_current_isr_max_cycles = 0U;
static float g_foc_seconds_per_core_cycle = (1.0f / 112000000.0f);
static uint8_t g_foc_overcurrent_sample_count = 0U;
static float g_foc_fault_max_current_a = 0.0f;
static float g_foc_fault_ia_a = 0.0f;
static float g_foc_fault_ib_a = 0.0f;
static float g_foc_fault_ic_a = 0.0f;
static float g_foc_fault_id_a = 0.0f;
static float g_foc_fault_iq_a = 0.0f;
static float g_foc_fault_electrical_angle_rad = 0.0f;
static uint8_t g_foc_fault_valid_mask = 0U;
static float g_foc_sine_lut[FOC_SINE_LUT_SIZE + 1U];
static uint8_t g_foc_sine_lut_ready = 0U;
static FOC_LogSample g_foc_log[FOC_LOG_CAPACITY];
static uint8_t g_foc_log_count = 0U;
#if FOC_PREFAULT_CAPTURE_ENABLE
static FOC_PreFaultSample g_foc_prefault_log[FOC_PREFAULT_LOG_CAPACITY];
static uint8_t g_foc_prefault_write_index = 0U;
static uint8_t g_foc_prefault_count = 0U;
static uint32_t g_foc_prefault_sequence = 0U;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */
static void Set_Phase_PWM_Ticks(uint32_t phase_a, uint32_t phase_b, uint32_t phase_c);
static void Debug_Status1_Set(GPIO_PinState state);
static void Debug_Status2_Set(GPIO_PinState state);
static bool DRV8353_PrintCSAGain(void);
static void CurrentSense_SampleLowPhase(uint8_t low_phase);
static int32_t CurrentSense_GetLowPhaseAverageMilliAmps(uint8_t phase);
static bool DRV8353_WriteAndVerifyCSA(void);
static bool DRV8353_DumpRegisters(const char *label);
static void DRV8353_EnableMISODiagnosticPullup(void);
static void DRV8353_SPIClockBurstTest(void);
static void DRV8353_BitBangInit(void);
static uint16_t DRV8353_BitBangTransfer(uint16_t tx);
static void AS5048A_InitChipSelect(void);
static bool AS5048A_ReadRegister(uint16_t address, uint16_t *response);
static void AS5048A_PrintTest(void);
static void SVPWM_Log(uint32_t now_tick, float electrical_angle_rad);
static void SVPWM_Log_Dump(void);
static bool CurrentSense_CalibrateOffsets(void);
static bool CurrentSense_StartSynchronized(void);
static void CurrentSense_SampleLowPhase(uint8_t low_phase);
static int32_t CurrentSense_GetLowPhaseAverageMilliAmps(uint8_t phase);
static int32_t SVPWM_ADCToMilliAmps(uint16_t adc_value, uint16_t offset_adc);
static float AbsFloat(float value);
static void UART_QuickTest(void);
static void Motor_TestCooldownCountdown(void);
static void LED_FlashTest(void);
static bool Boot_ForceMainFlashOptionBytes(void);
static void Motor_FieldOrientationDemo(uint8_t deadtime_ticks);
static bool AS5048A_ReadAngle(uint16_t *angle_count);
static void FOC_CurrentLoopISR(const PhaseCurrents_t *currents);
static void Motor_FOC_Demo(uint8_t deadtime_ticks);
static void AS5048A_EncoderTestLoop(void);
static void FOC_LogSampleCapture(uint32_t elapsed_ms, float mechanical_rpm);
static void FOC_LogDump(void);
#if FOC_PREFAULT_CAPTURE_ENABLE
static inline void FOC_PreFaultCapture(const PhaseCurrents_t *currents,
                                       float id, float iq);
static void FOC_PreFaultDump(void);
#endif
static float WrapRadians(float radians);
static float WrapSignedRadians(float radians);
static void FOC_SineLUTInit(void);
static void FOC_PublishEncoderObservation(float electrical_angle_rad,
                                          float electrical_velocity_rad_s,
                                          uint32_t observation_cycles);
static inline void FOC_FastSinCos(float radians, float *sin_value,
                                  float *cos_value);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static bool Boot_ForceMainFlashOptionBytes(void)
{
  FLASH_OBProgramInitTypeDef option_bytes = {0};
  HAL_StatusTypeDef status;
  const uint32_t boot_mask = FLASH_OPTR_nSWBOOT0 | FLASH_OPTR_nBOOT0;
  const uint32_t boot_from_main_flash = FLASH_OPTR_nBOOT0;

  HAL_FLASHEx_OBGetConfig(&option_bytes);

  if ((option_bytes.USERConfig & boot_mask) == boot_from_main_flash)
  {
    return true;
  }

  status = HAL_FLASH_Unlock();
  if (status != HAL_OK)
  {
    return false;
  }

  status = HAL_FLASH_OB_Unlock();
  if (status == HAL_OK)
  {
    option_bytes.OptionType = OPTIONBYTE_USER;
    option_bytes.USERType = OB_USER_nSWBOOT0 | OB_USER_nBOOT0;
    option_bytes.USERConfig = OB_BOOT0_FROM_OB | OB_nBOOT0_SET;
    status = HAL_FLASHEx_OBProgram(&option_bytes);
  }

  if (status == HAL_OK)
  {
    /* A successful launch reloads the option bytes and resets the MCU. */
    status = HAL_FLASH_OB_Launch();
  }

  (void)HAL_FLASH_OB_Lock();
  (void)HAL_FLASH_Lock();
  return (status == HAL_OK);
}

static uint32_t fdcan_dlc_from_len(uint8_t len)
{
  switch (len)
  {
    case 0: return FDCAN_DLC_BYTES_0;
    case 1: return FDCAN_DLC_BYTES_1;
    case 2: return FDCAN_DLC_BYTES_2;
    case 3: return FDCAN_DLC_BYTES_3;
    case 4: return FDCAN_DLC_BYTES_4;
    case 5: return FDCAN_DLC_BYTES_5;
    case 6: return FDCAN_DLC_BYTES_6;
    case 7: return FDCAN_DLC_BYTES_7;
    default: return FDCAN_DLC_BYTES_8;
  }
}

static uint8_t len_from_fdcan_dlc(uint32_t dlc)
{
  switch (dlc)
  {
    case FDCAN_DLC_BYTES_0: return 0;
    case FDCAN_DLC_BYTES_1: return 1;
    case FDCAN_DLC_BYTES_2: return 2;
    case FDCAN_DLC_BYTES_3: return 3;
    case FDCAN_DLC_BYTES_4: return 4;
    case FDCAN_DLC_BYTES_5: return 5;
    case FDCAN_DLC_BYTES_6: return 6;
    case FDCAN_DLC_BYTES_7: return 7;
    default: return 8;
  }
}

void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000U);

  while ((DWT->CYCCNT - start) < ticks)
  {
  }
}

static uint8_t SPI1_WaitNotBusy(uint32_t timeout_ms)
{
  uint32_t start_tick = HAL_GetTick();

  while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY))
  {
    if ((HAL_GetTick() - start_tick) >= timeout_ms)
    {
      Debug_Status1_Set(GPIO_PIN_SET);
      return 0U;
    }
  }

  return 1U;
}

static void Motor_PWM_Off(void)
{
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  TIM1->CCER &= ~TIM1_ALL_OUTPUTS;
}

static void Motor_PWM_Enable(void)
{
  TIM1->EGR = TIM_EGR_UG;
  TIM1->CCER |= TIM1_ALL_OUTPUTS;
  TIM1->BDTR |= TIM_BDTR_MOE;
  TIM1->CR1 |= TIM_CR1_CEN;
}

static void Motor_Fault_Shutdown(void)
{
  Motor_PWM_Off();
  HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
  Debug_Status1_Set(GPIO_PIN_SET);
}

static bool DRV8353_ReadSPI_Safe(uint8_t reg, uint16_t *value)
{
  uint16_t rx = 0;
  uint16_t tx = 0x8000U | ((reg & 0x0FU) << 11);
#if !DRV_SPI_BITBANG_GPIO_TEST
  HAL_StatusTypeDef status;
#endif

  if (value == NULL)
  {
    return false;
  }

#if DRV_SPI_BITBANG_GPIO_TEST
  rx = DRV8353_BitBangTransfer(tx);
#else
  if (SPI1_WaitNotBusy(2U) == 0U)
  {
    return false;
  }

  HAL_GPIO_WritePin(DRV_SCS_N_GPIO_Port, DRV_SCS_N_Pin, GPIO_PIN_RESET);
  delay_us(1);
  status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tx, (uint8_t *)&rx, 1, HAL_MAX_DELAY);
  delay_us(1);
  HAL_GPIO_WritePin(DRV_SCS_N_GPIO_Port, DRV_SCS_N_Pin, GPIO_PIN_SET);

  if (status != HAL_OK)
  {
    Debug_Status1_Set(GPIO_PIN_SET);
    return false;
  }
#endif

  delay_us(1);
  g_drv_spi_last_tx = tx;
  g_drv_spi_last_rx = rx;
  *value = rx & 0x07FFU;

  return true;
}

uint16_t DRV8353_ReadSPI(uint8_t reg)
{
  uint16_t value = 0U;

  if (!DRV8353_ReadSPI_Safe(reg, &value))
  {
    Motor_Fault_Shutdown();
  }

  return value;
}

static bool DRV8353_WriteSPI_Safe(uint8_t reg, uint16_t data)
{
  uint16_t rx = 0;
  uint16_t tx = ((reg & 0x0FU) << 11) | (data & 0x07FFU);
#if !DRV_SPI_BITBANG_GPIO_TEST
  HAL_StatusTypeDef status;
#endif

#if DRV_SPI_BITBANG_GPIO_TEST
  rx = DRV8353_BitBangTransfer(tx);
  (void)rx;
#else
  if (SPI1_WaitNotBusy(2U) == 0U)
  {
    return false;
  }

  HAL_GPIO_WritePin(DRV_SCS_N_GPIO_Port, DRV_SCS_N_Pin, GPIO_PIN_RESET);
  delay_us(1);
  status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tx, (uint8_t *)&rx, 1, HAL_MAX_DELAY);
  delay_us(1);
  HAL_GPIO_WritePin(DRV_SCS_N_GPIO_Port, DRV_SCS_N_Pin, GPIO_PIN_SET);

  if (status != HAL_OK)
  {
    Debug_Status1_Set(GPIO_PIN_SET);
    return false;
  }

  delay_us(1);
#endif
  return true;
}

void DRV8353_WriteSPI(uint8_t reg, uint16_t data)
{
  if (!DRV8353_WriteSPI_Safe(reg, data))
  {
    Motor_Fault_Shutdown();
  }
}

uint8_t DRV8353_ReadFaults(void)
{
  uint16_t fault_status1 = 0U;
  uint16_t vgs_status2 = 0U;

  if (!DRV8353_ReadSPI_Safe(FAULT_STATUS1_REG_ADDR, &fault_status1) ||
      !DRV8353_ReadSPI_Safe(VGS_STATUS2_REG_ADDR, &vgs_status2))
  {
    Motor_Fault_Shutdown();
    return 1U;
  }

  g_drv_fault_status1 = fault_status1;
  g_drv_vgs_status2 = vgs_status2;

  return ((g_drv_fault_status1 != 0U) || (g_drv_vgs_status2 != 0U)) ? 1U : 0U;
}

void DRV8353_UpdateFaultLED(void)
{
  if (DRV8353_ReadFaults() != 0U)
  {
    Motor_Fault_Shutdown();
  }
  else
  {
    Debug_Status1_Set(GPIO_PIN_RESET);
  }
}

bool DRV8353_ConfigureSixPWM(void)
{
  uint16_t driver_control = 0U;
  uint16_t readback = 0U;

  if (!DRV8353_ReadSPI_Safe(DRIVER_CONTROL_REG_ADDR, &driver_control))
  {
    Motor_Fault_Shutdown();
    return false;
  }

  driver_control |= DRV_CTRL_CLR_FLT;
  if (!DRV8353_WriteSPI_Safe(DRIVER_CONTROL_REG_ADDR, driver_control))
  {
    Motor_Fault_Shutdown();
    return false;
  }
  HAL_Delay(1);

  if (!DRV8353_WriteSPI_Safe(GATE_DRIVE_HS_REG_ADDR,
                             DRV_GATE_LOCK_UNLOCK |
                             DRV_IDRIVEP_100MA |
                             DRV_IDRIVEN_200MA) ||
      !DRV8353_WriteSPI_Safe(GATE_DRIVE_LS_REG_ADDR,
                             DRV_TDRIVE_500NS |
                             DRV_IDRIVEP_100MA |
                             DRV_IDRIVEN_200MA) ||
      !DRV8353_WriteSPI_Safe(OCP_CONTROL_REG_ADDR,
                             DRV_OCP_DEADTIME_200NS |
                             DRV_OCP_MODE_LATCHED |
                             DRV_OCP_DEG_4US |
                             DRV_VDS_LVL_0600MV) ||
      !DRV8353_WriteSPI_Safe(CSA_CONTROL_REG_ADDR,
                             DRV_CSA_VREF_DIV2 |
                             DRV_CSA_GAIN_40VV |
                             DRV_CSA_SEN_LVL_025V))
  {
    Motor_Fault_Shutdown();
    return false;
  }
  HAL_Delay(1);

  driver_control &= (uint16_t)~(DRV_CTRL_PWM_MODE_MASK |
                                DRV_CTRL_COAST |
                                DRV_CTRL_BRAKE |
                                DRV_CTRL_CLR_FLT);
  driver_control |= (DRV_CTRL_PWM_MODE_6X | DRV_CTRL_OCP_ACT);
  if (!DRV8353_WriteSPI_Safe(DRIVER_CONTROL_REG_ADDR, driver_control))
  {
    Motor_Fault_Shutdown();
    return false;
  }
  HAL_Delay(1);

  if (!DRV8353_ReadSPI_Safe(DRIVER_CONTROL_REG_ADDR, &readback))
  {
    Motor_Fault_Shutdown();
    return false;
  }

  g_drv_driver_control = readback;
  return true;
}

static bool DRV8353_PrintCSAGain(void)
{
  uint16_t csa_control = 0U;
  uint16_t gain_v_per_v;

  if (!DRV8353_ReadSPI_Safe(CSA_CONTROL_REG_ADDR, &csa_control))
  {
    printf("DRV8353S CSA register read failed\r\n");
    Motor_Fault_Shutdown();
    return false;
  }

  switch ((csa_control >> 6) & 0x03U)
  {
    case 0U: gain_v_per_v = 5U;  break;
    case 1U: gain_v_per_v = 10U; break;
    case 2U: gain_v_per_v = 20U; break;
    default: gain_v_per_v = 40U; break;
  }

  g_drv_csa_gain_v_per_v = gain_v_per_v;
  g_current_amps_per_adc_count = ADC_VREF /
      (CURRENT_ADC_FULL_SCALE * CURRENT_SHUNT_OHMS * (float)gain_v_per_v);

  printf("DRV8353S CSA_CONTROL=0x%03X, gain=%u V/V%s\r\n",
         csa_control,
         gain_v_per_v,
         (gain_v_per_v == (uint16_t)CURRENT_AMP_GAIN_V_PER_V) ? "" :
         " (WARNING: current conversion gain mismatch)");

  return true;
}

static bool DRV8353_WriteAndVerifyCSA(void)
{
  uint16_t before = 0U;
  uint16_t readback = 0U;

  if (!DRV8353_ReadSPI_Safe(CSA_CONTROL_REG_ADDR, &before))
  {
    printf("CSA_CONTROL initial read failed\r\n");
    return false;
  }

  printf("CSA_CONTROL before write: 0x%03X\r\n", before);

  if (!DRV8353_WriteSPI_Safe(CSA_CONTROL_REG_ADDR, DRV_CSA_DESIRED_CONFIG))
  {
    printf("CSA_CONTROL write transaction failed\r\n");
    return false;
  }

  HAL_Delay(1U);

  if (!DRV8353_ReadSPI_Safe(CSA_CONTROL_REG_ADDR, &readback))
  {
    printf("CSA_CONTROL readback transaction failed\r\n");
    return false;
  }

  printf("CSA_CONTROL wrote 0x%03X, read back 0x%03X\r\n",
         DRV_CSA_DESIRED_CONFIG, readback);

  if ((readback & DRV_CSA_CONFIG_MASK) != DRV_CSA_DESIRED_CONFIG)
  {
    printf("ERROR: CSA_CONTROL did not accept the requested configuration; PWM inhibited\r\n");
    Motor_Fault_Shutdown();
    return false;
  }

  return true;
}

static bool DRV8353_DumpRegisters(const char *label)
{
  uint8_t reg;
  uint16_t value;

  printf("DRV8353S register dump (%s)\r\n", label);
#if DRV_SPI_BITBANG_GPIO_TEST
  printf("  BITBANG PA4=nSCS PA5=SCLK PA6=SDI/TX PA7=SDO/RX; ENABLE=%u nSCS=%u nFAULT=%u SDO/PA7_idle=%u\r\n",
         (HAL_GPIO_ReadPin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin) == GPIO_PIN_SET) ? 1U : 0U,
         (HAL_GPIO_ReadPin(DRV_SCS_N_GPIO_Port, DRV_SCS_N_Pin) == GPIO_PIN_SET) ? 1U : 0U,
         (HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin) == GPIO_PIN_SET) ? 1U : 0U,
         (HAL_GPIO_ReadPin(DRV_BB_SDO_GPIO_Port, DRV_BB_SDO_Pin) == GPIO_PIN_SET) ? 1U : 0U);
#else
  printf("  ENABLE=%u nSCS=%u nFAULT=%u SDO/PA6_idle=%u SPI_state=%u SPI_error=0x%lX\r\n",
         (HAL_GPIO_ReadPin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin) == GPIO_PIN_SET) ? 1U : 0U,
         (HAL_GPIO_ReadPin(DRV_SCS_N_GPIO_Port, DRV_SCS_N_Pin) == GPIO_PIN_SET) ? 1U : 0U,
         (HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin) == GPIO_PIN_SET) ? 1U : 0U,
         (HAL_GPIO_ReadPin(DRV_MISO_GPIO_Port, DRV_MISO_Pin) == GPIO_PIN_SET) ? 1U : 0U,
         (unsigned int)HAL_SPI_GetState(&hspi1),
         (unsigned long)HAL_SPI_GetError(&hspi1));
#endif

  for (reg = 0U; reg <= 7U; ++reg)
  {
    if (!DRV8353_ReadSPI_Safe(reg, &value))
    {
      printf("  REG 0x%02X: SPI transaction failed\r\n", reg);
      return false;
    }

    printf("  REG 0x%02X: TX=0x%04X RAW_RX=0x%04X DATA=0x%03X\r\n",
           reg,
           g_drv_spi_last_tx,
           g_drv_spi_last_rx,
           value);
  }

  return true;
}

static void DRV8353_BitBangInit(void)
{
  GPIO_InitTypeDef gpio = {0};

  /* Stop SPI1 before changing its pins back to ordinary GPIO. */
  __HAL_SPI_DISABLE(&hspi1);

  /* Establish inactive levels before enabling the GPIO outputs. */
  HAL_GPIO_WritePin(DRV_SCS_N_GPIO_Port, DRV_SCS_N_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DRV_SCLK_GPIO_Port, DRV_SCLK_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DRV_BB_SDI_GPIO_Port, DRV_BB_SDI_Pin, GPIO_PIN_RESET);

  gpio.Pin = DRV_SCS_N_Pin | DRV_SCLK_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* Crossed PCB trace: drive PA6 push-pull into the DRV SDI input. */
  gpio.Pin = DRV_BB_SDI_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DRV_BB_SDI_GPIO_Port, &gpio);

  /* Crossed PCB trace: read open-drain DRV SDO on PA7 with a pull-up. */
  gpio.Pin = DRV_BB_SDO_Pin;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DRV_BB_SDO_GPIO_Port, &gpio);

  delay_us(5U);
}

static uint16_t DRV8353_BitBangTransfer(uint16_t tx)
{
  uint16_t rx = 0U;
  uint16_t mask;

  HAL_GPIO_WritePin(DRV_SCLK_GPIO_Port, DRV_SCLK_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DRV_SCS_N_GPIO_Port, DRV_SCS_N_Pin, GPIO_PIN_RESET);
  delay_us(DRV_BB_EDGE_DELAY_US);

  /* DRV8353 propagates SDO on rising SCLK and captures SDI on falling SCLK. */
  for (mask = 0x8000U; mask != 0U; mask >>= 1U)
  {
    HAL_GPIO_WritePin(DRV_BB_SDI_GPIO_Port, DRV_BB_SDI_Pin,
                      ((tx & mask) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    delay_us(DRV_BB_EDGE_DELAY_US);

    HAL_GPIO_WritePin(DRV_SCLK_GPIO_Port, DRV_SCLK_Pin, GPIO_PIN_SET);
    delay_us(DRV_BB_EDGE_DELAY_US);

    rx <<= 1U;
    if (HAL_GPIO_ReadPin(DRV_BB_SDO_GPIO_Port, DRV_BB_SDO_Pin) == GPIO_PIN_SET)
    {
      rx |= 1U;
    }

    HAL_GPIO_WritePin(DRV_SCLK_GPIO_Port, DRV_SCLK_Pin, GPIO_PIN_RESET);
    delay_us(DRV_BB_EDGE_DELAY_US);
  }

  HAL_GPIO_WritePin(DRV_SCS_N_GPIO_Port, DRV_SCS_N_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DRV_BB_SDI_GPIO_Port, DRV_BB_SDI_Pin, GPIO_PIN_RESET);
  delay_us(DRV_BB_EDGE_DELAY_US);

  return rx;
}

static void DRV8353_EnableMISODiagnosticPullup(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t before;
  uint8_t after;

  before = (HAL_GPIO_ReadPin(DRV_MISO_GPIO_Port, DRV_MISO_Pin) == GPIO_PIN_SET) ? 1U : 0U;

  gpio.Pin = DRV_MISO_Pin;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(DRV_MISO_GPIO_Port, &gpio);
  HAL_Delay(1U);

  after = (HAL_GPIO_ReadPin(DRV_MISO_GPIO_Port, DRV_MISO_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  printf("SDO/PA6 internal-pullup test: before=%u after=%u\r\n", before, after);
}

static void DRV8353_SPIClockBurstTest(void)
{
  uint32_t i;
  uint32_t non_ffff_count = 0U;
  uint32_t failed_count = 0U;
  uint16_t value = 0U;

  /* Long back-to-back burst makes PA5/nSCS activity easy to capture on a scope. */
  for (i = 0U; i < 4096U; ++i)
  {
    if (!DRV8353_ReadSPI_Safe(CSA_CONTROL_REG_ADDR, &value))
    {
      ++failed_count;
    }
    else if (g_drv_spi_last_rx != 0xFFFFU)
    {
      ++non_ffff_count;
    }
  }

  printf("SPI burst complete: 4096 reads of REG 0x06, last_RAW_RX=0x%04X, non_FFFF=%lu, HAL_failures=%lu\r\n",
         g_drv_spi_last_rx,
         (unsigned long)non_ffff_count,
         (unsigned long)failed_count);
}

static uint16_t AS5048A_AddEvenParity(uint16_t frame)
{
  uint16_t bits = frame & 0x7FFFU;
  uint16_t parity = 0U;

  while (bits != 0U)
  {
    parity ^= (bits & 1U);
    bits >>= 1U;
  }

  return frame | (uint16_t)(parity << 15U);
}

static bool AS5048A_ResponseParityOK(uint16_t frame)
{
  uint16_t parity = 0U;
  uint16_t bits = frame;

  while (bits != 0U)
  {
    parity ^= (bits & 1U);
    bits >>= 1U;
  }

  return parity == 0U;
}

static void AS5048A_InitChipSelect(void)
{
  GPIO_InitTypeDef gpio = {0};

  HAL_GPIO_WritePin(AS5048A_CS_GPIO_Port, AS5048A_CS_Pin, GPIO_PIN_SET);
  gpio.Pin = AS5048A_CS_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(AS5048A_CS_GPIO_Port, &gpio);
}

static bool AS5048A_Transfer(uint16_t tx, uint16_t *rx)
{
  HAL_StatusTypeDef status;

  HAL_GPIO_WritePin(AS5048A_CS_GPIO_Port, AS5048A_CS_Pin, GPIO_PIN_RESET);
  delay_us(1U);
  status = HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&tx, (uint8_t *)rx,
                                  1U, 10U);
  delay_us(1U);
  HAL_GPIO_WritePin(AS5048A_CS_GPIO_Port, AS5048A_CS_Pin, GPIO_PIN_SET);
  delay_us(1U);

  return status == HAL_OK;
}

static bool AS5048A_ReadRegister(uint16_t address, uint16_t *response)
{
  uint16_t ignored = 0U;
  uint16_t command = AS5048A_AddEvenParity(0x4000U | (address & 0x3FFFU));

  /* AS5048A responses are pipelined: command frame, then a separate NOP frame. */
  if (!AS5048A_Transfer(command, &ignored))
  {
    return false;
  }

  return AS5048A_Transfer(AS5048A_AddEvenParity(AS5048A_REG_NOP), response);
}

static bool AS5048A_ReadAngle(uint16_t *angle_count)
{
  uint16_t response = 0U;

  if (angle_count == NULL ||
      !AS5048A_ReadRegister(AS5048A_REG_ANGLE, &response) ||
      !AS5048A_ResponseParityOK(response) ||
      (response & 0x4000U) != 0U)
  {
    return false;
  }

  *angle_count = response & 0x3FFFU;
  return true;
}

static void AS5048A_PrintTest(void)
{
  uint16_t angle_response = 0U;
  uint16_t diagnostic_response = 0U;
  uint16_t angle;
  uint32_t degrees_milli;

  if (!AS5048A_ReadRegister(AS5048A_REG_ANGLE, &angle_response) ||
      !AS5048A_ReadRegister(AS5048A_REG_DIAGNOSTICS, &diagnostic_response))
  {
    printf("AS5048A SPI transaction failed: state=%u error=0x%lX\r\n",
           (unsigned int)HAL_SPI_GetState(&hspi3),
           (unsigned long)HAL_SPI_GetError(&hspi3));
    return;
  }

  angle = angle_response & 0x3FFFU;
  degrees_milli = (uint32_t)(((uint64_t)angle * 360000ULL) / 16384ULL);
  printf("AS5048A ANGLE raw=0x%04X count=%u angle=%lu.%03lu deg parity=%s EF=%u; DIAG=0x%04X\r\n",
         angle_response, angle,
         (unsigned long)(degrees_milli / 1000U),
         (unsigned long)(degrees_milli % 1000U),
         AS5048A_ResponseParityOK(angle_response) ? "OK" : "BAD",
         (angle_response & 0x4000U) ? 1U : 0U,
         diagnostic_response);
}

static inline void Phase_Disconnected(uint8_t ch)
{
  switch (ch)
  {
    case 1:
      TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);
      break;
    case 2:
      TIM1->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC2NE);
      break;
    case 3:
      TIM1->CCER &= ~(TIM_CCER_CC3E | TIM_CCER_CC3NE);
      break;
    default:
      break;
  }
}

static inline void Phase_High(uint8_t ch, uint16_t duty)
{
  switch (ch)
  {
    case 1:
      TIM1->CCR1 = duty;
      TIM1->CCER &= ~TIM_CCER_CC1NE;
      TIM1->CCER |= TIM_CCER_CC1E;
      break;
    case 2:
      TIM1->CCR2 = duty;
      TIM1->CCER &= ~TIM_CCER_CC2NE;
      TIM1->CCER |= TIM_CCER_CC2E;
      break;
    case 3:
      TIM1->CCR3 = duty;
      TIM1->CCER &= ~TIM_CCER_CC3NE;
      TIM1->CCER |= TIM_CCER_CC3E;
      break;
    default:
      break;
  }
}

static inline void Phase_Low(uint8_t ch)
{
  switch (ch)
  {
    case 1:
      TIM1->CCR1 = 0;
      TIM1->CCER &= ~TIM_CCER_CC1E;
      TIM1->CCER |= TIM_CCER_CC1NE;
      break;
    case 2:
      TIM1->CCR2 = 0;
      TIM1->CCER &= ~TIM_CCER_CC2E;
      TIM1->CCER |= TIM_CCER_CC2NE;
      break;
    case 3:
      TIM1->CCR3 = 0;
      TIM1->CCER &= ~TIM_CCER_CC3E;
      TIM1->CCER |= TIM_CCER_CC3NE;
      break;
    default:
      break;
  }
}

void Commutate(uint8_t high, uint8_t low, uint8_t floating, uint16_t duty)
{
  Motor_PWM_Off();

  /* Conservative break-before-make blanking for six-step debug paths. */
  Phase_Disconnected(1);
  Phase_Disconnected(2);
  Phase_Disconnected(3);
  delay_us(1);

  Phase_High(high, duty);
  Phase_Low(low);
  Phase_Disconnected(floating);
  TIM1->EGR = TIM_EGR_UG;
  Motor_PWM_Enable();

  CurrentSense_SampleLowPhase(low);

  prev_high = high;
  prev_low = low;
}

uint16_t read_adc_channel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

  uint16_t value = (uint16_t)HAL_ADC_GetValue(&hadc1);

  HAL_ADC_Stop(&hadc1);

  return value;
}

static bool CurrentSense_CalibrateOffsets(void)
{
  static const uint32_t channels[3] = {
    ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_8
  };
  static const char phase_names[3] = {'A', 'B', 'C'};
  uint32_t phase;
  bool valid = true;
  uint16_t csa_before = 0U;
  uint16_t cal_mode_before = 0U;
  uint16_t readback = 0U;

  Motor_PWM_Off();
  HAL_Delay(20U);

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    printf("ERROR: STM32 ADC self-calibration failed; PWM inhibited\r\n");
    return false;
  }

#if DRV_SPI_CSA_CALIBRATION_TRIAL
  if (!DRV8353_ReadSPI_Safe(CSA_CONTROL_REG_ADDR, &csa_before) ||
      !DRV8353_ReadSPI_Safe(DRIVER_CALIBRATION_REG_ADDR, &cal_mode_before))
  {
    printf("ERROR: could not read DRV CSA calibration registers\r\n");
    return false;
  }

  if (!DRV8353_WriteSPI_Safe(DRIVER_CALIBRATION_REG_ADDR,
                             cal_mode_before | DRV_CAL_MODE_MANUAL) ||
      !DRV8353_ReadSPI_Safe(DRIVER_CALIBRATION_REG_ADDR, &readback) ||
      ((readback & DRV_CAL_MODE_MANUAL) == 0U))
  {
    printf("ERROR: DRV CAL_MODE write/readback failed\r\n");
    return false;
  }

  if (!DRV8353_WriteSPI_Safe(CSA_CONTROL_REG_ADDR,
                             csa_before | DRV_CSA_CAL_ALL) ||
      !DRV8353_ReadSPI_Safe(CSA_CONTROL_REG_ADDR, &readback) ||
      ((readback & DRV_CSA_CAL_ALL) != DRV_CSA_CAL_ALL))
  {
    printf("ERROR: DRV CSA_CAL_A/B/C write/readback failed\r\n");
    (void)DRV8353_WriteSPI_Safe(DRIVER_CALIBRATION_REG_ADDR, cal_mode_before);
    return false;
  }

  printf("DRV manual CSA calibration active: REG6 0x%03X -> 0x%03X, REG7=0x%03X\r\n",
         csa_before, readback, (cal_mode_before | DRV_CAL_MODE_MANUAL));
  HAL_Delay(1U);
#endif

  printf("Current-sense zero calibration: PWM off, %u samples per channel\r\n",
         CURRENT_OFFSET_CAL_SAMPLES);

  for (phase = 0U; phase < 3U; ++phase)
  {
    uint32_t i;
    uint32_t sum = 0U;
    uint16_t minimum = 0xFFFFU;
    uint16_t maximum = 0U;
    uint16_t average;
    uint32_t millivolts;

    /* Discard initial conversions after changing ADC channels and calibration. */
    for (i = 0U; i < CURRENT_OFFSET_DISCARD_SAMPLES; ++i)
    {
      (void)read_adc_channel(channels[phase]);
    }

    for (i = 0U; i < CURRENT_OFFSET_CAL_SAMPLES; ++i)
    {
      uint16_t sample = read_adc_channel(channels[phase]);

      sum += sample;
      if (sample < minimum)
      {
        minimum = sample;
      }
      if (sample > maximum)
      {
        maximum = sample;
      }
    }

    average = (uint16_t)((sum + (CURRENT_OFFSET_CAL_SAMPLES / 2U)) /
                         CURRENT_OFFSET_CAL_SAMPLES);
    g_current_offset_adc[phase] = average;
    millivolts = ((uint32_t)average * 3300U + 2047U) / 4095U;

    printf("  SO%c offset=%u ADC (%lu mV), min=%u max=%u p-p=%u counts%s\r\n",
           phase_names[phase], average, (unsigned long)millivolts,
           minimum, maximum, (uint16_t)(maximum - minimum),
           (average < CURRENT_OFFSET_VALID_MIN_ADC ||
            average > CURRENT_OFFSET_VALID_MAX_ADC) ? " INVALID" : "");

    if (average < CURRENT_OFFSET_VALID_MIN_ADC ||
        average > CURRENT_OFFSET_VALID_MAX_ADC)
    {
      valid = false;
    }
  }

#if DRV_SPI_CSA_CALIBRATION_TRIAL
  /* Always restore normal CSA operation before allowing PWM. */
  if (!DRV8353_WriteSPI_Safe(CSA_CONTROL_REG_ADDR, csa_before) ||
      !DRV8353_ReadSPI_Safe(CSA_CONTROL_REG_ADDR, &readback) ||
      ((readback & DRV_CSA_CAL_ALL) != 0U))
  {
    printf("ERROR: failed to clear DRV CSA calibration bits; PWM inhibited\r\n");
    valid = false;
  }
  if (!DRV8353_WriteSPI_Safe(DRIVER_CALIBRATION_REG_ADDR, cal_mode_before) ||
      !DRV8353_ReadSPI_Safe(DRIVER_CALIBRATION_REG_ADDR, &readback) ||
      ((readback & DRV_CAL_MODE_MANUAL) != (cal_mode_before & DRV_CAL_MODE_MANUAL)))
  {
    printf("ERROR: failed to restore DRV CAL_MODE; PWM inhibited\r\n");
    valid = false;
  }
  printf("DRV CSA calibration disabled; normal current sensing restored\r\n");
#endif

  if (!valid)
  {
    printf("ERROR: one or more CSA outputs are near a supply rail; PWM inhibited\r\n");
  }
  else
  {
    printf("Current conversion: 1 mOhm shunt, verified %u V/V gain, channel-specific offsets\r\n",
           g_drv_csa_gain_v_per_v);
  }

  return valid;
}

float adc_to_current(uint16_t adc_val, float offset)
{
  float voltage = ((float)adc_val / CURRENT_ADC_FULL_SCALE) * ADC_VREF;
  return (voltage - offset) / CURRENT_GAIN;
}

static float CurrentSense_CountsToAmps(uint16_t sample, uint16_t offset)
{
  return ((float)((int32_t)sample - (int32_t)offset) * ADC_VREF) /
         (CURRENT_ADC_FULL_SCALE * CURRENT_SHUNT_OHMS *
          (float)g_drv_csa_gain_v_per_v);
}

static bool CurrentSense_StartSynchronized(void)
{
  if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
  {
    printf("ERROR: failed to arm TIM1-triggered injected current conversions\r\n");
    return false;
  }

#if !CURRENT_OFFSET_CALIBRATION_ENABLE
  g_current_offset_adc[0] = (uint16_t)(((CURRENT_FIXED_OFFSET_MV * 4095U) + 1650U) / 3300U);
  g_current_offset_adc[1] = g_current_offset_adc[0];
  g_current_offset_adc[2] = g_current_offset_adc[0];
  printf("Current-sense offsets fixed at %u mV: A=%u, B=%u, C=%u ADC counts\r\n",
         CURRENT_FIXED_OFFSET_MV, g_current_offset_adc[0], g_current_offset_adc[1],
         g_current_offset_adc[2]);
  return true;
#endif

  return true;
}

PhaseCurrents_t CurrentSense_GetPhaseCurrents(void)
{
  PhaseCurrents_t snapshot;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  snapshot = g_phase_currents;
  __set_PRIMASK(primask);
  return snapshot;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  PhaseCurrents_t next;
  uint32_t trigger_tick;
  uint32_t ccr_a;
  uint32_t ccr_b;
  uint32_t ccr_c;
  uint32_t isr_start_cycles;
  uint8_t valid_count;
  uint8_t foc_was_enabled;
  float abs_current;

  if (hadc->Instance != ADC1)
  {
    return;
  }

  isr_start_cycles = DWT->CYCCNT;
  foc_was_enabled = g_foc_enabled;

#if CURRENT_SENSE_SAMPLE_DEBUG_GPIO
  STATUS_1_GPIO_Port->BSRR = STATUS_1_Pin;
#endif

  next = g_phase_currents;
  next.raw_a = (uint16_t)ADC1->JDR1;
  next.raw_b = (uint16_t)ADC1->JDR2;
  next.raw_c = (uint16_t)ADC1->JDR3;
  next.offset_a = g_current_offset_adc[0];
  next.offset_b = g_current_offset_adc[1];
  next.offset_c = g_current_offset_adc[2];
  /* This common scale is calculated when the DRV CSA gain is verified, not
   * inside the current-control interrupt. */
  next.ia = (float)((int32_t)next.raw_a - (int32_t)next.offset_a) *
            g_current_amps_per_adc_count;
  next.ib_sensed = (float)((int32_t)next.raw_b - (int32_t)next.offset_b) *
                   g_current_amps_per_adc_count;
  next.ib_sensed_corrected = next.ib_sensed;
  if (g_foc_b_bias_correction_enabled != 0U)
  {
    next.ib_sensed_corrected -= g_foc_b_pwm_bias_a;
  }
  next.ib = next.ib_sensed_corrected;
  next.ic_sensed = (float)((int32_t)next.raw_c - (int32_t)next.offset_c) *
                   g_current_amps_per_adc_count;
  next.ic = next.ic_sensed;

  trigger_tick = TIM1->CCR4;
  ccr_a = TIM1->CCR1;
  ccr_b = TIM1->CCR2;
  ccr_c = TIM1->CCR3;
  next.valid_a = (ccr_a + CURRENT_MIN_VALID_WINDOW_TICKS <= trigger_tick);
  next.valid_b = (ccr_b + CURRENT_MIN_VALID_WINDOW_TICKS <= trigger_tick);
  next.valid_c = (ccr_c + CURRENT_MIN_VALID_WINDOW_TICKS <= trigger_tick);
  valid_count = next.valid_a + next.valid_b + next.valid_c;

  /* Prefer coherent A/C measurements when both are available. At higher
   * modulation one phase can have too little low-side conduction time, so
   * reconstruct that phase from the other two. Phase B uses the PWM-bias
   * correction measured during alignment; ib_sensed remains raw for safety
   * and diagnostics. */
  if (next.valid_a != 0U && next.valid_c != 0U)
  {
    next.ib = -(next.ia + next.ic);
  }
  else if (valid_count == 2U)
  {
    if (!next.valid_a) next.ia = -(next.ib_sensed_corrected + next.ic);
    if (!next.valid_c) next.ic = -(next.ia + next.ib_sensed_corrected);
  }
  else if (valid_count < 2U)
  {
    /* Insufficient independent shunts: preserve readings for diagnostics, but mark invalid. */
    next.valid_a = 0U;
    next.valid_b = 0U;
    next.valid_c = 0U;
  }

  next.saturated_a = (next.raw_a <= CURRENT_ADC_SATURATION_COUNTS ||
                      next.raw_a >= (uint16_t)CURRENT_ADC_FULL_SCALE - CURRENT_ADC_SATURATION_COUNTS);
  next.saturated_b = (next.raw_b <= CURRENT_ADC_SATURATION_COUNTS ||
                      next.raw_b >= (uint16_t)CURRENT_ADC_FULL_SCALE - CURRENT_ADC_SATURATION_COUNTS);
  next.saturated_c = (next.raw_c <= CURRENT_ADC_SATURATION_COUNTS ||
                      next.raw_c >= (uint16_t)CURRENT_ADC_FULL_SCALE - CURRENT_ADC_SATURATION_COUNTS);
  next.current_sum = next.ia + next.ib + next.ic;
  if (foc_was_enabled == 0U)
  {
    if (next.sample_count == 0U)
    {
      next.ia_filtered = next.ia;
      next.ib_filtered = next.ib;
      next.ic_filtered = next.ic;
    }
    else if (valid_count >= 2U)
    {
      /* Diagnostic-only one-pole filter; the FOC loop has its own interval
       * telemetry and must not pay for these operations in its ISR. */
      next.ia_filtered += CURRENT_DIAGNOSTIC_FILTER_ALPHA * (next.ia - next.ia_filtered);
      next.ib_filtered += CURRENT_DIAGNOSTIC_FILTER_ALPHA * (next.ib - next.ib_filtered);
      next.ic_filtered += CURRENT_DIAGNOSTIC_FILTER_ALPHA * (next.ic - next.ic_filtered);
    }
    abs_current = AbsFloat(next.ia);
    if (AbsFloat(next.ib) > abs_current) abs_current = AbsFloat(next.ib);
    if (AbsFloat(next.ic) > abs_current) abs_current = AbsFloat(next.ic);
    if (abs_current > next.peak_phase_current) next.peak_phase_current = abs_current;
  }
  next.sample_count++;
  g_phase_currents = next;

  if (g_foc_enabled != 0U)
  {
    FOC_CurrentLoopISR(&next);
  }

  if (foc_was_enabled != 0U)
  {
    uint32_t isr_cycles = DWT->CYCCNT - isr_start_cycles;
    if (isr_cycles > g_foc_current_isr_max_cycles)
    {
      g_foc_current_isr_max_cycles = isr_cycles;
    }
  }

#if CURRENT_SENSE_SAMPLE_DEBUG_GPIO
  STATUS_1_GPIO_Port->BRR = STATUS_1_Pin;
#endif
}

static void CurrentSense_SampleLowPhase(uint8_t low_phase)
{
  uint32_t channel;
  int32_t current_ma;

  switch (low_phase)
  {
    case 1: channel = ADC_CHANNEL_6; break;
    case 2: channel = ADC_CHANNEL_7; break;
    case 3: channel = ADC_CHANNEL_8; break;
    default: return;
  }

  /* Wait briefly for the low-side switch to conduct and stabilize. */
  delay_us(CURRENT_SENSE_SAMPLE_DELAY_US);

  uint8_t idx = low_phase - 1U;

  current_ma = SVPWM_ADCToMilliAmps(read_adc_channel(channel),
                                     g_current_offset_adc[idx]);

  if (g_current_low_phase_count[idx] >= CURRENT_SENSE_MOVING_AVERAGE_SAMPLES)
  {
    g_current_low_phase_sum_ma[idx] -=
        g_current_low_phase_buffer_ma[idx][g_current_low_phase_index[idx]];
  }
  else
  {
    g_current_low_phase_count[idx]++;
  }

  g_current_low_phase_buffer_ma[idx][g_current_low_phase_index[idx]] = current_ma;
  g_current_low_phase_sum_ma[idx] += current_ma;
  g_current_low_phase_index[idx] =
      (uint8_t)((g_current_low_phase_index[idx] + 1U) % CURRENT_SENSE_MOVING_AVERAGE_SAMPLES);

#if CURRENT_SENSE_DEBUG_PRINT
  printf("6-step current sense: low phase=%u sample=%ld mA avg=%ld mA count=%u\r\n",
         low_phase, (long)current_ma,
         (long)CurrentSense_GetLowPhaseAverageMilliAmps(low_phase),
         g_current_low_phase_count[idx]);
#endif
}

static int32_t CurrentSense_GetLowPhaseAverageMilliAmps(uint8_t phase)
{
  if (phase < 1U || phase > 3U)
  {
    return 0;
  }

  uint8_t idx = phase - 1U;
  if (g_current_low_phase_count[idx] == 0U)
  {
    return 0;
  }

  return g_current_low_phase_sum_ma[idx] / (int32_t)g_current_low_phase_count[idx];
}

/* Collect one set of raw current-sense readings without doing any UART I/O. */
static void SVPWM_Log(uint32_t now_tick, float electrical_angle_rad)
{
  SVPWM_LogSample *sample;
  PhaseCurrents_t currents;

  if (g_svpwm_log_count >= SVPWM_LOG_CAPACITY ||
      (now_tick - g_svpwm_log_last_tick) < SVPWM_LOG_INTERVAL_MS)
  {
    return;
  }

  g_svpwm_log_last_tick = now_tick;
  currents = CurrentSense_GetPhaseCurrents();
  sample = &g_svpwm_log[g_svpwm_log_count++];
  sample->time_ms = (uint16_t)(now_tick - g_svpwm_log_start_tick);
  sample->electrical_hz_x10 = (uint16_t)((g_actual_electrical_hz * 10.0f) + 0.5f);
  sample->electrical_angle_mrad = (uint16_t)((electrical_angle_rad * 1000.0f) + 0.5f);
  sample->modulation_x10000 = (uint16_t)((g_open_loop_modulation * 10000.0f) + 0.5f);
  sample->phase_a_adc = currents.raw_a;
  sample->phase_b_adc = currents.raw_b;
  sample->phase_c_adc = currents.raw_c;
  sample->valid_a = currents.valid_a;
  sample->valid_b = currents.valid_b;
  sample->valid_c = currents.valid_c;
  sample->phase_a_filtered_ma = (int32_t)(currents.ia_filtered * 1000.0f);
  sample->phase_b_filtered_ma = (int32_t)(currents.ib_filtered * 1000.0f);
  sample->phase_c_filtered_ma = (int32_t)(currents.ic_filtered * 1000.0f);
}

static int32_t SVPWM_ADCToMilliAmps(uint16_t adc_value, uint16_t offset_adc)
{
  int64_t sense_uv;

  /* With a 1 mOhm shunt, sense_uV / gain is numerically current in mA. */
  sense_uv = ((int64_t)((int32_t)adc_value - (int32_t)offset_adc) * 3300000LL) / 4095LL;
  return (int32_t)(sense_uv / (int64_t)g_drv_csa_gain_v_per_v);
}

static void SVPWM_Log_Dump(void)
{
  uint16_t i;

  printf("SVPWM current log (%u samples)\r\n", g_svpwm_log_count);
  printf("Current conversion: gain=%u V/V, shunt=1 mOhm, fixed_offset_A=%u, fixed_offset_B=%u, fixed_offset_C=%u\r\n",
         g_drv_csa_gain_v_per_v, g_current_offset_adc[0], g_current_offset_adc[1],
         g_current_offset_adc[2]);
  printf("CSV_BEGIN\r\n");
  printf("capture_time_ms,electrical_hz_x10,electrical_angle_mrad,modulation_x10000,adc_a,adc_b,adc_c,valid_a,valid_b,valid_c,ia_raw_mA,ib_raw_mA,ic_raw_mA,current_sum_raw_mA,ia_filtered_mA,ib_filtered_mA,ic_filtered_mA\r\n");

  for (i = 0U; i < g_svpwm_log_count; ++i)
  {
    int32_t current_a_ma = SVPWM_ADCToMilliAmps(g_svpwm_log[i].phase_a_adc,
                                                g_current_offset_adc[0]);
    int32_t current_b_ma = SVPWM_ADCToMilliAmps(g_svpwm_log[i].phase_b_adc,
                                                g_current_offset_adc[1]);
    int32_t current_c_ma = SVPWM_ADCToMilliAmps(g_svpwm_log[i].phase_c_adc,
                                                g_current_offset_adc[2]);

    printf("%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
           g_svpwm_log[i].time_ms,
           g_svpwm_log[i].electrical_hz_x10,
           g_svpwm_log[i].electrical_angle_mrad,
           g_svpwm_log[i].modulation_x10000,
           g_svpwm_log[i].phase_a_adc,
           g_svpwm_log[i].phase_b_adc,
           g_svpwm_log[i].phase_c_adc,
           g_svpwm_log[i].valid_a,
           g_svpwm_log[i].valid_b,
           g_svpwm_log[i].valid_c,
           (long)current_a_ma,
           (long)current_b_ma,
           (long)current_c_ma,
           (long)(current_a_ma + current_b_ma + current_c_ma),
           (long)g_svpwm_log[i].phase_a_filtered_ma,
           (long)g_svpwm_log[i].phase_b_filtered_ma,
           (long)g_svpwm_log[i].phase_c_filtered_ma);
  }

  printf("CSV_END\r\n");
}

void read_phase_currents_and_print(void)
{
  uint16_t i1 = read_adc_channel(ADC_CHANNEL_6);
  uint16_t i2 = read_adc_channel(ADC_CHANNEL_7);
  uint16_t i3 = read_adc_channel(ADC_CHANNEL_8);

  printf("I1: %.2f, I2: %.2f, I3: %.2f\r\n",
         adc_to_current(i1, 1.66f),
         adc_to_current(i2, 1.66f),
         adc_to_current(i3, 1.66f));
}

float read_filtered_current(uint32_t ch, float offset)
{
  float sum = 0.0f;
  int valid = 0;

  for (int i = 0; i < 5; i++)
  {
    uint16_t adc = read_adc_channel(ch);
    float current = adc_to_current(adc, offset);

    if (current > -5.0f && current < 5.0f)
    {
      sum += current;
      valid++;
    }
  }

  if (valid == 0)
  {
    return 0.0f;
  }

  return sum / valid;
}

static float AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float Read_Max_Phase_Current_A(void)
{
  PhaseCurrents_t currents = CurrentSense_GetPhaseCurrents();
  float max_current = 0.0f;

  /* Ignore a channel explicitly rejected by the PWM-window validity logic. */
  if (currents.valid_a) max_current = AbsFloat(currents.ia);
  if (currents.valid_b && AbsFloat(currents.ib) > max_current)
    max_current = AbsFloat(currents.ib);
  if (currents.valid_c && AbsFloat(currents.ic) > max_current)
    max_current = AbsFloat(currents.ic);

  return max_current;
}

static float Read_AverageAbs_Phase_Current_A(void)
{
  float ia = AbsFloat(adc_to_current(read_adc_channel(ADC_CHANNEL_6), CURRENT_ADC_OFFSET_V));
  float ib = AbsFloat(adc_to_current(read_adc_channel(ADC_CHANNEL_7), CURRENT_ADC_OFFSET_V));
  float ic = AbsFloat(adc_to_current(read_adc_channel(ADC_CHANNEL_8), CURRENT_ADC_OFFSET_V));

  return (ia + ib + ic) / 3.0f;
}

static void Debug_CurrentLEDs_Update(float current_a)
{
  uint8_t level = 0U;
  float one_third_scale = CURRENT_LED_FULL_SCALE_A / 3.0f;
  float two_thirds_scale = 2.0f * one_third_scale;

  if (current_a >= CURRENT_LED_FULL_SCALE_A)
  {
    level = 3U;
  }
  else if (current_a >= two_thirds_scale)
  {
    level = 2U;
  }
  else if (current_a >= one_third_scale)
  {
    level = 1U;
  }

  Debug_Status1_Set((level & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  Debug_Status2_Set((level & 0x02U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t Motor_Check_CurrentLimit(void)
{
#if MOTOR_PHASE_CURRENT_LIMIT_ENABLE
  if (Read_Max_Phase_Current_A() > MOTOR_PHASE_CURRENT_LIMIT_A)
  {
    g_current_limit_fault = 1U;
    Motor_Fault_Shutdown();
    return 1U;
  }
#endif

  return 0U;
}

uint32_t Get_TIM1_ClockHz(void)
{
  RCC_ClkInitTypeDef clk = {0};
  uint32_t flash_latency = 0;
  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();

  HAL_RCC_GetClockConfig(&clk, &flash_latency);

  if (clk.APB2CLKDivider == RCC_HCLK_DIV1)
  {
    return pclk2;
  }

  return pclk2 * 2U;
}

uint32_t Get_PWM_FrequencyHz(void)
{
  uint32_t tim_clk_hz = Get_TIM1_ClockHz();
  uint32_t prescaler = htim1.Init.Prescaler + 1U;
  uint32_t period = htim1.Init.Period + 1U;
  uint32_t center_aligned_divider = 1U;

  if (htim1.Init.CounterMode == TIM_COUNTERMODE_CENTERALIGNED1 ||
      htim1.Init.CounterMode == TIM_COUNTERMODE_CENTERALIGNED2 ||
      htim1.Init.CounterMode == TIM_COUNTERMODE_CENTERALIGNED3)
  {
    center_aligned_divider = 2U;
  }

  return tim_clk_hz / (prescaler * period * center_aligned_divider);
}

float Get_CommutationStepFrequencyHz(void)
{
  return g_actual_electrical_hz * 6.0f;
}

float Get_MechanicalRPM(void)
{
  return (g_actual_electrical_hz * 60.0f) / (float)MOTOR_POLE_PAIRS;
}

static uint32_t ElectricalHz_ToStepDelayUs(float electrical_hz)
{
  if (electrical_hz < 0.1f)
  {
    electrical_hz = 0.1f;
  }

  return (uint32_t)(1000000.0f / (electrical_hz * 6.0f));
}

static float RampToward(float current, float target, float rate_per_sec, float dt_sec)
{
  float max_delta = rate_per_sec * dt_sec;

  if (current < target)
  {
    current += max_delta;
    if (current > target)
    {
      current = target;
    }
  }
  else if (current > target)
  {
    current -= max_delta;
    if (current < target)
    {
      current = target;
    }
  }

  return current;
}

static float ClampFloat(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

static float OpenLoop_ModulationForElapsedMs(uint32_t elapsed_ms)
{
  float progress;

  if (elapsed_ms <= MOTOR_OPEN_LOOP_MODULATION_STAGE1_MS)
  {
    progress = (float)elapsed_ms / (float)MOTOR_OPEN_LOOP_MODULATION_STAGE1_MS;
    progress = ClampFloat(progress, 0.0f, 1.0f);
    return MOTOR_OPEN_LOOP_MODULATION_START +
           ((MOTOR_OPEN_LOOP_MODULATION_MID - MOTOR_OPEN_LOOP_MODULATION_START) * progress);
  }

  progress = (float)(elapsed_ms - MOTOR_OPEN_LOOP_MODULATION_STAGE1_MS) /
             (float)(MOTOR_SVPWM_RUN_TIME_MS - MOTOR_OPEN_LOOP_MODULATION_STAGE1_MS);
  progress = ClampFloat(progress, 0.0f, 1.0f);
  return MOTOR_OPEN_LOOP_MODULATION_MID +
         ((MOTOR_OPEN_LOOP_MODULATION_END - MOTOR_OPEN_LOOP_MODULATION_MID) * progress);
}

static float WrapUnitTurn(float turns)
{
  while (turns >= 1.0f)
  {
    turns -= 1.0f;
  }

  while (turns < 0.0f)
  {
    turns += 1.0f;
  }

  return turns;
}

static float WrapRadians(float radians)
{
  while (radians >= TWO_PI_F)
  {
    radians -= TWO_PI_F;
  }

  while (radians < 0.0f)
  {
    radians += TWO_PI_F;
  }

  return radians;
}

static float WrapSignedRadians(float radians)
{
  while (radians > PI_F)
  {
    radians -= TWO_PI_F;
  }

  while (radians < -PI_F)
  {
    radians += TWO_PI_F;
  }

  return radians;
}

static void FOC_SineLUTInit(void)
{
  uint32_t index;

  if (g_foc_sine_lut_ready != 0U)
  {
    return;
  }

  for (index = 0U; index < FOC_SINE_LUT_SIZE; ++index)
  {
    g_foc_sine_lut[index] =
        sinf((TWO_PI_F * (float)index) / (float)FOC_SINE_LUT_SIZE);
  }
  /* Duplicate index zero so interpolation across the wrap has no branch. */
  g_foc_sine_lut[FOC_SINE_LUT_SIZE] = g_foc_sine_lut[0U];
  g_foc_sine_lut_ready = 1U;
}

static inline void FOC_FastSinCos(float radians, float *sin_value,
                                  float *cos_value)
{
  float position;
  float fraction;
  uint32_t sine_index;
  uint32_t cosine_index;

  /* The FOC observer already wraps this angle; these guards cover boundary
   * roundoff without putting general-purpose trig in the interrupt. */
  if (radians >= TWO_PI_F)
  {
    radians -= TWO_PI_F;
  }
  else if (radians < 0.0f)
  {
    radians += TWO_PI_F;
  }

  position = radians * ((float)FOC_SINE_LUT_SIZE / TWO_PI_F);
  sine_index = ((uint32_t)position) & FOC_SINE_LUT_MASK;
  fraction = position - (float)((uint32_t)position);
  cosine_index =
      (sine_index + (FOC_SINE_LUT_SIZE / 4U)) & FOC_SINE_LUT_MASK;

  *sin_value = g_foc_sine_lut[sine_index] +
      (fraction * (g_foc_sine_lut[sine_index + 1U] -
                   g_foc_sine_lut[sine_index]));
  *cos_value = g_foc_sine_lut[cosine_index] +
      (fraction * (g_foc_sine_lut[cosine_index + 1U] -
                   g_foc_sine_lut[cosine_index]));
}

static uint16_t Clamp_PWM_Ticks(uint32_t duty)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);

  if (duty > arr)
  {
    duty = arr;
  }

  return (uint16_t)duty;
}

static void Set_PWM_DutyTicks(uint16_t duty)
{
  g_pwm_duty_ticks = Clamp_PWM_Ticks(duty);
}

static void Debug_Status1_Set(GPIO_PinState state)
{
  HAL_GPIO_WritePin(STATUS_1_GPIO_Port, STATUS_1_Pin, state);
}

static void Debug_Status2_Set(GPIO_PinState state)
{
  HAL_GPIO_WritePin(STATUS_2_GPIO_Port, STATUS_2_Pin, state);
}

static void Debug_Status2_ToggleSlow(void)
{
  static uint32_t last_toggle_tick = 0U;
  uint32_t now_tick = HAL_GetTick();

  if ((now_tick - last_toggle_tick) >= 500U)
  {
    last_toggle_tick = now_tick;
    HAL_GPIO_TogglePin(STATUS_2_GPIO_Port, STATUS_2_Pin);
  }
}

static void TIM1_Force_PWM_Outputs_Enabled(void)
{
  Motor_PWM_Enable();
}

void TIM1_Set_EdgeAligned_For_OpenLoopPWM(void)
{
  Motor_PWM_Off();
  __HAL_TIM_DISABLE(&htim1);
  TIM1->CR1 &= ~(TIM_CR1_CMS | TIM_CR1_DIR);
  TIM1->BDTR = (TIM1->BDTR & ~TIM_BDTR_DTG) | g_motor_deadtime_ticks;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  TIM1->EGR = TIM_EGR_UG;
  __HAL_TIM_ENABLE(&htim1);
  Motor_PWM_Enable();
}

void TIM1_Set_CenterAligned_For_SVPWM(void)
{
  Motor_PWM_Off();
  __HAL_TIM_DISABLE(&htim1);
  __HAL_TIM_SET_AUTORELOAD(&htim1, FOC_TIM1_PERIOD_TICKS);
  htim1.Init.Period = FOC_TIM1_PERIOD_TICKS;
  TIM1->CCR4 = FOC_TIM1_PERIOD_TICKS - CURRENT_SAMPLE_BEFORE_TIM1_PEAK_TICKS;
  Set_Phase_PWM_Ticks(FOC_TIM1_PERIOD_TICKS / 2U,
                      FOC_TIM1_PERIOD_TICKS / 2U,
                      FOC_TIM1_PERIOD_TICKS / 2U);
  TIM1->CR1 = (TIM1->CR1 & ~(TIM_CR1_CMS | TIM_CR1_DIR)) | TIM_COUNTERMODE_CENTERALIGNED1;
  TIM1->BDTR = (TIM1->BDTR & ~TIM_BDTR_DTG) | g_motor_deadtime_ticks;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  TIM1->EGR = TIM_EGR_UG;
  __HAL_TIM_ENABLE(&htim1);
  if (!CurrentSense_StartSynchronized())
  {
    Error_Handler();
  }
  Motor_PWM_Enable();
}

static void Set_Phase_PWM_Ticks(uint32_t phase_a, uint32_t phase_b, uint32_t phase_c)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);

  if (phase_a > arr)
  {
    phase_a = arr;
  }

  if (phase_b > arr)
  {
    phase_b = arr;
  }

  if (phase_c > arr)
  {
    phase_c = arr;
  }

  TIM1->CCR1 = phase_a;
  TIM1->CCR2 = phase_b;
  TIM1->CCR3 = phase_c;
}

static void Inverse_Park_Transform(float vd, float vq, float sin_theta, float cos_theta, float *v_alpha, float *v_beta)
{
  *v_alpha = vd * cos_theta - vq * sin_theta;
  *v_beta = vd * sin_theta + vq * cos_theta;
}

static void Set_AlphaBeta_SVPWM(float v_alpha, float v_beta)
{
  float va;
  float vb;
  float vc;
  float v_max;
  float v_min;
  float v_offset;
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);

  va = v_alpha;
  vb = -0.5f * v_alpha + (0.5f * SQRT3_F) * v_beta;
  vc = -0.5f * v_alpha - (0.5f * SQRT3_F) * v_beta;

  /* Direct comparisons compile to VFP compare/select instructions.  Calling
   * the C library min/max helpers here consumed a large part of the 25 us
   * current-loop budget. */
  v_max = va;
  if (vb > v_max) v_max = vb;
  if (vc > v_max) v_max = vc;
  v_min = va;
  if (vb < v_min) v_min = vb;
  if (vc < v_min) v_min = vc;
  v_offset = -0.5f * (v_max + v_min);

  va = ClampFloat(0.5f + va + v_offset, 0.0f, 1.0f);
  vb = ClampFloat(0.5f + vb + v_offset, 0.0f, 1.0f);
  vc = ClampFloat(0.5f + vc + v_offset, 0.0f, 1.0f);

  Set_Phase_PWM_Ticks((uint32_t)(va * (float)arr),
                      (uint32_t)(vb * (float)arr),
                      (uint32_t)(vc * (float)arr));
  /* HAL_GetTick() is useful for the slow status LED in open-loop tests, but
   * it does not belong in the FOC interrupt. */
  if (g_foc_enabled == 0U)
  {
    Debug_Status2_ToggleSlow();
  }
}

static void Set_DQ_SVPWM(float electrical_angle_rad, float vd, float vq)
{
  float sin_theta = sinf(electrical_angle_rad);
  float cos_theta = cosf(electrical_angle_rad);
  float magnitude = sqrtf((vd * vd) + (vq * vq));
  float v_alpha;
  float v_beta;

  if (magnitude > MOTOR_SVPWM_MAX_INDEX)
  {
    float scale = MOTOR_SVPWM_MAX_INDEX / magnitude;
    vd *= scale;
    vq *= scale;
  }

  Inverse_Park_Transform(vd, vq, sin_theta, cos_theta, &v_alpha, &v_beta);
  Set_AlphaBeta_SVPWM(v_alpha, v_beta);
}

static void Set_OpenLoop_SVPWM(float electrical_angle_rad, float modulation_index)
{
  modulation_index = ClampFloat(modulation_index, 0.0f, MOTOR_SVPWM_MAX_INDEX);

  /* Vd = 0, Vq = modulation_index. This makes a rotating voltage vector without encoder feedback. */
  Set_DQ_SVPWM(electrical_angle_rad, 0.0f, modulation_index);
}

/*
 * Hold the stator voltage vector at a directly specified electrical angle.
 * Set_OpenLoop_SVPWM() uses Vq, whose physical alpha/beta vector leads its
 * angle argument by 90 electrical degrees, so compensate for that here.
 */
static void Set_StatorField_SVPWM(float field_angle_rad, float modulation_index)
{
  Set_OpenLoop_SVPWM(WrapRadians(field_angle_rad - (0.5f * PI_F)),
                     modulation_index);
}

static void Motor_FieldOrientationDemo(uint8_t deadtime_ticks)
{
  uint32_t step;
  const float electrical_step_rad =
      TWO_PI_F / (float)MOTOR_FIELD_STEPS_PER_ELECTRICAL_REV;
  const float mechanical_step_deg =
      360.0f / ((float)MOTOR_FIELD_STEPS_PER_ELECTRICAL_REV *
                (float)MOTOR_POLE_PAIRS);

  g_motor_deadtime_ticks = deadtime_ticks;
  g_actual_electrical_hz = 0.0f;
  g_target_electrical_hz = 0.0f;
  g_open_loop_modulation = MOTOR_FIELD_STEP_MODULATION;

  TIM1_Set_CenterAligned_For_SVPWM();
  Set_StatorField_SVPWM(0.0f, 0.0f);

  printf("Static SVPWM field-orientation demonstration\r\n");
  printf("11 pole pairs: each %u-degree electrical step is approximately %ld.%02ld degrees mechanical\r\n",
         360U / MOTOR_FIELD_STEPS_PER_ELECTRICAL_REV,
         (long)mechanical_step_deg,
         (long)((mechanical_step_deg - (float)((long)mechanical_step_deg)) * 100.0f));
  printf("modulation_x10000=%u, hold_ms=%u, no software current limit\r\n",
         (unsigned int)(MOTOR_FIELD_STEP_MODULATION * 10000.0f),
         MOTOR_FIELD_STEP_HOLD_MS);

  /* Include the 360-degree endpoint. Its PWM vector equals step zero, but a
     following 11-pole-pair rotor has moved to the adjacent stable alignment. */
  for (step = 0U; step <= MOTOR_FIELD_STEPS_PER_ELECTRICAL_REV; ++step)
  {
    float field_angle_rad = (float)step * electrical_step_rad;
    uint32_t hold_start_tick;
    uint8_t current_printed = 0U;

    Set_StatorField_SVPWM(field_angle_rad, MOTOR_FIELD_STEP_MODULATION);
    printf("FIELD step=%lu/%u electrical_deg=%lu expected_mechanical_deg_x100=%lu\r\n",
           (unsigned long)step,
           MOTOR_FIELD_STEPS_PER_ELECTRICAL_REV,
           (unsigned long)((step * 360U) /
                           MOTOR_FIELD_STEPS_PER_ELECTRICAL_REV),
           (unsigned long)(step * mechanical_step_deg * 100.0f));

    hold_start_tick = HAL_GetTick();
    while ((HAL_GetTick() - hold_start_tick) < MOTOR_FIELD_STEP_HOLD_MS)
    {
      if (HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin) == GPIO_PIN_RESET)
      {
        Motor_Fault_Shutdown();
        printf("FIELD demo aborted: DRV8353S nFAULT asserted\r\n");
        return;
      }

      if (current_printed == 0U &&
          (HAL_GetTick() - hold_start_tick) >= 100U)
      {
        PhaseCurrents_t currents = CurrentSense_GetPhaseCurrents();
        printf("  settled_current_mA A=%ld B=%ld C=%ld sum=%ld\r\n",
               (long)(currents.ia_filtered * 1000.0f),
               (long)(currents.ib_filtered * 1000.0f),
               (long)(currents.ic_filtered * 1000.0f),
               (long)(currents.current_sum * 1000.0f));
        current_printed = 1U;
      }

      HAL_Delay(1U);
    }
  }

  Set_StatorField_SVPWM(0.0f, 0.0f);
  Motor_PWM_Off();
  printf("FIELD demo complete: expected travel=360/%u = 32.73 mechanical degrees; PWM inhibited\r\n",
         MOTOR_POLE_PAIRS);
}

static int32_t AS5048A_SignedDelta(uint16_t newer, uint16_t older)
{
  int32_t delta = (int32_t)newer - (int32_t)older;

  if (delta > 8191)
  {
    delta -= 16384;
  }
  else if (delta < -8192)
  {
    delta += 16384;
  }

  return delta;
}

static void AS5048A_EncoderTestLoop(void)
{
  uint16_t previous_count;
  int64_t accumulated_counts = 0;
  int32_t print_delta_counts = 0;
  uint32_t sample_tick;
  uint32_t print_tick;
  uint32_t sweep_start_tick = 0U;
  uint32_t read_errors = 0U;
  uint32_t diagnostic_errors = 0U;
  uint32_t comp_high_samples = 0U;
  uint32_t comp_low_samples = 0U;
  uint32_t cordic_overflow_samples = 0U;
  uint8_t agc_min = 255U;
  uint8_t agc_max = 0U;
  uint16_t magnitude_min = 0x3FFFU;
  uint16_t magnitude_max = 0U;
  uint8_t sweep_started = 0U;

  Motor_PWM_Off();
  HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
  AS5048A_InitChipSelect();

  printf("AS5048A magnetic-quality sweep; DRV disabled and PWM inhibited\r\n");
  printf("Slowly rotate the shaft one complete mechanical revolution in either direction\r\n");
  AS5048A_PrintTest();

  if (!AS5048A_ReadAngle(&previous_count))
  {
    printf("ERROR: initial AS5048A angle read failed\r\n");
    while (1)
    {
      AS5048A_PrintTest();
      HAL_Delay(1000U);
    }
  }

  printf("ENCODER_QUALITY_CSV_BEGIN\r\n");
  printf("time_ms,raw_count,angle_deg_x1000,delta_counts,total_counts,total_turns_x1000,rpm_x100,diag_raw,ocf,cof,comp_low,comp_high,agc,magnitude,read_errors,diagnostic_errors\r\n");
  sample_tick = HAL_GetTick();
  print_tick = sample_tick;

  while (accumulated_counts < 16384LL && accumulated_counts > -16384LL)
  {
    uint32_t now_tick = HAL_GetTick();

    if ((now_tick - sample_tick) >= ENCODER_TEST_INTERVAL_MS)
    {
      uint16_t count;

      sample_tick = now_tick;
      if (AS5048A_ReadAngle(&count))
      {
        int32_t delta = AS5048A_SignedDelta(count, previous_count);
        uint16_t diagnostic_response = 0U;
        uint16_t magnitude_response = 0U;
        uint16_t diagnostic_data = 0U;
        uint16_t magnitude = 0U;
        uint8_t diagnostic_ok = 0U;
        uint8_t ocf = 0U;
        uint8_t cof = 0U;
        uint8_t comp_low = 0U;
        uint8_t comp_high = 0U;
        uint8_t agc = 0U;

        previous_count = count;
        if (sweep_started == 0U && delta != 0)
        {
          sweep_started = 1U;
          sweep_start_tick = now_tick;
          print_tick = now_tick;
        }
        if (sweep_started != 0U)
        {
          accumulated_counts += delta;
          print_delta_counts += delta;
        }

        if (AS5048A_ReadRegister(AS5048A_REG_DIAGNOSTICS,
                                 &diagnostic_response) &&
            AS5048A_ReadRegister(AS5048A_REG_MAGNITUDE,
                                 &magnitude_response) &&
            AS5048A_ResponseParityOK(diagnostic_response) &&
            AS5048A_ResponseParityOK(magnitude_response) &&
            (diagnostic_response & 0x4000U) == 0U &&
            (magnitude_response & 0x4000U) == 0U)
        {
          diagnostic_data = diagnostic_response & 0x3FFFU;
          magnitude = magnitude_response & 0x3FFFU;
          comp_high = (diagnostic_data & (1U << 11)) != 0U;
          comp_low = (diagnostic_data & (1U << 10)) != 0U;
          cof = (diagnostic_data & (1U << 9)) != 0U;
          ocf = (diagnostic_data & (1U << 8)) != 0U;
          agc = (uint8_t)(diagnostic_data & 0xFFU);
          diagnostic_ok = 1U;

          if (agc < agc_min) agc_min = agc;
          if (agc > agc_max) agc_max = agc;
          if (magnitude < magnitude_min) magnitude_min = magnitude;
          if (magnitude > magnitude_max) magnitude_max = magnitude;
          if (comp_high != 0U) comp_high_samples++;
          if (comp_low != 0U) comp_low_samples++;
          if (cof != 0U) cordic_overflow_samples++;
        }
        else
        {
          diagnostic_errors++;
        }

        if (sweep_started != 0U &&
            (now_tick - print_tick) >= ENCODER_TEST_PRINT_INTERVAL_MS)
        {
          uint32_t print_elapsed_ms = now_tick - print_tick;
          uint32_t angle_deg_x1000 =
              (uint32_t)(((uint64_t)count * 360000ULL) / 16384ULL);
          int32_t total_turns_x1000 =
              (int32_t)((accumulated_counts * 1000LL) / 16384LL);
          int32_t rpm_x100 =
              (int32_t)(((int64_t)print_delta_counts * 6000000LL) /
                        (16384LL * (int64_t)print_elapsed_ms));

          printf("%lu,%u,%lu,%ld,%ld,%ld,%ld,0x%04X,%u,%u,%u,%u,%u,%u,%lu,%lu\r\n",
                 (unsigned long)(now_tick - sweep_start_tick),
                 count,
                 (unsigned long)angle_deg_x1000,
                 (long)print_delta_counts,
                 (long)accumulated_counts,
                 (long)total_turns_x1000,
                 (long)rpm_x100,
                 diagnostic_response,
                 ocf, cof, comp_low, comp_high,
                 diagnostic_ok != 0U ? agc : 0U,
                 diagnostic_ok != 0U ? magnitude : 0U,
                 (unsigned long)read_errors,
                 (unsigned long)diagnostic_errors);
          print_tick = now_tick;
          print_delta_counts = 0;
        }
      }
      else
      {
        read_errors++;
        printf("ENCODER_READ_ERROR,%lu,total=%lu\r\n",
               (unsigned long)now_tick,
               (unsigned long)read_errors);
      }

    }
  }

  printf("ENCODER_QUALITY_CSV_END\r\n");
  printf("ENCODER_QUALITY_SUMMARY duration_ms=%lu total_counts=%ld read_errors=%lu diagnostic_errors=%lu agc_min=%u agc_max=%u magnitude_min=%u magnitude_max=%u comp_low_samples=%lu comp_high_samples=%lu cordic_overflow_samples=%lu\r\n",
         (unsigned long)(HAL_GetTick() - sweep_start_tick),
         (long)accumulated_counts,
         (unsigned long)read_errors,
         (unsigned long)diagnostic_errors,
         agc_min, agc_max, magnitude_min, magnitude_max,
         (unsigned long)comp_low_samples,
         (unsigned long)comp_high_samples,
         (unsigned long)cordic_overflow_samples);
  printf("Encoder sweep complete; DRV remains disabled and PWM inhibited\r\n");
  while (1)
  {
  }
}

static float FOC_ElectricalAngleFromEncoder(uint16_t encoder_count)
{
  int32_t mechanical_delta =
      AS5048A_SignedDelta(encoder_count, g_foc_encoder_zero_count);
  float electrical_angle =
      ((float)(g_foc_encoder_direction * mechanical_delta * (int32_t)MOTOR_POLE_PAIRS) *
       TWO_PI_F) / 16384.0f;

  return WrapRadians(electrical_angle);
}

static void FOC_PublishEncoderObservation(float electrical_angle_rad,
                                          float electrical_velocity_rad_s,
                                          uint32_t observation_cycles)
{
  uint32_t interrupt_state = __get_PRIMASK();

  /* Publish one coherent angle/velocity/timestamp set to the ADC ISR. */
  __disable_irq();
  g_foc_encoder_observed_angle_rad = electrical_angle_rad;
  g_foc_electrical_velocity_rad_s = electrical_velocity_rad_s;
  g_foc_encoder_observation_cycles = observation_cycles;
  __DMB();
  g_foc_encoder_observation_sequence++;
  if (interrupt_state == 0U)
  {
    __enable_irq();
  }
}

#if FOC_PREFAULT_CAPTURE_ENABLE
static inline void FOC_PreFaultCapture(const PhaseCurrents_t *currents,
                                       float id, float iq)
{
  FOC_PreFaultSample *sample =
      &g_foc_prefault_log[g_foc_prefault_write_index];

  sample->sequence = g_foc_prefault_sequence++;
  sample->raw_a = currents->raw_a;
  sample->raw_b = currents->raw_b;
  sample->raw_c = currents->raw_c;
  sample->ccr_a = (uint16_t)TIM1->CCR1;
  sample->ccr_b = (uint16_t)TIM1->CCR2;
  sample->ccr_c = (uint16_t)TIM1->CCR3;
  sample->ccr_trigger = (uint16_t)TIM1->CCR4;
  sample->ia = currents->ia;
  sample->ib = currents->ib;
  sample->ib_sensed_corrected = currents->ib_sensed_corrected;
  sample->ic = currents->ic;
  sample->id = id;
  sample->iq = iq;
  sample->electrical_angle_rad = g_foc_electrical_angle_rad;
  sample->encoder_raw_angle_rad = g_foc_encoder_raw_angle_rad;
  sample->encoder_innovation_rad = g_foc_encoder_innovation_rad;
  sample->encoder_correction_rad = g_foc_encoder_correction_rad;
  sample->encoder_velocity_rad_s = g_foc_electrical_velocity_rad_s;
  /* These are the modulation commands that produced the sampled PWM cycle. */
  sample->vd_previous = g_foc_vd_modulation;
  sample->vq_previous = g_foc_vq_modulation;
  sample->valid_mask =
      (currents->valid_a != 0U ? 1U : 0U) |
      (currents->valid_b != 0U ? 2U : 0U) |
      (currents->valid_c != 0U ? 4U : 0U);

  g_foc_prefault_write_index++;
  if (g_foc_prefault_write_index >= FOC_PREFAULT_LOG_CAPACITY)
  {
    g_foc_prefault_write_index = 0U;
  }
  if (g_foc_prefault_count < FOC_PREFAULT_LOG_CAPACITY)
  {
    g_foc_prefault_count++;
  }
}

static void FOC_PreFaultDump(void)
{
  uint8_t index;
  uint8_t remaining;

  if (g_foc_prefault_count == 0U)
  {
    return;
  }

  index = (g_foc_prefault_count < FOC_PREFAULT_LOG_CAPACITY) ? 0U :
          g_foc_prefault_write_index;
  remaining = g_foc_prefault_count;
  printf("FOC_PREFAULT_CSV_BEGIN\r\n");
  printf("sequence,raw_a,raw_b,raw_c,ia_mA,ib_control_mA,ib_sensed_corrected_mA,ic_mA,id_mA,iq_mA,electrical_angle_mrad,encoder_raw_angle_mrad,encoder_innovation_mrad,encoder_correction_mrad,encoder_velocity_rad_s,ccr_a,ccr_b,ccr_c,ccr_trigger,vd_previous_x10000,vq_previous_x10000,valid_mask\r\n");
  while (remaining-- != 0U)
  {
    const FOC_PreFaultSample *sample = &g_foc_prefault_log[index];
    printf("%lu,%u,%u,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%u,%u,%u,%u,%ld,%ld,0x%X\r\n",
           (unsigned long)sample->sequence,
           sample->raw_a, sample->raw_b, sample->raw_c,
           (long)(sample->ia * 1000.0f),
           (long)(sample->ib * 1000.0f),
           (long)(sample->ib_sensed_corrected * 1000.0f),
           (long)(sample->ic * 1000.0f),
           (long)(sample->id * 1000.0f),
           (long)(sample->iq * 1000.0f),
           (long)(sample->electrical_angle_rad * 1000.0f),
           (long)(sample->encoder_raw_angle_rad * 1000.0f),
           (long)(sample->encoder_innovation_rad * 1000.0f),
           (long)(sample->encoder_correction_rad * 1000.0f),
           (long)sample->encoder_velocity_rad_s,
           sample->ccr_a, sample->ccr_b, sample->ccr_c,
           sample->ccr_trigger,
           (long)(sample->vd_previous * 10000.0f),
           (long)(sample->vq_previous * 10000.0f),
           sample->valid_mask);
    index++;
    if (index >= FOC_PREFAULT_LOG_CAPACITY)
    {
      index = 0U;
    }
  }
  printf("FOC_PREFAULT_CSV_END\r\n");
}
#endif

static void FOC_CurrentLoopISR(const PhaseCurrents_t *currents)
{
  uint32_t observation_sequence;
  uint32_t prediction_age_cycles;
  uint32_t prediction_max_cycles;
  uint8_t valid_count;
  float sin_theta;
  float cos_theta;
  float i_alpha;
  float i_beta;
  float id;
  float iq;
  float id_error;
  float iq_error;
  float vd_unsaturated;
  float vq_unsaturated;
  float vd;
  float vq;
  float magnitude;
  float magnitude_squared;
  float max_abs_current;

  observation_sequence = g_foc_encoder_observation_sequence;
  if (observation_sequence != 0U)
  {
    /* The published timestamp is when the delayed sensor sample represents
     * the shaft, not when the SPI read completed. Recompute from that fixed
     * observation on every current-loop iteration so SPI timing cannot create an
     * angle staircase or cumulative integration drift. */
    prediction_age_cycles = DWT->CYCCNT - g_foc_encoder_observation_cycles;
    prediction_max_cycles =
        (SystemCoreClock / 1000000U) * FOC_ENCODER_PREDICTION_MAX_US;
    if (prediction_age_cycles > prediction_max_cycles)
    {
      /* A frozen electrical angle is much more dangerous than coasting.  At
       * high speed it rotates the commanded voltage away from q-axis and can
       * create a large real phase-current surge within a few PWM periods. */
      g_foc_fault = 8U;
      g_foc_enabled = 0U;
      Motor_PWM_Off();
      return;
    }
    g_foc_electrical_angle_rad =
        WrapRadians(g_foc_encoder_observed_angle_rad +
                    (g_foc_electrical_velocity_rad_s *
                     ((float)prediction_age_cycles *
                      g_foc_seconds_per_core_cycle)));
  }

  if (currents == NULL)
  {
    g_foc_fault = 1U;
    g_foc_enabled = 0U;
    Motor_PWM_Off();
    return;
  }

  valid_count = currents->valid_a + currents->valid_b + currents->valid_c;
  if (valid_count < 2U ||
      (currents->valid_a != 0U && currents->saturated_a != 0U) ||
      (currents->valid_b != 0U && currents->saturated_b != 0U) ||
      (currents->valid_c != 0U && currents->saturated_c != 0U))
  {
    g_foc_fault = 1U;
    g_foc_enabled = 0U;
    Motor_PWM_Off();
    return;
  }

  FOC_FastSinCos(g_foc_electrical_angle_rad, &sin_theta, &cos_theta);

  /* Amplitude-invariant Clarke and Park transforms. */
  i_alpha = (float)g_foc_current_polarity * currents->ia;
  i_beta = (float)g_foc_current_polarity *
           (currents->ia + (2.0f * currents->ib)) * (1.0f / SQRT3_F);
  id = (i_alpha * cos_theta) + (i_beta * sin_theta);
  iq = (-i_alpha * sin_theta) + (i_beta * cos_theta);

#if FOC_PREFAULT_CAPTURE_ENABLE
  /* Optional high-rate diagnostic. Keep it out of normal runs so the encoder
   * foreground task receives as much CPU time as possible. */
  FOC_PreFaultCapture(currents, id, iq);
#endif

  /* A large d/q vector is already an abnormal control condition. Stop well
   * before it can reach the separate active-mode phase hard limit. */
  if (AbsFloat(id) > g_foc_active_dq_fault_limit_a ||
      AbsFloat(iq) > g_foc_active_dq_fault_limit_a)
  {
    g_foc_fault_max_current_a =
        (AbsFloat(id) > AbsFloat(iq)) ? AbsFloat(id) : AbsFloat(iq);
    g_foc_fault_ia_a = currents->ia;
    g_foc_fault_ib_a = currents->ib;
    g_foc_fault_ic_a = currents->ic;
    g_foc_fault_id_a = id;
    g_foc_fault_iq_a = iq;
    g_foc_fault_electrical_angle_rad = g_foc_electrical_angle_rad;
    g_foc_fault_valid_mask =
        (currents->valid_a != 0U ? 1U : 0U) |
        (currents->valid_b != 0U ? 2U : 0U) |
        (currents->valid_c != 0U ? 4U : 0U);
    g_foc_fault = 9U;
    g_foc_enabled = 0U;
    Motor_PWM_Off();
    return;
  }

  max_abs_current = AbsFloat(currents->ia);
  if (AbsFloat(currents->ib) > max_abs_current)
  {
    max_abs_current = AbsFloat(currents->ib);
  }
  if (AbsFloat(currents->ic) > max_abs_current)
  {
    max_abs_current = AbsFloat(currents->ic);
  }
  if (currents->valid_b != 0U &&
      AbsFloat(currents->ib_sensed) > max_abs_current)
  {
    max_abs_current = AbsFloat(currents->ib_sensed);
  }
  if (currents->valid_c != 0U &&
      AbsFloat(currents->ic_sensed) > max_abs_current)
  {
    max_abs_current = AbsFloat(currents->ic_sensed);
  }
  if (max_abs_current > g_foc_active_hard_current_limit_a)
  {
    if (g_foc_overcurrent_sample_count < 255U)
    {
      g_foc_overcurrent_sample_count++;
    }
    if (g_foc_overcurrent_sample_count >= FOC_OVERCURRENT_CONFIRM_SAMPLES)
    {
      g_foc_fault_max_current_a = max_abs_current;
      g_foc_fault_ia_a = currents->ia;
      g_foc_fault_ib_a = currents->ib;
      g_foc_fault_ic_a = currents->ic;
      g_foc_fault_id_a = id;
      g_foc_fault_iq_a = iq;
      g_foc_fault_electrical_angle_rad = g_foc_electrical_angle_rad;
      g_foc_fault_valid_mask =
          (currents->valid_a != 0U ? 1U : 0U) |
          (currents->valid_b != 0U ? 2U : 0U) |
          (currents->valid_c != 0U ? 4U : 0U);
      g_foc_fault = 2U;
      g_foc_enabled = 0U;
      Motor_PWM_Off();
      return;
    }
  }
  else
  {
    g_foc_overcurrent_sample_count = 0U;
  }

  id_error = g_foc_id_reference_a - id;
  iq_error = g_foc_iq_reference_a - iq;
  g_foc_id_integrator = ClampFloat(g_foc_id_integrator +
                                    (FOC_CURRENT_KI * FOC_CURRENT_LOOP_DT_S * id_error),
                                    -FOC_MAX_MODULATION, FOC_MAX_MODULATION);
  g_foc_iq_integrator = ClampFloat(g_foc_iq_integrator +
                                    (FOC_CURRENT_KI * FOC_CURRENT_LOOP_DT_S * iq_error),
                                    -FOC_MAX_MODULATION, FOC_MAX_MODULATION);
  vd_unsaturated = (FOC_CURRENT_KP * id_error) + g_foc_id_integrator;
  vq_unsaturated = (FOC_CURRENT_KP * iq_error) + g_foc_iq_integrator;
  vd = vd_unsaturated;
  vq = vq_unsaturated;

  magnitude_squared = (vd * vd) + (vq * vq);
  if (magnitude_squared > (FOC_MAX_MODULATION * FOC_MAX_MODULATION))
  {
    magnitude = sqrtf(magnitude_squared);
    float scale = FOC_MAX_MODULATION / magnitude;
    vd *= scale;
    vq *= scale;

    /* Track the voltage vector that SVPWM can actually apply. Without this
     * back-calculation, the independent D/Q integrators can remain wound up
     * outside the circular modulation limit and drive a large Id transient. */
    g_foc_id_integrator = ClampFloat(
        g_foc_id_integrator + (vd - vd_unsaturated),
        -FOC_MAX_MODULATION, FOC_MAX_MODULATION);
    g_foc_iq_integrator = ClampFloat(
        g_foc_iq_integrator + (vq - vq_unsaturated),
        -FOC_MAX_MODULATION, FOC_MAX_MODULATION);
  }

  g_foc_id_a = id;
  g_foc_iq_a = iq;
  g_foc_vd_modulation = vd;
  g_foc_vq_modulation = vq;
  g_foc_telemetry_id_sum += id;
  g_foc_telemetry_iq_sum += iq;
  g_foc_telemetry_phase_square_sum +=
      (currents->ia * currents->ia) +
      (currents->ib * currents->ib) +
      (currents->ic * currents->ic);
  {
    float phase_peak = AbsFloat(currents->ia);
    if (AbsFloat(currents->ib) > phase_peak)
    {
      phase_peak = AbsFloat(currents->ib);
    }
    if (AbsFloat(currents->ic) > phase_peak)
    {
      phase_peak = AbsFloat(currents->ic);
    }
    if (phase_peak > g_foc_telemetry_phase_peak_a)
    {
      g_foc_telemetry_phase_peak_a = phase_peak;
    }
  }
  g_foc_telemetry_sample_count++;
  /* Reuse the Park-transform sine/cosine pair; recomputing it in
   * Set_DQ_SVPWM() consumed too much of the 25 us control-loop budget. */
  {
    float v_alpha;
    float v_beta;

    Inverse_Park_Transform(vd, vq, sin_theta, cos_theta, &v_alpha, &v_beta);
    Set_AlphaBeta_SVPWM(v_alpha, v_beta);
  }
}

static bool FOC_AlignmentHold(float field_angle_rad, uint32_t hold_ms)
{
  uint32_t start_tick = HAL_GetTick();

  Set_StatorField_SVPWM(field_angle_rad, FOC_ALIGNMENT_MODULATION);
  while ((HAL_GetTick() - start_tick) < hold_ms)
  {
    PhaseCurrents_t currents = CurrentSense_GetPhaseCurrents();
    float max_current = fmaxf(AbsFloat(currents.ia),
                              fmaxf(AbsFloat(currents.ib), AbsFloat(currents.ic)));
    max_current = fmaxf(max_current, AbsFloat(currents.ib_sensed));
    max_current = fmaxf(max_current, AbsFloat(currents.ic_sensed));

    if (max_current > FOC_HARD_CURRENT_LIMIT_A)
    {
      printf("FOC alignment aborted: phase current %ld mA exceeds %ld mA\r\n",
             (long)(max_current * 1000.0f),
             (long)(FOC_HARD_CURRENT_LIMIT_A * 1000.0f));
      return false;
    }
    if (HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin) == GPIO_PIN_RESET)
    {
      printf("FOC alignment aborted: DRV8353S nFAULT asserted\r\n");
      return false;
    }
    HAL_Delay(1U);
  }

  return true;
}

static void FOC_LogSampleCapture(uint32_t elapsed_ms, float mechanical_rpm)
{
  PhaseCurrents_t currents;
  FOC_LogSample *sample;
  float id_sum;
  float iq_sum;
  float phase_square_sum;
  float phase_peak_a;
  uint32_t telemetry_count;
  uint32_t current_isr_max_cycles;
  uint32_t primask;

  if (g_foc_log_count >= FOC_LOG_CAPACITY)
  {
    return;
  }

  currents = CurrentSense_GetPhaseCurrents();
  primask = __get_PRIMASK();
  __disable_irq();
  id_sum = g_foc_telemetry_id_sum;
  iq_sum = g_foc_telemetry_iq_sum;
  phase_square_sum = g_foc_telemetry_phase_square_sum;
  phase_peak_a = g_foc_telemetry_phase_peak_a;
  telemetry_count = g_foc_telemetry_sample_count;
  current_isr_max_cycles = g_foc_current_isr_max_cycles;
  g_foc_telemetry_id_sum = 0.0f;
  g_foc_telemetry_iq_sum = 0.0f;
  g_foc_telemetry_phase_square_sum = 0.0f;
  g_foc_telemetry_phase_peak_a = 0.0f;
  g_foc_telemetry_sample_count = 0U;
  g_foc_current_isr_max_cycles = 0U;
  __set_PRIMASK(primask);

  sample = &g_foc_log[g_foc_log_count++];
  sample->time_ms = (uint16_t)elapsed_ms;
  sample->raw_a = currents.raw_a;
  sample->raw_b = currents.raw_b;
  sample->raw_c = currents.raw_c;
  sample->ia_ma = (int32_t)(currents.ia * 1000.0f);
  sample->ib_ma = (int32_t)(currents.ib * 1000.0f);
  sample->ib_sensed_ma = (int32_t)(currents.ib_sensed * 1000.0f);
  sample->ib_sensed_corrected_ma =
      (int32_t)(currents.ib_sensed_corrected * 1000.0f);
  sample->ic_ma = (int32_t)(currents.ic * 1000.0f);
  sample->current_sum_ma = (int32_t)(currents.current_sum * 1000.0f);
  sample->sensed_current_sum_ma =
      (int32_t)((currents.ia + currents.ib_sensed + currents.ic_sensed) * 1000.0f);
  sample->id_ma = (int32_t)(g_foc_id_a * 1000.0f);
  sample->iq_ma = (int32_t)(g_foc_iq_a * 1000.0f);
  sample->electrical_angle_mrad =
      (int32_t)(g_foc_electrical_angle_rad * 1000.0f);
  sample->electrical_velocity_mrad_s =
      (int32_t)(g_foc_electrical_velocity_rad_s * 1000.0f);
  sample->mechanical_rpm_x100 = (int32_t)(mechanical_rpm * 100.0f);
  sample->mechanical_rpm_window_x100 =
      (int32_t)(g_foc_debug_mechanical_rpm_window * 100.0f);
  sample->speed_reference_rpm_x100 =
      (int32_t)(g_foc_speed_reference_rpm * 100.0f);
  sample->speed_error_rpm_x100 =
      (int32_t)(g_foc_debug_speed_error_rpm * 100.0f);
  sample->mechanical_position_mdeg =
      (int32_t)(((int64_t)g_foc_mechanical_position_counts * 360000LL) /
                16384LL);
  sample->output_position_mdeg =
      (int32_t)((((float)g_foc_mechanical_position_counts * 360000.0f) /
                 16384.0f) * FOC_OUTPUT_DIRECTION_SIGN /
                FOC_MOTOR_TO_OUTPUT_GEAR_RATIO);
  sample->trajectory_position_mdeg =
      (int32_t)(g_foc_output_trajectory_position_deg * 1000.0f);
  sample->position_error_mdeg =
      (int32_t)(g_foc_debug_position_error_deg * 1000.0f);
  sample->position_target_mdeg =
      (int32_t)(g_foc_output_position_target_deg * 1000.0f);
  sample->encoder_window_counts = g_foc_debug_encoder_window_counts;
  sample->encoder_innovation_mrad =
      (int32_t)(g_foc_debug_encoder_innovation_rad * 1000.0f);
  sample->encoder_correction_mrad =
      (int32_t)(g_foc_debug_encoder_correction_rad * 1000.0f);
  sample->encoder_age_us =
      (uint32_t)(((uint64_t)(DWT->CYCCNT -
                             g_foc_encoder_observation_cycles) * 1000000ULL) /
                 (uint64_t)SystemCoreClock);
  sample->id_reference_ma = (int32_t)(g_foc_id_reference_a * 1000.0f);
  sample->iq_reference_ma = (int32_t)(g_foc_iq_reference_a * 1000.0f);
  sample->speed_integrator_ma =
      (int32_t)(g_foc_debug_speed_integrator_a * 1000.0f);
  sample->breakaway_current_ma =
      (int32_t)(g_foc_debug_breakaway_current_a * 1000.0f);
  sample->vd_x10000 = (int32_t)(g_foc_vd_modulation * 10000.0f);
  sample->vq_x10000 = (int32_t)(g_foc_vq_modulation * 10000.0f);
  if (telemetry_count != 0U)
  {
    sample->id_average_ma =
        (int32_t)((id_sum / (float)telemetry_count) * 1000.0f);
    sample->iq_average_ma =
        (int32_t)((iq_sum / (float)telemetry_count) * 1000.0f);
    sample->phase_rms_ma =
        (int32_t)(sqrtf(phase_square_sum / (3.0f * (float)telemetry_count)) *
                  1000.0f);
    sample->phase_peak_ma = (int32_t)(phase_peak_a * 1000.0f);
  }
  else
  {
    sample->id_average_ma = 0;
    sample->iq_average_ma = 0;
    sample->phase_rms_ma = 0;
    sample->phase_peak_ma = 0;
  }
  sample->current_samples = telemetry_count;
  sample->current_isr_max_cycles = current_isr_max_cycles;
  sample->valid_a = currents.valid_a;
  sample->valid_b = currents.valid_b;
  sample->valid_c = currents.valid_c;
  sample->iq_saturated = g_foc_debug_iq_saturated;
}

static void FOC_LogDump(void)
{
  uint8_t i;

  printf("FOC_CSV_BEGIN\r\n");
  printf("time_ms,raw_a,raw_b,raw_c,ia_mA,ib_control_mA,ib_sensed_raw_mA,ib_sensed_corrected_mA,ic_mA,reconstructed_sum_mA,sensed_sum_mA,valid_a,valid_b,valid_c,id_mA,iq_mA,electrical_angle_mrad,electrical_velocity_mrad_s,motor_rpm_filtered_x100,motor_rpm_window_x100,motor_speed_reference_rpm_x100,motor_speed_error_rpm_x100,motor_position_mdeg,output_position_mdeg,output_trajectory_position_mdeg,output_trajectory_error_mdeg,output_position_target_mdeg,encoder_window_counts,encoder_innovation_mrad,encoder_correction_mrad,encoder_age_us,id_reference_mA,iq_reference_mA,outer_integrator_mA,breakaway_current_mA,iq_saturated,vd_x10000,vq_x10000,id_average_mA,iq_average_mA,phase_rms_mA,phase_peak_mA,current_samples,current_isr_max_cycles\r\n");
  for (i = 0U; i < g_foc_log_count; ++i)
  {
    const FOC_LogSample *sample = &g_foc_log[i];

    printf("%u,%u,%u,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%u,%u,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%lu,%ld,%ld,%ld,%ld,%u,%ld,%ld,%ld,%ld,%ld,%ld,%lu,%lu\r\n",
           sample->time_ms,
           sample->raw_a, sample->raw_b, sample->raw_c,
           (long)sample->ia_ma, (long)sample->ib_ma,
           (long)sample->ib_sensed_ma,
           (long)sample->ib_sensed_corrected_ma,
           (long)sample->ic_ma,
           (long)sample->current_sum_ma, (long)sample->sensed_current_sum_ma,
           sample->valid_a, sample->valid_b, sample->valid_c,
           (long)sample->id_ma, (long)sample->iq_ma,
           (long)sample->electrical_angle_mrad,
           (long)sample->electrical_velocity_mrad_s,
           (long)sample->mechanical_rpm_x100,
           (long)sample->mechanical_rpm_window_x100,
           (long)sample->speed_reference_rpm_x100,
           (long)sample->speed_error_rpm_x100,
           (long)sample->mechanical_position_mdeg,
           (long)sample->output_position_mdeg,
           (long)sample->trajectory_position_mdeg,
           (long)sample->position_error_mdeg,
           (long)sample->position_target_mdeg,
           (long)sample->encoder_window_counts,
           (long)sample->encoder_innovation_mrad,
           (long)sample->encoder_correction_mrad,
           (unsigned long)sample->encoder_age_us,
           (long)sample->id_reference_ma,
           (long)sample->iq_reference_ma,
           (long)sample->speed_integrator_ma,
           (long)sample->breakaway_current_ma,
           sample->iq_saturated,
           (long)sample->vd_x10000, (long)sample->vq_x10000,
           (long)sample->id_average_ma, (long)sample->iq_average_ma,
           (long)sample->phase_rms_ma, (long)sample->phase_peak_ma,
           (unsigned long)sample->current_samples,
           (unsigned long)sample->current_isr_max_cycles);
  }
  printf("FOC_CSV_END\r\n");
}

static void Motor_FOC_Demo(uint8_t deadtime_ticks)
{
  uint16_t angle_zero_first;
  uint16_t angle_quarter;
  uint16_t angle_zero_final;
  uint16_t previous_angle;
  uint32_t start_tick;
  uint32_t last_encoder_cycles;
  uint32_t last_encoder_observation_cycles;
  uint32_t speed_window_start_cycles;
  uint32_t last_log_tick;
  uint32_t encoder_update_cycles;
  uint32_t speed_window_cycles;
  uint32_t encoder_duplicate_samples = 0U;
  uint32_t encoder_rejected_samples = 0U;
  uint32_t encoder_limited_corrections = 0U;
  uint32_t encoder_service_count = 0U;
  uint32_t encoder_max_service_interval_cycles = 0U;
  uint32_t encoder_consecutive_rejections = 0U;
  float encoder_max_rejected_innovation_rad = 0.0f;
  int32_t speed_window_counts = 0;
  uint8_t encoder_errors = 0U;
  float mechanical_rpm_filtered = 0.0f;
  float mechanical_rpm_window = 0.0f;
  float foc_iq_limit_a = FOC_IQ_TARGET_A;
#if FOC_POSITION_DEMO_ENABLE
  const uint32_t foc_test_duration_ms = FOC_POSITION_TEST_DURATION_MS;
  const uint32_t foc_log_interval_ms = FOC_POSITION_LOG_INTERVAL_MS;
  const uint32_t foc_min_current_samples_per_log =
      FOC_POSITION_MIN_CURRENT_SAMPLES_PER_LOG;
  const float foc_overspeed_rpm = FOC_POSITION_OVERSPEED_RPM;
  uint32_t position_hold_start_tick = 0U;
  uint32_t position_step_start_tick = 0U;
  float output_trajectory_speed_rpm = 0.0f;
  uint8_t position_target_index = 0U;
  uint8_t position_demo_completed = 0U;
  const uint8_t position_target_count =
      (uint8_t)(sizeof(g_foc_output_position_demo_targets_deg) /
                sizeof(g_foc_output_position_demo_targets_deg[0]));
#elif FOC_CURRENT_STEP_TEST_ENABLE
  const uint32_t foc_test_duration_ms = FOC_CURRENT_STEP_TEST_DURATION_MS;
  const uint32_t foc_log_interval_ms = FOC_CURRENT_STEP_LOG_INTERVAL_MS;
  const uint32_t foc_min_current_samples_per_log =
      FOC_CURRENT_STEP_MIN_CURRENT_SAMPLES;
  const float foc_overspeed_rpm = FOC_CURRENT_STEP_OVERSPEED_RPM;
#else
#if FOC_LOW_SPEED_VELOCITY_TEST_ENABLE
  const uint32_t foc_test_duration_ms = FOC_LOW_SPEED_TEST_DURATION_MS;
  const uint32_t foc_log_interval_ms = FOC_LOW_SPEED_LOG_INTERVAL_MS;
  const uint32_t foc_min_current_samples_per_log =
      FOC_LOW_SPEED_MIN_CURRENT_SAMPLES;
  const float foc_overspeed_rpm = FOC_LOW_SPEED_OVERSPEED_RPM;
  const float foc_speed_target_rpm = FOC_LOW_SPEED_TARGET_RPM;
  const float foc_speed_kp_a_per_rpm = FOC_LOW_SPEED_KP_A_PER_RPM;
  const float foc_speed_ki_a_per_rpm_s = FOC_LOW_SPEED_KI_A_PER_RPM_S;
  const float foc_speed_integral_limit_a =
      FOC_LOW_SPEED_INTEGRAL_LIMIT_A;
#else
  const uint32_t foc_test_duration_ms = FOC_TEST_DURATION_MS;
  const uint32_t foc_log_interval_ms = FOC_LOG_INTERVAL_MS;
  const uint32_t foc_min_current_samples_per_log =
      FOC_MIN_CURRENT_SAMPLES_PER_LOG;
  const float foc_overspeed_rpm = FOC_MAX_MECHANICAL_RPM;
  const float foc_speed_target_rpm = FOC_SPEED_TARGET_RPM;
  const float foc_speed_kp_a_per_rpm = FOC_SPEED_KP_A_PER_RPM;
  const float foc_speed_ki_a_per_rpm_s = FOC_SPEED_KI_A_PER_RPM_S;
  const float foc_speed_integral_limit_a = FOC_IQ_TARGET_A;
#endif
  float speed_integrator_a = 0.0f;
#endif
  int32_t direction_delta;

  g_foc_enabled = 0U;
  g_foc_fault = 0U;
  g_foc_b_pwm_bias_a = 0.0f;
  g_foc_b_bias_correction_enabled = 0U;
  g_motor_deadtime_ticks = deadtime_ticks;
  FOC_SineLUTInit();
  TIM1_Set_CenterAligned_For_SVPWM();
  Set_StatorField_SVPWM(0.0f, 0.0f);

  printf("FOC center-aligned PWM frequency=%lu Hz\r\n",
         (unsigned long)Get_PWM_FrequencyHz());
  printf("FOC ADC trigger=%lu ticks before PWM peak, valid-duty ceiling=%lu/10000\r\n",
         (unsigned long)CURRENT_SAMPLE_BEFORE_TIM1_PEAK_TICKS,
         (unsigned long)(((FOC_TIM1_PERIOD_TICKS -
                           CURRENT_SAMPLE_BEFORE_TIM1_PEAK_TICKS -
                           CURRENT_MIN_VALID_WINDOW_TICKS) * 10000U) /
                         FOC_TIM1_PERIOD_TICKS));

  printf("FOC encoder alignment: field 0 electrical degrees\r\n");
  if (!FOC_AlignmentHold(0.0f, FOC_ALIGNMENT_HOLD_MS) ||
      !AS5048A_ReadAngle(&angle_zero_first))
  {
    printf("FOC startup failed at initial encoder alignment\r\n");
    goto foc_stop;
  }

  printf("FOC encoder direction test: field +90 electrical degrees\r\n");
  if (!FOC_AlignmentHold(0.5f * PI_F, FOC_ALIGNMENT_HOLD_MS) ||
      !AS5048A_ReadAngle(&angle_quarter))
  {
    printf("FOC startup failed during encoder direction test\r\n");
    goto foc_stop;
  }

  direction_delta = AS5048A_SignedDelta(angle_quarter, angle_zero_first);
  if (direction_delta > -64 && direction_delta < 64)
  {
    printf("FOC startup failed: encoder moved only %ld counts; expected approximately %lu\r\n",
           (long)direction_delta,
           (unsigned long)(16384U / (4U * MOTOR_POLE_PAIRS)));
    goto foc_stop;
  }
  if ((FOC_VERIFIED_ENCODER_DIRECTION > 0 && direction_delta < 0) ||
      (FOC_VERIFIED_ENCODER_DIRECTION < 0 && direction_delta > 0))
  {
    printf("FOC startup failed: direction test moved %ld counts, opposite verified encoder direction %s\r\n",
           (long)direction_delta,
           (FOC_VERIFIED_ENCODER_DIRECTION > 0) ? "+1" : "-1");
    goto foc_stop;
  }
  g_foc_encoder_direction = FOC_VERIFIED_ENCODER_DIRECTION;
  printf("FOC encoder direction=%s (verified), +90 electrical moved %ld counts\r\n",
         (g_foc_encoder_direction > 0) ? "+1" : "-1",
         (long)direction_delta);

  printf("FOC returning field to electrical zero\r\n");
  if (!FOC_AlignmentHold(0.0f, FOC_ALIGNMENT_HOLD_MS) ||
      !AS5048A_ReadAngle(&angle_zero_final))
  {
    printf("FOC startup failed while establishing final electrical zero\r\n");
    goto foc_stop;
  }

  {
    PhaseCurrents_t aligned_currents = {0};
    float ia_sum = 0.0f;
    float ib_sum = 0.0f;
    float ib_sensed_sum = 0.0f;
    float ic_sum = 0.0f;
    uint32_t aligned_sample_count = 0U;
    uint32_t sample_index;

    for (sample_index = 0U;
         sample_index < FOC_ALIGNMENT_CURRENT_AVG_SAMPLES;
         ++sample_index)
    {
      PhaseCurrents_t sample = CurrentSense_GetPhaseCurrents();

      if (sample.valid_a != 0U && sample.valid_c != 0U)
      {
        ia_sum += sample.ia;
        ib_sum += sample.ib;
        ib_sensed_sum += sample.ib_sensed;
        ic_sum += sample.ic;
        aligned_sample_count++;
      }
      HAL_Delay(1U);
    }

    if (aligned_sample_count == 0U)
    {
      printf("FOC startup failed: no valid A/C alignment-current samples\r\n");
      goto foc_stop;
    }

    aligned_currents.ia = ia_sum / (float)aligned_sample_count;
    aligned_currents.ib = ib_sum / (float)aligned_sample_count;
    aligned_currents.ib_sensed =
        ib_sensed_sum / (float)aligned_sample_count;
    aligned_currents.ic = ic_sum / (float)aligned_sample_count;
    g_foc_b_pwm_bias_a = aligned_currents.ib_sensed - aligned_currents.ib;
    if (AbsFloat(g_foc_b_pwm_bias_a) > 5.0f)
    {
      printf("FOC startup failed: phase-B PWM bias %ld mA is outside +/-5000 mA\r\n",
             (long)(g_foc_b_pwm_bias_a * 1000.0f));
      goto foc_stop;
    }

    if (AbsFloat(aligned_currents.ia) >= 0.25f)
    {
      g_foc_current_polarity = (aligned_currents.ia > 0.0f) ? 1 : -1;
    }
    else
    {
      g_foc_current_polarity = FOC_VERIFIED_CURRENT_POLARITY;
      printf("FOC alignment phase-A average only %ld mA; using verified current polarity %s\r\n",
             (long)(aligned_currents.ia * 1000.0f),
             (g_foc_current_polarity > 0) ? "+1" : "-1");
    }
    printf("FOC current polarity=%s; averaged aligned currents A=%ld B_reconstructed=%ld B_sensed=%ld C=%ld mA (%lu samples)\r\n",
           (g_foc_current_polarity > 0) ? "+1" : "-1",
           (long)(aligned_currents.ia * 1000.0f),
           (long)(aligned_currents.ib * 1000.0f),
           (long)(aligned_currents.ib_sensed * 1000.0f),
           (long)(aligned_currents.ic * 1000.0f),
           (unsigned long)aligned_sample_count);
    printf("FOC phase-B PWM bias correction=%ld mA; dynamic two-shunt reconstruction enabled\r\n",
           (long)(g_foc_b_pwm_bias_a * 1000.0f));
    g_foc_b_bias_correction_enabled = 1U;
  }

  g_foc_encoder_zero_count = angle_zero_final;
  g_foc_electrical_angle_rad = 0.0f;
  g_foc_encoder_observed_angle_rad = 0.0f;
  g_foc_electrical_velocity_rad_s = 0.0f;
#if FOC_PREFAULT_CAPTURE_ENABLE
  g_foc_encoder_raw_angle_rad = 0.0f;
  g_foc_encoder_innovation_rad = 0.0f;
  g_foc_encoder_correction_rad = 0.0f;
#endif
  g_foc_encoder_observation_cycles = DWT->CYCCNT;
  g_foc_encoder_observation_sequence = 0U;
  g_foc_id_integrator = 0.0f;
  g_foc_iq_integrator = 0.0f;
  g_foc_id_reference_a = FOC_ID_TARGET_A;
  g_foc_iq_reference_a = 0.0f;
  g_foc_speed_reference_rpm = 0.0f;
  g_foc_mechanical_position_counts = 0;
  g_foc_output_trajectory_position_deg = 0.0f;
  g_foc_debug_mechanical_rpm_window = 0.0f;
  g_foc_debug_speed_error_rpm = 0.0f;
  g_foc_debug_position_error_deg = 0.0f;
  g_foc_debug_encoder_innovation_rad = 0.0f;
  g_foc_debug_encoder_correction_rad = 0.0f;
  g_foc_debug_encoder_window_counts = 0;
  g_foc_debug_iq_saturated = 0U;
  g_foc_debug_speed_integrator_a = 0.0f;
  g_foc_debug_breakaway_current_a = 0.0f;
#if FOC_POSITION_DEMO_ENABLE
  g_foc_output_position_target_deg =
      g_foc_output_position_demo_targets_deg[0];
  foc_iq_limit_a = FOC_POSITION_IQ_LIMIT_A;
  g_foc_active_hard_current_limit_a = FOC_POSITION_HARD_CURRENT_LIMIT_A;
  g_foc_active_dq_fault_limit_a = FOC_POSITION_DQ_FAULT_LIMIT_A;
#elif FOC_CURRENT_STEP_TEST_ENABLE
  g_foc_output_position_target_deg = 0.0f;
  foc_iq_limit_a = FOC_CURRENT_STEP_IQ_LIMIT_A;
  g_foc_active_hard_current_limit_a =
      FOC_CURRENT_STEP_HARD_CURRENT_LIMIT_A;
  g_foc_active_dq_fault_limit_a = FOC_CURRENT_STEP_DQ_FAULT_LIMIT_A;
#else
  g_foc_output_position_target_deg = 0.0f;
#if FOC_LOW_SPEED_VELOCITY_TEST_ENABLE
  foc_iq_limit_a = FOC_LOW_SPEED_IQ_LIMIT_A;
  g_foc_active_hard_current_limit_a = FOC_POSITION_HARD_CURRENT_LIMIT_A;
  g_foc_active_dq_fault_limit_a = FOC_POSITION_DQ_FAULT_LIMIT_A;
#else
  g_foc_active_hard_current_limit_a = FOC_HARD_CURRENT_LIMIT_A;
  g_foc_active_dq_fault_limit_a = FOC_DQ_FAULT_LIMIT_A;
#endif
#endif
  g_foc_telemetry_id_sum = 0.0f;
  g_foc_telemetry_iq_sum = 0.0f;
  g_foc_telemetry_phase_square_sum = 0.0f;
  g_foc_telemetry_phase_peak_a = 0.0f;
  g_foc_telemetry_sample_count = 0U;
  g_foc_current_isr_max_cycles = 0U;
  g_foc_overcurrent_sample_count = 0U;
  g_foc_fault_max_current_a = 0.0f;
  g_foc_fault_ia_a = 0.0f;
  g_foc_fault_ib_a = 0.0f;
  g_foc_fault_ic_a = 0.0f;
  g_foc_fault_id_a = 0.0f;
  g_foc_fault_iq_a = 0.0f;
  g_foc_fault_electrical_angle_rad = 0.0f;
  g_foc_fault_valid_mask = 0U;
#if FOC_PREFAULT_CAPTURE_ENABLE
  g_foc_prefault_write_index = 0U;
  g_foc_prefault_count = 0U;
  g_foc_prefault_sequence = 0U;
#endif
  previous_angle = angle_zero_final;
  g_foc_log_count = 0U;

  /* Finish blocking 9600-baud UART output before enabling current control. */
#if FOC_POSITION_DEMO_ENABLE
  printf("FOC profiled output-position-to-Iq PD demo: %u targets, no integral or breakaway pulse\r\n",
         (unsigned int)position_target_count);
  printf("FOC gearbox=%ld/1000 motor rev/output rev; output 360 deg requires %ld/1000 motor turns\r\n",
         (long)(FOC_MOTOR_TO_OUTPUT_GEAR_RATIO * 1000.0f),
         (long)(FOC_MOTOR_TO_OUTPUT_GEAR_RATIO * 1000.0f));
  printf("FOC output position is inferred from the motor encoder; gearbox backlash is not measured\r\n");
  printf("FOC output targets relative to aligned zero: 0,90,180,270,360,270,180,90,0 deg\r\n");
  printf("FOC output max=%ld x0.001 rpm, accel/decel=%ld/%ld x0.001 rpm/s; motor max=%ld rpm, accel/decel=%ld/%ld rpm/s\r\n",
         (long)(FOC_POSITION_TRAJECTORY_MAX_OUTPUT_RPM * 1000.0f),
         (long)(FOC_POSITION_TRAJECTORY_ACCEL_OUTPUT_RPM_S * 1000.0f),
         (long)(FOC_POSITION_TRAJECTORY_DECEL_OUTPUT_RPM_S * 1000.0f),
         (long)((FOC_POSITION_TRAJECTORY_MAX_OUTPUT_RPM *
                 FOC_MOTOR_TO_OUTPUT_GEAR_RATIO) + 0.5f),
         (long)((FOC_POSITION_TRAJECTORY_ACCEL_OUTPUT_RPM_S *
                 FOC_MOTOR_TO_OUTPUT_GEAR_RATIO) + 0.5f),
         (long)((FOC_POSITION_TRAJECTORY_DECEL_OUTPUT_RPM_S *
                 FOC_MOTOR_TO_OUTPUT_GEAR_RATIO) + 0.5f));
  printf("FOC duration=%lu ms, step timeout=%lu ms, CSV interval=%lu ms\r\n",
         (unsigned long)foc_test_duration_ms,
         (unsigned long)FOC_POSITION_STEP_TIMEOUT_MS,
         (unsigned long)foc_log_interval_ms);
  printf("FOC output-frame PD: P=%ld/1000 A/output-deg, D=%ld/1000 A/output-rpm; output tolerance=%ld mdeg, hold=%lu ms, Iq_limit=%ld mA\r\n",
         (long)(FOC_POSITION_KP_A_PER_OUTPUT_DEG * 1000.0f),
         (long)(FOC_POSITION_KD_A_PER_OUTPUT_RPM * 1000.0f),
         (long)(FOC_OUTPUT_POSITION_TOLERANCE_DEG * 1000.0f),
         (unsigned long)FOC_POSITION_HOLD_MS,
         (long)(foc_iq_limit_a * 1000.0f));
#elif FOC_CURRENT_STEP_TEST_ENABLE
  printf("FOC d-axis current-step diagnostic: duration=%lu ms, CSV interval=%lu ms\r\n",
         (unsigned long)foc_test_duration_ms,
         (unsigned long)foc_log_interval_ms);
  printf("FOC Id sequence: 0 A, +%ld mA at %lu ms, +%ld mA at %lu ms, 0 A at %lu ms; Iq held at 0 mA\r\n",
         (long)(FOC_CURRENT_STEP_1_A * 1000.0f),
         (unsigned long)FOC_CURRENT_STEP_1_START_MS,
         (long)(FOC_CURRENT_STEP_2_A * 1000.0f),
         (unsigned long)FOC_CURRENT_STEP_2_START_MS,
         (unsigned long)FOC_CURRENT_STEP_ZERO_START_MS);
  printf("FOC diagnostic protections: phase=%ld mA, d/q=%ld mA, overspeed=%ld rpm, reserved Iq authority=%ld mA\r\n",
         (long)(g_foc_active_hard_current_limit_a * 1000.0f),
         (long)(g_foc_active_dq_fault_limit_a * 1000.0f),
         (long)foc_overspeed_rpm,
         (long)(foc_iq_limit_a * 1000.0f));
#else
#if FOC_LOW_SPEED_VELOCITY_TEST_ENABLE
  printf("FOC standalone low-speed velocity PI test: target=%ld rpm, duration=%lu ms, CSV interval=%lu ms\r\n",
         (long)foc_speed_target_rpm,
         (unsigned long)foc_test_duration_ms,
         (unsigned long)foc_log_interval_ms);
  printf("FOC velocity PI: Kp=%ld/1000 A/rpm, Ki=%ld/1000 A/(rpm*s), integral_limit=%ld mA, Iq_limit=%ld mA\r\n",
         (long)(foc_speed_kp_a_per_rpm * 1000.0f),
         (long)(foc_speed_ki_a_per_rpm_s * 1000.0f),
         (long)(foc_speed_integral_limit_a * 1000.0f),
         (long)(foc_iq_limit_a * 1000.0f));
#else
  printf("FOC speed-loop trial: target=%ld rpm, ramp=%ld rpm/s, Iq_limit=%ld mA, max modulation=%ld/10000\r\n",
         (long)FOC_SPEED_TARGET_RPM,
         (long)FOC_SPEED_REFERENCE_RAMP_RPM_S,
         (long)(FOC_IQ_TARGET_A * 1000.0f),
         (long)(FOC_MAX_MODULATION * 10000.0f));
#endif
#endif
  printf("FOC encoder timing: %lu Hz reads, %lu us effective sensor age, SPI=%lu Hz\r\n",
         (unsigned long)(1000000U / FOC_ENCODER_UPDATE_US),
         (unsigned long)FOC_ENCODER_SENSOR_DELAY_US,
         (unsigned long)(HAL_RCC_GetPCLK1Freq() / 16U));

  /* HAL_GetTick() has 1 ms resolution. Starting just before its boundary can
   * make the first nominal 1 ms telemetry bucket only a few microseconds
   * long, falsely reporting that the 20 kHz current ISR did not run. Begin
   * immediately after a fresh tick so the first bucket has a full interval. */
#if FOC_CURRENT_STEP_TEST_ENABLE
  {
    uint32_t tick_before_sync = HAL_GetTick();

    while (HAL_GetTick() == tick_before_sync)
    {
      /* PWM/current control is still disabled during this bounded wait. */
    }
  }
#endif
  start_tick = HAL_GetTick();
#if FOC_POSITION_DEMO_ENABLE
  position_step_start_tick = start_tick;
#endif
  last_encoder_cycles = DWT->CYCCNT;
  last_encoder_observation_cycles =
      last_encoder_cycles -
      ((SystemCoreClock / 1000000U) * FOC_ENCODER_SENSOR_DELAY_US);
  speed_window_start_cycles = last_encoder_cycles;
  encoder_update_cycles =
      (SystemCoreClock / 1000000U) * FOC_ENCODER_UPDATE_US;
  speed_window_cycles =
      (SystemCoreClock / 1000000U) * FOC_SPEED_WINDOW_US;
  last_log_tick = start_tick;
  g_foc_seconds_per_core_cycle = 1.0f / (float)SystemCoreClock;
  g_foc_enabled = 1U;

  while ((HAL_GetTick() - start_tick) < foc_test_duration_ms &&
         g_foc_enabled != 0U)
  {
    uint32_t now_tick = HAL_GetTick();
    uint32_t now_cycles = DWT->CYCCNT;

#if FOC_CURRENT_STEP_TEST_ENABLE
    {
      uint32_t current_step_elapsed_ms = now_tick - start_tick;

      if (current_step_elapsed_ms < FOC_CURRENT_STEP_1_START_MS)
      {
        g_foc_id_reference_a = 0.0f;
      }
      else if (current_step_elapsed_ms < FOC_CURRENT_STEP_2_START_MS)
      {
        g_foc_id_reference_a = FOC_CURRENT_STEP_1_A;
      }
      else if (current_step_elapsed_ms < FOC_CURRENT_STEP_ZERO_START_MS)
      {
        g_foc_id_reference_a = FOC_CURRENT_STEP_2_A;
      }
      else
      {
        g_foc_id_reference_a = 0.0f;
      }
      /* This diagnostic deliberately exercises only the flux-producing
       * current axis. Any sustained torque current or shaft acceleration is
       * therefore evidence of an angle, sign, or transform error. */
      g_foc_iq_reference_a = 0.0f;
      g_foc_speed_reference_rpm = 0.0f;
    }
#endif

    if ((now_cycles - last_encoder_cycles) >= encoder_update_cycles)
    {
      uint16_t angle_now;
      uint32_t encoder_service_interval_cycles =
          now_cycles - last_encoder_cycles;
      uint32_t encoder_read_start_cycles = DWT->CYCCNT;

      if (encoder_service_interval_cycles > encoder_max_service_interval_cycles)
      {
        encoder_max_service_interval_cycles = encoder_service_interval_cycles;
      }
      encoder_service_count++;
      last_encoder_cycles = now_cycles;
      if (AS5048A_ReadAngle(&angle_now))
      {
        uint32_t encoder_read_end_cycles = DWT->CYCCNT;
        uint32_t encoder_read_midpoint_cycles =
            encoder_read_start_cycles +
            ((encoder_read_end_cycles - encoder_read_start_cycles) / 2U);
        uint32_t sensor_delay_cycles =
            (SystemCoreClock / 1000000U) * FOC_ENCODER_SENSOR_DELAY_US;
        uint32_t observation_cycles =
            encoder_read_midpoint_cycles - sensor_delay_cycles;
        int32_t encoder_delta = AS5048A_SignedDelta(angle_now, previous_angle);
        encoder_errors = 0U;

        /* At 8 kHz, polling is slower than the AS5048A's minimum internal
         * update rate.  An equal count can therefore be a fresh observation
         * of a stationary/slow rotor and is allowed through the same
         * innovation test.  At speed, a genuinely stale equal count produces
         * a large negative innovation and is rejected below. */
        if (encoder_delta == 0)
        {
          encoder_duplicate_samples++;
        }
        {
          uint32_t elapsed_observation_cycles =
              observation_cycles - last_encoder_observation_cycles;
          float sample_dt_s =
              (float)elapsed_observation_cycles / (float)SystemCoreClock;
          float electrical_angle_observed =
              FOC_ElectricalAngleFromEncoder(angle_now);
          float electrical_angle_predicted = WrapRadians(
              g_foc_encoder_observed_angle_rad +
              (g_foc_electrical_velocity_rad_s * sample_dt_s));
          float encoder_innovation_rad = WrapSignedRadians(
              electrical_angle_observed - electrical_angle_predicted);

          g_foc_debug_encoder_innovation_rad = encoder_innovation_rad;
          g_foc_debug_encoder_correction_rad = 0.0f;

#if FOC_PREFAULT_CAPTURE_ENABLE
          g_foc_encoder_raw_angle_rad = electrical_angle_observed;
          g_foc_encoder_innovation_rad = encoder_innovation_rad;
          g_foc_encoder_correction_rad = 0.0f;
#endif

          /* The AS5048A output is asynchronous to SPI, so its effective age
           * moves by roughly one internal sample period.  Never publish the
           * raw measurement as an abrupt phase step.  Plausible observations
           * make a bounded PLL-like correction to the predicted angle;
           * genuinely implausible observations are discarded. */
          if (AbsFloat(encoder_innovation_rad) >
              FOC_ENCODER_MAX_INNOVATION_RAD)
          {
            float rejected_magnitude = AbsFloat(encoder_innovation_rad);

            encoder_rejected_samples++;
            encoder_consecutive_rejections++;
            if (rejected_magnitude > encoder_max_rejected_innovation_rad)
            {
              encoder_max_rejected_innovation_rad = rejected_magnitude;
            }
          }
          else
          {
            float angle_correction_unclamped =
                FOC_ENCODER_CORRECTION_GAIN * encoder_innovation_rad;
            float angle_correction = ClampFloat(
                angle_correction_unclamped,
                -FOC_ENCODER_MAX_CORRECTION_RAD,
                FOC_ENCODER_MAX_CORRECTION_RAD);
            float electrical_velocity_instant =
                ((float)(g_foc_encoder_direction * encoder_delta *
                         (int32_t)MOTOR_POLE_PAIRS) * TWO_PI_F) /
                (16384.0f * sample_dt_s);
            /* Polling at 8 kHz sees either one or two of the AS5048A's
             * internal updates.  A low-gain linear filter averages that
             * deterministic timing pattern without following each step. */
            float velocity_correction = ClampFloat(
                FOC_ENCODER_VELOCITY_GAIN *
                    (electrical_velocity_instant -
                     g_foc_electrical_velocity_rad_s),
                -FOC_ENCODER_MAX_VELOCITY_STEP_RAD_S,
                FOC_ENCODER_MAX_VELOCITY_STEP_RAD_S);
            float electrical_velocity_filtered =
                g_foc_electrical_velocity_rad_s + velocity_correction;

            if (angle_correction != angle_correction_unclamped)
            {
              encoder_limited_corrections++;
            }
            g_foc_debug_encoder_correction_rad = angle_correction;
#if FOC_PREFAULT_CAPTURE_ENABLE
            g_foc_encoder_correction_rad = angle_correction;
#endif
            g_foc_mechanical_position_counts +=
                g_foc_encoder_direction * encoder_delta;
            previous_angle = angle_now;
            last_encoder_observation_cycles = observation_cycles;
            encoder_consecutive_rejections = 0U;
            speed_window_counts += g_foc_encoder_direction * encoder_delta;
            FOC_PublishEncoderObservation(
                WrapRadians(electrical_angle_predicted + angle_correction),
                electrical_velocity_filtered,
                observation_cycles);

            if ((now_cycles - speed_window_start_cycles) >= speed_window_cycles)
            {
              uint32_t elapsed_speed_cycles =
                  now_cycles - speed_window_start_cycles;

              mechanical_rpm_window =
                  ((float)speed_window_counts * 60.0f *
                   (float)SystemCoreClock) /
                  (16384.0f * (float)elapsed_speed_cycles);
              mechanical_rpm_filtered +=
                  FOC_SPEED_FILTER_ALPHA *
                  (mechanical_rpm_window - mechanical_rpm_filtered);
              g_foc_debug_mechanical_rpm_window = mechanical_rpm_window;
              g_foc_debug_encoder_window_counts = speed_window_counts;

#if FOC_POSITION_DEMO_ENABLE
              {
                float motor_position_deg =
                    ((float)g_foc_mechanical_position_counts * 360.0f) /
                    16384.0f;
                float output_position_deg =
                    (FOC_OUTPUT_DIRECTION_SIGN * motor_position_deg) /
                    FOC_MOTOR_TO_OUTPUT_GEAR_RATIO;
                float output_rpm_filtered =
                    (FOC_OUTPUT_DIRECTION_SIGN * mechanical_rpm_filtered) /
                    FOC_MOTOR_TO_OUTPUT_GEAR_RATIO;
                float position_dt_s =
                    (float)elapsed_speed_cycles / (float)SystemCoreClock;
                float output_trajectory_remaining_deg =
                    g_foc_output_position_target_deg -
                    g_foc_output_trajectory_position_deg;
                float output_trajectory_stop_rpm = sqrtf(
                    FOC_POSITION_TRAJECTORY_DECEL_OUTPUT_RPM_S *
                    AbsFloat(output_trajectory_remaining_deg) / 3.0f);
                float output_trajectory_desired_rpm =
                    (output_trajectory_remaining_deg > 0.0f) ?
                        ClampFloat(output_trajectory_stop_rpm, 0.0f,
                                   FOC_POSITION_TRAJECTORY_MAX_OUTPUT_RPM) :
                    (output_trajectory_remaining_deg < 0.0f) ?
                        -ClampFloat(output_trajectory_stop_rpm, 0.0f,
                                    FOC_POSITION_TRAJECTORY_MAX_OUTPUT_RPM) :
                        0.0f;
                float output_trajectory_old_rpm =
                    output_trajectory_speed_rpm;
                float output_trajectory_rate_rpm_s =
                    ((output_trajectory_old_rpm *
                      output_trajectory_desired_rpm) < 0.0f ||
                     AbsFloat(output_trajectory_desired_rpm) <
                         AbsFloat(output_trajectory_old_rpm)) ?
                        FOC_POSITION_TRAJECTORY_DECEL_OUTPUT_RPM_S :
                        FOC_POSITION_TRAJECTORY_ACCEL_OUTPUT_RPM_S;
                float output_trajectory_next_deg;
                float output_final_position_error_deg;
                float output_tracking_position_error_deg;
                float output_speed_error_rpm;
                float motor_trajectory_speed_rpm;
                float motor_speed_error_rpm;

                output_trajectory_speed_rpm = RampToward(
                    output_trajectory_old_rpm,
                    output_trajectory_desired_rpm,
                    output_trajectory_rate_rpm_s,
                    position_dt_s);
                output_trajectory_next_deg =
                    g_foc_output_trajectory_position_deg +
                    (0.5f * (output_trajectory_old_rpm +
                             output_trajectory_speed_rpm) *
                     6.0f * position_dt_s);

                if ((output_trajectory_remaining_deg > 0.0f &&
                     output_trajectory_next_deg >=
                         g_foc_output_position_target_deg) ||
                    (output_trajectory_remaining_deg < 0.0f &&
                     output_trajectory_next_deg <=
                         g_foc_output_position_target_deg) ||
                    (AbsFloat(output_trajectory_remaining_deg) < 0.001f &&
                     AbsFloat(output_trajectory_speed_rpm) < 0.01f))
                {
                  g_foc_output_trajectory_position_deg =
                      g_foc_output_position_target_deg;
                  output_trajectory_speed_rpm = 0.0f;
                }
                else
                {
                  g_foc_output_trajectory_position_deg =
                      output_trajectory_next_deg;
                }

                output_final_position_error_deg =
                    g_foc_output_position_target_deg - output_position_deg;
                output_tracking_position_error_deg =
                    g_foc_output_trajectory_position_deg -
                    output_position_deg;
                output_speed_error_rpm =
                    output_trajectory_speed_rpm - output_rpm_filtered;
                motor_trajectory_speed_rpm =
                    FOC_OUTPUT_DIRECTION_SIGN *
                    FOC_MOTOR_TO_OUTPUT_GEAR_RATIO *
                    output_trajectory_speed_rpm;
                motor_speed_error_rpm =
                    motor_trajectory_speed_rpm - mechanical_rpm_filtered;

                g_foc_debug_position_error_deg =
                    output_tracking_position_error_deg;
                g_foc_speed_reference_rpm = motor_trajectory_speed_rpm;
                g_foc_debug_speed_error_rpm = motor_speed_error_rpm;

                if (AbsFloat(g_foc_output_position_target_deg -
                             g_foc_output_trajectory_position_deg) < 0.01f &&
                    AbsFloat(output_trajectory_speed_rpm) < 0.01f &&
                    AbsFloat(output_final_position_error_deg) <=
                        FOC_OUTPUT_POSITION_TOLERANCE_DEG &&
                    AbsFloat(output_rpm_filtered) <=
                        FOC_OUTPUT_SPEED_TOLERANCE_RPM)
                {
                  if (position_hold_start_tick == 0U)
                  {
                    position_hold_start_tick = now_tick;
                  }
                  else if ((now_tick - position_hold_start_tick) >=
                           FOC_POSITION_HOLD_MS)
                  {
                    if ((uint8_t)(position_target_index + 1U) <
                        position_target_count)
                    {
                      position_target_index++;
                      g_foc_output_position_target_deg =
                          g_foc_output_position_demo_targets_deg[
                              position_target_index];
                      g_foc_debug_speed_integrator_a = 0.0f;
                      position_hold_start_tick = 0U;
                      position_step_start_tick = now_tick;
                    }
                    else
                    {
                      position_demo_completed = 1U;
                      g_foc_speed_reference_rpm = 0.0f;
                      g_foc_iq_reference_a = 0.0f;
                      g_foc_enabled = 0U;
                    }
                  }
                }
                else if ((now_tick - position_step_start_tick) >=
                         FOC_POSITION_STEP_TIMEOUT_MS)
                {
                  g_foc_fault = 10U;
                  g_foc_speed_reference_rpm = 0.0f;
                  g_foc_iq_reference_a = 0.0f;
                  g_foc_enabled = 0U;
                  Motor_PWM_Off();
                }
                else
                {
                  position_hold_start_tick = 0U;
                }

                if (position_demo_completed == 0U &&
                    g_foc_enabled != 0U)
                {
                  float iq_unclamped =
                      FOC_OUTPUT_DIRECTION_SIGN *
                      ((FOC_POSITION_KP_A_PER_OUTPUT_DEG *
                        output_tracking_position_error_deg) +
                       (FOC_POSITION_KD_A_PER_OUTPUT_RPM *
                        output_speed_error_rpm));

                  /* The outer PD is output-referenced. This prevents the gear
                   * conversion from multiplying its effective stiffness and
                   * damping by the ratio; current/electrical FOC stays motor-side. */
                  g_foc_debug_speed_integrator_a = 0.0f;
                  g_foc_debug_breakaway_current_a = 0.0f;
                  g_foc_debug_iq_saturated =
                      (AbsFloat(iq_unclamped) >= foc_iq_limit_a) ? 1U : 0U;
                  g_foc_iq_reference_a = ClampFloat(
                      iq_unclamped,
                      -foc_iq_limit_a,
                      foc_iq_limit_a);
                }
              }
#elif FOC_CURRENT_STEP_TEST_ENABLE
              g_foc_debug_speed_error_rpm = -mechanical_rpm_filtered;
              g_foc_debug_speed_integrator_a = 0.0f;
              g_foc_debug_breakaway_current_a = 0.0f;
              g_foc_debug_iq_saturated = 0U;
#else
#if FOC_LOW_SPEED_VELOCITY_TEST_ENABLE
              g_foc_speed_reference_rpm = foc_speed_target_rpm;
#else
              g_foc_speed_reference_rpm = ClampFloat(
                  FOC_SPEED_REFERENCE_RAMP_RPM_S *
                      ((float)(now_tick - start_tick) / 1000.0f),
                  0.0f, foc_speed_target_rpm);
#endif
              if (g_foc_enabled != 0U)
              {
                float speed_error_rpm =
                    g_foc_speed_reference_rpm - mechanical_rpm_filtered;
                float speed_dt_s =
                    (float)elapsed_speed_cycles / (float)SystemCoreClock;
                float integrator_candidate = ClampFloat(
                    speed_integrator_a +
                        (foc_speed_ki_a_per_rpm_s * speed_error_rpm * speed_dt_s),
                    -foc_speed_integral_limit_a,
                    foc_speed_integral_limit_a);
                float iq_unclamped =
                    (foc_speed_kp_a_per_rpm * speed_error_rpm) +
                    integrator_candidate;
                float iq_command = ClampFloat(iq_unclamped,
                                              -foc_iq_limit_a,
                                              foc_iq_limit_a);

                /* Conditional integration prevents windup at the active
                 * velocity- or position-mode torque-current limit. */
                if (iq_command == iq_unclamped ||
                    (iq_command >= foc_iq_limit_a && speed_error_rpm < 0.0f) ||
                    (iq_command <= -foc_iq_limit_a && speed_error_rpm > 0.0f))
                {
                  speed_integrator_a = integrator_candidate;
                }
                g_foc_debug_speed_error_rpm = speed_error_rpm;
                g_foc_debug_speed_integrator_a = speed_integrator_a;
                g_foc_debug_breakaway_current_a = 0.0f;
                g_foc_debug_iq_saturated =
                    (AbsFloat(iq_unclamped) >= foc_iq_limit_a) ? 1U : 0U;
                g_foc_iq_reference_a = ClampFloat(
                    (foc_speed_kp_a_per_rpm * speed_error_rpm) +
                        speed_integrator_a,
                    -foc_iq_limit_a, foc_iq_limit_a);
              }
#endif
              speed_window_counts = 0;
              speed_window_start_cycles = now_cycles;
            }
          }
        }

        if (encoder_consecutive_rejections >=
            FOC_ENCODER_MAX_CONSECUTIVE_REJECTIONS)
        {
          g_foc_fault = 7U;
          g_foc_enabled = 0U;
          Motor_PWM_Off();
        }
      }
      else if (++encoder_errors >= FOC_ENCODER_MAX_CONSECUTIVE_ERRORS)
      {
        g_foc_fault = 3U;
        g_foc_enabled = 0U;
        Motor_PWM_Off();
        printf("FOC stopped: AS5048A angle read failed %u consecutive times\r\n",
               encoder_errors);
      }

      /* A zero limit disables the experimental position-mode overspeed
       * check. The velocity demo still supplies its 5200 RPM limit here. */
      if (foc_overspeed_rpm > 0.0f &&
          AbsFloat(mechanical_rpm_window) > foc_overspeed_rpm)
      {
        mechanical_rpm_filtered = mechanical_rpm_window;
        g_foc_fault = 4U;
        g_foc_enabled = 0U;
        Motor_PWM_Off();
        printf("FOC stopped: speed %ld rpm exceeds %ld rpm\r\n",
               (long)mechanical_rpm_window,
               (long)foc_overspeed_rpm);
      }
    }

    if (HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin) == GPIO_PIN_RESET)
    {
      g_foc_fault = 5U;
      g_foc_enabled = 0U;
      Motor_Fault_Shutdown();
      printf("FOC stopped: DRV8353S nFAULT asserted\r\n");
    }

    if ((now_tick - last_log_tick) >= foc_log_interval_ms)
    {
      last_log_tick = now_tick;
      FOC_LogSampleCapture(now_tick - start_tick, mechanical_rpm_filtered);
      if (g_foc_log_count != 0U &&
#if FOC_CURRENT_STEP_TEST_ENABLE
          /* The first bucket is retained for visibility but is not a valid
           * ISR-health decision if timer startup consumed part of it. */
          (now_tick - start_tick) > FOC_CURRENT_STEP_LOG_INTERVAL_MS &&
#endif
          g_foc_log[g_foc_log_count - 1U].current_samples <
              foc_min_current_samples_per_log)
      {
        uint32_t completed_samples =
            g_foc_log[g_foc_log_count - 1U].current_samples;

        g_foc_fault = 6U;
        g_foc_enabled = 0U;
        Motor_PWM_Off();
        printf("FOC stopped: only %lu current-loop samples; max ISR=%lu cycles (%lu us), budget=%lu cycles\r\n",
               (unsigned long)completed_samples,
               (unsigned long)g_foc_log[g_foc_log_count - 1U].current_isr_max_cycles,
               (unsigned long)(((uint64_t)g_foc_log[g_foc_log_count - 1U].current_isr_max_cycles *
                                1000000ULL) / (uint64_t)SystemCoreClock),
               (unsigned long)(SystemCoreClock / FOC_CURRENT_LOOP_HZ));
      }
    }
  }

#if FOC_POSITION_DEMO_ENABLE
  if (g_foc_fault == 0U && position_demo_completed == 0U)
  {
    g_foc_fault = 10U;
    g_foc_speed_reference_rpm = 0.0f;
    g_foc_iq_reference_a = 0.0f;
    g_foc_enabled = 0U;
    Motor_PWM_Off();
  }
#endif

  {
    uint32_t final_elapsed_ms = HAL_GetTick() - start_tick;

    if (g_foc_log_count == 0U ||
        g_foc_log[g_foc_log_count - 1U].time_ms != (uint16_t)final_elapsed_ms)
    {
      FOC_LogSampleCapture(final_elapsed_ms, mechanical_rpm_filtered);
    }
  }
  g_foc_iq_reference_a = 0.0f;
  g_foc_enabled = 0U;
  /* Encoder observations are serviced by the foreground loop above. Once
   * that loop exits, do not leave PWM active while the rotor keeps moving
   * and the predicted electrical angle becomes stale. Coast immediately;
   * writing neutral compare values is safe after MOE is clear. */
  Motor_PWM_Off();
  Set_DQ_SVPWM(g_foc_electrical_angle_rad, 0.0f, 0.0f);
  printf("FOC encoder duplicate samples suppressed=%lu\r\n",
         (unsigned long)encoder_duplicate_samples);
  printf("FOC encoder innovation rejections=%lu, max=%ld mrad\r\n",
         (unsigned long)encoder_rejected_samples,
         (long)(encoder_max_rejected_innovation_rad * 1000.0f));
  printf("FOC encoder bounded corrections=%lu, maximum phase step=%ld mrad\r\n",
         (unsigned long)encoder_limited_corrections,
         (long)(FOC_ENCODER_MAX_CORRECTION_RAD * 1000.0f));
  {
    uint32_t elapsed_ms = HAL_GetTick() - start_tick;
    uint32_t actual_encoder_hz =
        (elapsed_ms != 0U) ?
        (uint32_t)(((uint64_t)encoder_service_count * 1000ULL) / elapsed_ms) : 0U;
    uint32_t max_encoder_gap_us =
        (uint32_t)(((uint64_t)encoder_max_service_interval_cycles *
                    1000000ULL) / (uint64_t)SystemCoreClock);

    printf("FOC encoder actual service=%lu Hz, maximum service gap=%lu us\r\n",
           (unsigned long)actual_encoder_hz,
           (unsigned long)max_encoder_gap_us);
  }
  if (g_foc_fault == 7U)
  {
    printf("FOC stopped: %u consecutive encoder observations failed plausibility checks\r\n",
           FOC_ENCODER_MAX_CONSECUTIVE_REJECTIONS);
  }
  else if (g_foc_fault == 8U)
  {
    printf("FOC stopped: accepted encoder angle became older than %lu us\r\n",
           (unsigned long)FOC_ENCODER_PREDICTION_MAX_US);
  }
#if FOC_POSITION_DEMO_ENABLE
  printf("FOC output position result: completed=%u, step=%u/%u, target=%ld mdeg, output=%ld mdeg, motor=%ld mdeg\r\n",
         (unsigned int)position_demo_completed,
         (unsigned int)(position_target_index + 1U),
         (unsigned int)position_target_count,
         (long)(g_foc_output_position_target_deg * 1000.0f),
         (long)((((float)g_foc_mechanical_position_counts * 360000.0f) /
                 16384.0f) * FOC_OUTPUT_DIRECTION_SIGN /
                FOC_MOTOR_TO_OUTPUT_GEAR_RATIO),
         (long)(((int64_t)g_foc_mechanical_position_counts * 360000LL) /
                16384LL));
  if (g_foc_fault == 10U)
  {
    printf("FOC stopped: position sequence did not settle before the step or overall timeout\r\n");
  }
#endif
  printf("FOC trial finished with fault=%u\r\n", g_foc_fault);
  if (g_foc_fault == 2U)
  {
    printf("FOC overcurrent snapshot: max=%ld mA A=%ld B=%ld C=%ld Id_prev=%ld Iq_prev=%ld angle=%ld mrad valid_mask=0x%X confirm=%u samples\r\n",
           (long)(g_foc_fault_max_current_a * 1000.0f),
           (long)(g_foc_fault_ia_a * 1000.0f),
           (long)(g_foc_fault_ib_a * 1000.0f),
           (long)(g_foc_fault_ic_a * 1000.0f),
           (long)(g_foc_fault_id_a * 1000.0f),
           (long)(g_foc_fault_iq_a * 1000.0f),
           (long)(g_foc_fault_electrical_angle_rad * 1000.0f),
           g_foc_fault_valid_mask,
           FOC_OVERCURRENT_CONFIRM_SAMPLES);
  }
  else if (g_foc_fault == 9U)
  {
    printf("FOC abnormal d/q-current snapshot: limit=%ld mA A=%ld B=%ld C=%ld Id=%ld Iq=%ld angle=%ld mrad valid_mask=0x%X\r\n",
           (long)(g_foc_active_dq_fault_limit_a * 1000.0f),
           (long)(g_foc_fault_ia_a * 1000.0f),
           (long)(g_foc_fault_ib_a * 1000.0f),
           (long)(g_foc_fault_ic_a * 1000.0f),
           (long)(g_foc_fault_id_a * 1000.0f),
           (long)(g_foc_fault_iq_a * 1000.0f),
           (long)(g_foc_fault_electrical_angle_rad * 1000.0f),
           g_foc_fault_valid_mask);
  }
#if FOC_PREFAULT_CAPTURE_ENABLE
  if (g_foc_fault == 0U || g_foc_fault == 2U ||
      g_foc_fault == 7U || g_foc_fault == 8U ||
      g_foc_fault == 9U)
  {
    FOC_PreFaultDump();
  }
#endif
  FOC_LogDump();

foc_stop:
  g_foc_enabled = 0U;
  g_foc_id_reference_a = 0.0f;
  g_foc_iq_reference_a = 0.0f;
  g_foc_speed_reference_rpm = 0.0f;
  Set_DQ_SVPWM(g_foc_electrical_angle_rad, 0.0f, 0.0f);
  Motor_PWM_Off();
}

static void CommutateStep(uint8_t step, uint16_t duty)
{
  switch (step % 6U)
  {
    case 0:
      Commutate(1, 2, 3, duty);
      break;
    case 1:
      Commutate(1, 3, 2, duty);
      break;
    case 2:
      Commutate(2, 3, 1, duty);
      break;
    case 3:
      Commutate(2, 1, 3, duty);
      break;
    case 4:
      Commutate(3, 1, 2, duty);
      break;
    default:
      Commutate(3, 2, 1, duty);
      break;
  }
}

static uint8_t CommutationStep_GetLowPhase(uint8_t step)
{
  switch (step % 6U)
  {
    case 0: return 2U;
    case 1: return 3U;
    case 2: return 3U;
    case 3: return 1U;
    case 4: return 1U;
    default: return 2U;
  }
}

void Print_Frequency_Info(void)
{
  printf("SYSCLK: %lu Hz\r\n", HAL_RCC_GetSysClockFreq());
  printf("HCLK:   %lu Hz\r\n", HAL_RCC_GetHCLKFreq());
  printf("PCLK1:  %lu Hz\r\n", HAL_RCC_GetPCLK1Freq());
  printf("PCLK2:  %lu Hz\r\n", HAL_RCC_GetPCLK2Freq());
  printf("TIM1:   %lu Hz\r\n", Get_TIM1_ClockHz());
  printf("PWM before selected test: %lu Hz (timer is switched by the test)\r\n",
         Get_PWM_FrequencyHz());
}

void BLDC_SixStep(void)
{
  uint8_t step = 0;
  float electrical_hz = MOTOR_START_ELECTRICAL_HZ;
  uint32_t step_delay_us = ElectricalHz_ToStepDelayUs(electrical_hz);

  TIM1_Set_EdgeAligned_For_OpenLoopPWM();

  while (1)
  {
    g_actual_electrical_hz = electrical_hz;
    CommutateStep(step, g_pwm_duty_ticks);

    if ((step & 31U) == 0U)
    {
      uint8_t low_phase = CommutationStep_GetLowPhase(step);
      int32_t avg_ma = CurrentSense_GetLowPhaseAverageMilliAmps(low_phase);
      printf("6-step commutation low-phase %u average current = %ld mA\r\n",
             low_phase, (long)avg_ma);
    }

    step++;
    delay_us(step_delay_us);
  }
}

void BLDC_SixStep_Read(void)
{
  uint16_t duty = 7;
  uint16_t phase_delay = 10;
  float offset_A = 1.66f;
  float offset_B = 1.66f;
  float offset_C = 1.66f;

  TIM1_Set_EdgeAligned_For_OpenLoopPWM();

  while (1)
  {
    Commutate(1, 2, 3, duty);
    HAL_Delay(phase_delay);
    printf("IB: %.2f A\r\n", adc_to_current(read_adc_channel(ADC_CHANNEL_7), offset_B));

    Commutate(1, 3, 2, duty);
    HAL_Delay(phase_delay);
    printf("IC: %.2f A\r\n", adc_to_current(read_adc_channel(ADC_CHANNEL_8), offset_C));

    Commutate(2, 3, 1, duty);
    HAL_Delay(phase_delay);
    printf("IC: %.2f A\r\n", adc_to_current(read_adc_channel(ADC_CHANNEL_8), offset_C));

    Commutate(2, 1, 3, duty);
    HAL_Delay(phase_delay);
    printf("IA: %.2f A\r\n", adc_to_current(read_adc_channel(ADC_CHANNEL_6), offset_A));

    Commutate(3, 1, 2, duty);
    HAL_Delay(phase_delay);
    printf("IA: %.2f A\r\n", adc_to_current(read_adc_channel(ADC_CHANNEL_6), offset_A));

    Commutate(3, 2, 1, duty);
    HAL_Delay(phase_delay);
    printf("IB: %.2f A\r\n", adc_to_current(read_adc_channel(ADC_CHANNEL_7), offset_B));
  }
}

void BLDC_SixStep_Read_FILTERED(void)
{
  uint8_t step = 0;
  uint32_t last_tick = HAL_GetTick();
  uint32_t start_tick = last_tick;
  uint32_t last_print_tick = last_tick;
  float offset_A = 1.66f;
  float offset_B = 1.66f;
  float offset_C = 1.66f;
  uint32_t duty_ticks = __HAL_TIM_GET_AUTORELOAD(&htim1)*DESIRED_MOTOR_DUTY;

  TIM1_Set_EdgeAligned_For_OpenLoopPWM();
  Set_PWM_DutyTicks(duty_ticks);
  g_target_electrical_hz = MOTOR_START_ELECTRICAL_HZ;
  g_actual_electrical_hz = MOTOR_START_ELECTRICAL_HZ;

  while (1)
  {
    uint32_t now_tick = HAL_GetTick();
    uint32_t elapsed_ms = now_tick - start_tick;
    float dt_sec = (now_tick - last_tick) / 1000.0f;

    last_tick = now_tick;

    if (elapsed_ms < 3000U)
    {
      g_target_electrical_hz = MOTOR_START_ELECTRICAL_HZ;
    }
    else if (elapsed_ms < 12000U)
    {
      g_target_electrical_hz = MOTOR_HIGH_ELECTRICAL_HZ;
    }
    else
    {
      g_target_electrical_hz = MOTOR_LOW_ELECTRICAL_HZ;
    }

    g_actual_electrical_hz = RampToward(g_actual_electrical_hz,
                                        g_target_electrical_hz,
                                        MOTOR_RAMP_EHZ_PER_SEC,
                                        dt_sec);

    CommutateStep(step++, g_pwm_duty_ticks);

    if ((now_tick - last_print_tick) >= MOTOR_DEBUG_PRINT_MS)
    {
      last_print_tick = now_tick;

      printf("target: %.2f eHz, actual: %.2f eHz, step: %.2f Hz, rpm: %.1f, pwm: %lu Hz, duty: %u/%lu\r\n",
             g_target_electrical_hz,
             g_actual_electrical_hz,
             Get_CommutationStepFrequencyHz(),
             Get_MechanicalRPM(),
             Get_PWM_FrequencyHz(),
             g_pwm_duty_ticks,
             __HAL_TIM_GET_AUTORELOAD(&htim1));

      printf("IA: %.2f A, IB: %.2f A, IC: %.2f A\r\n",
             read_filtered_current(ADC_CHANNEL_6, offset_A),
             read_filtered_current(ADC_CHANNEL_7, offset_B),
             read_filtered_current(ADC_CHANNEL_8, offset_C));
    }

    delay_us(ElectricalHz_ToStepDelayUs(g_actual_electrical_hz));
  }
}

void BLDC_SixStep_RampLoop(void)
{
  uint8_t step = 0;
  uint8_t ramping_up = 1;
  uint32_t last_tick = HAL_GetTick();

  uint32_t duty_ticks = __HAL_TIM_GET_AUTORELOAD(&htim1)*DESIRED_MOTOR_DUTY;

  TIM1_Set_EdgeAligned_For_OpenLoopPWM();
  Set_PWM_DutyTicks(duty_ticks);
  g_target_electrical_hz = MOTOR_HIGH_ELECTRICAL_HZ;
  g_actual_electrical_hz = MOTOR_LOW_ELECTRICAL_HZ;

  while (1)
  {
    uint32_t now_tick = HAL_GetTick();
    float dt_sec = (now_tick - last_tick) / 1000.0f;

    last_tick = now_tick;

    if (ramping_up)
    {
      g_target_electrical_hz = MOTOR_HIGH_ELECTRICAL_HZ;
      if (g_actual_electrical_hz >= MOTOR_HIGH_ELECTRICAL_HZ)
      {
        ramping_up = 0;
      }
    }
    else
    {
      g_target_electrical_hz = MOTOR_LOW_ELECTRICAL_HZ;
      if (g_actual_electrical_hz <= MOTOR_LOW_ELECTRICAL_HZ)
      {
        ramping_up = 1;
      }
    }

    g_actual_electrical_hz = RampToward(g_actual_electrical_hz,
                                        g_target_electrical_hz,
                                        MOTOR_RAMP_EHZ_PER_SEC,
                                        dt_sec);

    CommutateStep(step++, g_pwm_duty_ticks);
    delay_us(ElectricalHz_ToStepDelayUs(g_actual_electrical_hz));
  }
}

void BLDC_OpenLoop_SVPWM_RampLoop(uint8_t deadtime_ticks, bool dump_log)
{
  float electrical_angle_rad = 0.0f;
  float electrical_hz = MOTOR_START_ELECTRICAL_HZ;
  uint32_t last_update_cycles = DWT->CYCCNT;
  uint32_t last_tick = HAL_GetTick();
  uint32_t start_tick = last_tick;
  uint32_t last_fault_poll_tick = last_tick;
  uint32_t last_current_poll_tick = last_tick;
  uint32_t last_current_led_tick = last_tick;
  const float update_period_sec = (float)MOTOR_SVPWM_UPDATE_US / 1000000.0f;
  const uint32_t update_period_cycles = (SystemCoreClock / 1000000U) * MOTOR_SVPWM_UPDATE_US;

  g_motor_deadtime_ticks = deadtime_ticks;
#if UART_OUTPUT_ENABLE
  printf("Starting SVPWM test with dead-time ticks=%u\r\n", deadtime_ticks);
#endif
  g_target_electrical_hz = MOTOR_HIGH_ELECTRICAL_HZ;
  g_actual_electrical_hz = electrical_hz;
  g_open_loop_modulation = MOTOR_OPEN_LOOP_MODULATION_START;
  g_svpwm_log_count = 0U;
  g_current_sample_sync_timeouts = 0U;
  g_svpwm_log_start_tick = start_tick + SVPWM_CAPTURE_START_MS;
  g_svpwm_log_last_tick = g_svpwm_log_start_tick - SVPWM_LOG_INTERVAL_MS;
  TIM1_Set_CenterAligned_For_SVPWM();
  Set_OpenLoop_SVPWM(electrical_angle_rad, 0.0f);

  /* Establish a known rotor position before asking it to follow a rotating field. */
#if UART_OUTPUT_ENABLE
  printf("Rotor alignment: angle_mrad=%ld, modulation_x10000=%u, hold=%u ms\r\n",
         (long)(MOTOR_ALIGNMENT_ANGLE_RAD * 1000.0f),
         (unsigned int)(MOTOR_OPEN_LOOP_MODULATION_START * 10000.0f),
         MOTOR_ALIGNMENT_HOLD_MS);
#endif
  electrical_angle_rad = MOTOR_ALIGNMENT_ANGLE_RAD;
  g_actual_electrical_hz = 0.0f;
  Set_OpenLoop_SVPWM(electrical_angle_rad, MOTOR_OPEN_LOOP_MODULATION_START);
  {
    uint32_t alignment_start_tick = HAL_GetTick();
    while ((HAL_GetTick() - alignment_start_tick) < MOTOR_ALIGNMENT_HOLD_MS)
    {
      if (Motor_Check_CurrentLimit() != 0U)
      {
        Set_OpenLoop_SVPWM(electrical_angle_rad, 0.0f);
        Motor_PWM_Off();
        printf("SVPWM stopped by phase-current limit during rotor alignment\r\n");
        return;
      }
      HAL_Delay(1U);
    }
  }

  /* Start the timed run only after alignment; keep the aligned electrical angle. */
  electrical_hz = MOTOR_START_ELECTRICAL_HZ;
  g_actual_electrical_hz = electrical_hz;
  g_target_electrical_hz = MOTOR_HIGH_ELECTRICAL_HZ;
  start_tick = HAL_GetTick();
  last_tick = start_tick;
  last_current_poll_tick = start_tick;
  last_fault_poll_tick = start_tick;
  last_update_cycles = DWT->CYCCNT;
  g_svpwm_log_start_tick = start_tick + SVPWM_CAPTURE_START_MS;
  g_svpwm_log_last_tick = g_svpwm_log_start_tick - SVPWM_LOG_INTERVAL_MS;

  while (1)
  {
    uint32_t now_cycles = DWT->CYCCNT;

    if ((now_cycles - last_update_cycles) >= update_period_cycles)
    {
      uint32_t now_tick = HAL_GetTick();
      float ramp_dt_sec = (now_tick - last_tick) / 1000.0f;
      float modulation_index;

      last_update_cycles += update_period_cycles;
      last_tick = now_tick;

#if !MOTOR_SVPWM_CONTINUOUS_RUN
      if ((now_tick - start_tick) >= MOTOR_SVPWM_RUN_TIME_MS)
      {
        break;
      }
#endif

      g_target_electrical_hz = MOTOR_HIGH_ELECTRICAL_HZ;

      electrical_hz = RampToward(electrical_hz,
                                 g_target_electrical_hz,
                                 MOTOR_RAMP_EHZ_PER_SEC,
                                 ramp_dt_sec);

      g_actual_electrical_hz = electrical_hz;
      electrical_angle_rad = WrapRadians(electrical_angle_rad +
                                          (TWO_PI_F * electrical_hz * update_period_sec));
      modulation_index = OpenLoop_ModulationForElapsedMs(now_tick - start_tick);
      g_open_loop_modulation = modulation_index;

#if SVPWM_CURRENT_LOG_ENABLE
      if (now_tick >= g_svpwm_log_start_tick)
      {
        SVPWM_Log(now_tick, electrical_angle_rad);
      }
#endif

      if ((now_tick - last_current_poll_tick) >= 1U)
      {
        last_current_poll_tick = now_tick;
        if (Motor_Check_CurrentLimit() != 0U)
        {
          Set_OpenLoop_SVPWM(electrical_angle_rad, 0.0f);
          Motor_PWM_Off();
          printf("SVPWM stopped by phase-current limit\r\n");
          SVPWM_Log_Dump();
          return;
        }
      }

#if CURRENT_LED_DIAGNOSTIC_ENABLE
      if ((now_tick - last_current_led_tick) >= CURRENT_LED_UPDATE_MS)
      {
        last_current_led_tick = now_tick;
        Debug_CurrentLEDs_Update(Read_AverageAbs_Phase_Current_A());
      }
#endif

      Set_OpenLoop_SVPWM(electrical_angle_rad, modulation_index);

      if (g_svpwm_log_count >= SVPWM_LOG_CAPACITY)
      {
        break;
      }

#if DRV_FAULT_POLL_ENABLE
      if ((now_tick - last_fault_poll_tick) >= 10U)
      {
        last_fault_poll_tick = now_tick;
        if (DRV8353_ReadFaults() != 0U)
        {
          Motor_Fault_Shutdown();
          printf("SVPWM stopped by DRV8353S fault: status1=0x%03X, status2=0x%03X\r\n",
                 g_drv_fault_status1, g_drv_vgs_status2);
          SVPWM_Log_Dump();
          return;
        }
      }
#else
      (void)last_fault_poll_tick;
#endif

    }
  }

  Set_OpenLoop_SVPWM(electrical_angle_rad, 0.0f);
  Motor_PWM_Off();
  printf("SVPWM waveform capture complete: %u samples at %u ms intervals\r\n",
         g_svpwm_log_count, SVPWM_LOG_INTERVAL_MS);
  if (dump_log)
  {
    SVPWM_Log_Dump();
  }
  else
  {
    printf("SVPWM current-log dump disabled for this bring-up run\r\n");
  }
}

float get_offset_voltage(uint32_t channel)
{
  float sum = 0.0f;

  for (int i = 0; i < 50; i++)
  {
    uint16_t adc = read_adc_channel(channel);
    sum += (adc / 4095.0f) * ADC_VREF;
  }

  return sum / 50.0f;
}

void return_cal_offsets(void)
{
  uint16_t reg_val = DRV8353_ReadSPI(0x06);

  reg_val |= (1U << 4);
  reg_val |= (1U << 3);
  reg_val |= (1U << 2);
  DRV8353_WriteSPI(0x06, reg_val);

  HAL_Delay(1);

  uint16_t readback = DRV8353_ReadSPI(0x06);

  printf("CSA REG: ");
  for (int i = 10; i >= 0; i--)
  {
    printf("%d", (readback >> i) & 0x01U);
  }
  printf("\r\n");

  printf("Offsets:\r\n");
  printf("A: %.4f V\r\n", get_offset_voltage(ADC_CHANNEL_6));
  printf("B: %.4f V\r\n", get_offset_voltage(ADC_CHANNEL_7));
  printf("C: %.4f V\r\n", get_offset_voltage(ADC_CHANNEL_8));

  reg_val &= ~(1U << 4);
  reg_val &= ~(1U << 3);
  reg_val &= ~(1U << 2);
  DRV8353_WriteSPI(0x06, reg_val);
}

void CAN_Start(void)
{
  FDCAN_FilterTypeDef filter = {0};

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0x000;
  filter.FilterID2 = 0x000;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_FILTER_REMOTE,
                                   FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

uint8_t CAN_Tx(uint32_t id, uint8_t *data, uint8_t len)
{
  FDCAN_TxHeaderTypeDef txHeader = {0};

  txHeader.Identifier = id;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = fdcan_dlc_from_len(len);
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data) != HAL_OK)
  {
    return 0;
  }

  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
  {
  }

  return 1;
}

uint8_t CAN_Rx(uint32_t *id, uint8_t *data, uint8_t *len)
{
  FDCAN_RxHeaderTypeDef rxHeader = {0};

  if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0U)
  {
    return 0;
  }

  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, data) != HAL_OK)
  {
    return 0;
  }

  *id = rxHeader.Identifier;
  *len = len_from_fdcan_dlc(rxHeader.DataLength);

  return 1;
}

void CAN_Test_Send(void)
{
  uint8_t data[8] = {6, 9, 6, 7, 1, 2, 3, 4};

  CAN_Tx(0x123, data, 8);
}

void uart1_tx(uint8_t data)
{
  while (!(USART1->ISR & USART_ISR_TXE_TXFNF))
  {
  }

  USART1->TDR = data;

  while (!(USART1->ISR & USART_ISR_TC))
  {
  }
}

uint8_t uart1_rx(void)
{
  while (1)
  {
    uint32_t isr = USART1->ISR;

    if (isr & USART_ISR_ORE)
    {
      USART1->ICR |= USART_ICR_ORECF;
    }

    if (isr & USART_ISR_FE)
    {
      USART1->ICR |= USART_ICR_FECF;
    }

    if (isr & USART_ISR_RXNE_RXFNE)
    {
      return (uint8_t)USART1->RDR;
    }
  }
}

void uart1_tx_string(char *str)
{
  while (*str)
  {
    uart1_tx((uint8_t)*str++);
  }
}

void test_tx(void)
{
  while (1)
  {
    uart1_tx_string("SupWill\r\n");
    HAL_Delay(100);
  }
}

void uart1_rx_string(char *buffer)
{
  int i = 0;
  char c;

  while (1)
  {
    c = (char)uart1_rx();

    if (c == '\r')
    {
      continue;
    }

    if (c == '\n' || i >= 127)
    {
      buffer[i] = '\0';
      return;
    }

    buffer[i++] = c;
  }
}

int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

static void UART_QuickTest(void)
{
  static const uint8_t test_message[] =
      "\r\n*** ESC UART TEST: USART1 TX on PC4, 9600 8-N-1 ***\r\n";
  uint8_t i;

  /* Use HAL directly so this also tests UART independently of printf. */
  for (i = 1U; i <= 5U; ++i)
  {
    HAL_UART_Transmit(&huart1,
                      (uint8_t *)test_message,
                      (uint16_t)(sizeof(test_message) - 1U),
                      1000U);
    HAL_UART_Transmit(&huart1, (uint8_t *)"UART test pulse\r\n", 17U, 1000U);
    HAL_Delay(500U);
  }
}

static void LED_FlashTest(void)
{
  HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
  Motor_PWM_Off();

  while (1)
  {
    Debug_Status1_Set(GPIO_PIN_SET);
    Debug_Status2_Set(GPIO_PIN_RESET);
    HAL_Delay(250U);
    Debug_Status1_Set(GPIO_PIN_RESET);
    Debug_Status2_Set(GPIO_PIN_SET);
    HAL_Delay(250U);
  }
}

static void Motor_TestCooldownCountdown(void)
{
  uint32_t remaining;

  Motor_PWM_Off();
  printf("PWM off; starting %u-second cooldown\r\n", MOTOR_TEST_COOLDOWN_SEC);

  for (remaining = MOTOR_TEST_COOLDOWN_SEC; remaining > 0U; --remaining)
  {
    printf("Cooldown: %lu seconds remaining\r\n", (unsigned long)remaining);
    HAL_Delay(1000U);
  }

  printf("Cooldown complete; starting 1 us comparison run\r\n");
}

void Start_PWM(void)
{
  Debug_Status1_Set(GPIO_PIN_RESET);
  Debug_Status2_Set(GPIO_PIN_RESET);
  Motor_PWM_Off();

  Set_Phase_PWM_Ticks(__HAL_TIM_GET_AUTORELOAD(&htim1) / 2U,
                      __HAL_TIM_GET_AUTORELOAD(&htim1) / 2U,
                      __HAL_TIM_GET_AUTORELOAD(&htim1) / 2U);

  __HAL_TIM_MOE_ENABLE(&htim1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  Motor_PWM_Off();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  if (!Boot_ForceMainFlashOptionBytes())
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
#if UART_OUTPUT_ENABLE
  /* Bring-up banner must run before unrelated peripherals can enter Error_Handler(). */
  UART_QuickTest();
#endif
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
#if LED_FLASH_TEST_ENABLE
  LED_FlashTest();
#endif
  DWT_Init();
  Motor_PWM_Off();
#if MOTOR_ENCODER_TEST_ENABLE
  AS5048A_EncoderTestLoop();
#else
#if MOTOR_FOC_DEMO_ENABLE
  AS5048A_InitChipSelect();
  AS5048A_PrintTest();
#endif
#if DRV_SPI_BITBANG_GPIO_TEST && (DRV_SPI_CONTINUOUS_READ_TEST || \
    DRV_SPI_CONFIGURATION_ENABLE || DRV_SPI_CSA_CALIBRATION_TRIAL)
  /* Reassign the crossed PA6/PA7 traces before the DRV is enabled. */
  DRV8353_BitBangInit();
#endif
  HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
#if DRV_SPI_CONTINUOUS_READ_TEST
  printf("DRV8353S read-only GPIO SPI diagnostic: PA6 TX->SDI, PA7 RX<-SDO; PWM inhibited\r\n");
  while (1)
  {
    (void)DRV8353_DumpRegisters("continuous read test");
    HAL_Delay(1000U);
  }
#endif
#if DRV_SPI_CSA_CALIBRATION_TRIAL
  if (!DRV8353_DumpRegisters("before CSA calibration") ||
      !DRV8353_PrintCSAGain())
  {
    Error_Handler();
  }
#endif
#if DRV_SPI_CONFIGURATION_ENABLE
  if (!DRV8353_DumpRegisters("before configuration"))
  {
    Error_Handler();
  }
  if (!DRV8353_ConfigureSixPWM())
  {
    Error_Handler();
  }
  DRV8353_UpdateFaultLED();

  if (!DRV8353_WriteAndVerifyCSA())
  {
    Error_Handler();
  }

  if (!DRV8353_PrintCSAGain())
  {
    Error_Handler();
  }
#else
#if UART_OUTPUT_ENABLE
  printf("DRV full configuration disabled; preserving verified reset settings\r\n");
#endif
#endif

  Start_PWM();
  if (!CurrentSense_CalibrateOffsets())
  {
    Motor_PWM_Off();
    Error_Handler();
  }
#if DRV_SPI_CSA_CALIBRATION_TRIAL
  if (!DRV8353_PrintCSAGain())
  {
    Motor_PWM_Off();
    Error_Handler();
  }
#endif
#if UART_OUTPUT_ENABLE
  Print_Frequency_Info();
#if MOTOR_FIELD_STEP_DEMO_ENABLE
  printf("Selected test: stepped static SVPWM field orientation\r\n");
  printf("Keep the shaft unloaded; each point is a two-second static hold\r\n");
#elif MOTOR_FOC_DEMO_ENABLE
  printf("The shaft must be unloaded and free to rotate during automatic alignment\r\n");
#if FOC_POSITION_DEMO_ENABLE
  printf("Selected test: geared output-position PD to Iq FOC demonstration\r\n");
  printf("FOC AS5048A=motor-side, gearbox=%ld/1000 motor rev/output rev, direction=%ld\r\n",
         (long)(FOC_MOTOR_TO_OUTPUT_GEAR_RATIO * 1000.0f),
         (long)FOC_OUTPUT_DIRECTION_SIGN);
  printf("FOC output-frame position P=%ld/1000 A/output-deg, D=%ld/1000 A/output-rpm, no integral or breakaway pulse\r\n",
         (long)(FOC_POSITION_KP_A_PER_OUTPUT_DEG * 1000.0f),
         (long)(FOC_POSITION_KD_A_PER_OUTPUT_RPM * 1000.0f));
  printf("FOC output max=%ld x0.001 rpm, accel/decel=%ld/%ld x0.001 rpm/s; motor max=%ld rpm, accel/decel=%ld/%ld rpm/s, Iq clamp=%ld mA\r\n",
         (long)(FOC_POSITION_TRAJECTORY_MAX_OUTPUT_RPM * 1000.0f),
         (long)(FOC_POSITION_TRAJECTORY_ACCEL_OUTPUT_RPM_S * 1000.0f),
         (long)(FOC_POSITION_TRAJECTORY_DECEL_OUTPUT_RPM_S * 1000.0f),
         (long)((FOC_POSITION_TRAJECTORY_MAX_OUTPUT_RPM *
                 FOC_MOTOR_TO_OUTPUT_GEAR_RATIO) + 0.5f),
         (long)((FOC_POSITION_TRAJECTORY_ACCEL_OUTPUT_RPM_S *
                 FOC_MOTOR_TO_OUTPUT_GEAR_RATIO) + 0.5f),
         (long)((FOC_POSITION_TRAJECTORY_DECEL_OUTPUT_RPM_S *
                 FOC_MOTOR_TO_OUTPUT_GEAR_RATIO) + 0.5f),
         (long)(FOC_POSITION_IQ_LIMIT_A * 1000.0f));
  printf("FOC protections: modulation=%ld/10000, hard phase current=%ld mA, abnormal d/q current=%ld mA\r\n",
         (long)(FOC_MAX_MODULATION * 10000.0f),
         (long)(FOC_POSITION_HARD_CURRENT_LIMIT_A * 1000.0f),
         (long)(FOC_POSITION_DQ_FAULT_LIMIT_A * 1000.0f));
#elif FOC_CURRENT_STEP_TEST_ENABLE
  printf("Selected test: encoder-locked d-axis current-step diagnostic\r\n");
  printf("FOC current-step firmware revision=2 (tick-synchronized)\r\n");
  printf("FOC test holds Iq at zero and steps Id 0 -> %ld -> %ld -> 0 mA over %lu ms\r\n",
         (long)(FOC_CURRENT_STEP_1_A * 1000.0f),
         (long)(FOC_CURRENT_STEP_2_A * 1000.0f),
         (unsigned long)FOC_CURRENT_STEP_TEST_DURATION_MS);
  printf("The rotor should align first, then remain essentially stationary during the current steps\r\n");
#else
#if FOC_LOW_SPEED_VELOCITY_TEST_ENABLE
  printf("Selected test: standalone +10 RPM velocity PI characterization\r\n");
  printf("FOC low-speed velocity PI: Kp=%ld/1000 A/rpm, Ki=%ld/1000 A/(rpm*s), integral=%ld mA, Iq=%ld mA, overspeed=%ld rpm\r\n",
         (long)(FOC_LOW_SPEED_KP_A_PER_RPM * 1000.0f),
         (long)(FOC_LOW_SPEED_KI_A_PER_RPM_S * 1000.0f),
         (long)(FOC_LOW_SPEED_INTEGRAL_LIMIT_A * 1000.0f),
         (long)(FOC_LOW_SPEED_IQ_LIMIT_A * 1000.0f),
         (long)FOC_LOW_SPEED_OVERSPEED_RPM);
#else
  printf("Selected test: encoder-based Id/Iq current-loop FOC bring-up\r\n");
  printf("FOC speed target=%ld rpm, ramp=%ld rpm/s, Iq limit=%ld mA, modulation=%ld/10000, hard current=%ld mA, overspeed=%ld rpm\r\n",
         (long)FOC_SPEED_TARGET_RPM,
         (long)FOC_SPEED_REFERENCE_RAMP_RPM_S,
         (long)(FOC_IQ_TARGET_A * 1000.0f),
         (long)(FOC_MAX_MODULATION * 10000.0f),
         (long)(FOC_HARD_CURRENT_LIMIT_A * 1000.0f),
         (long)FOC_MAX_MECHANICAL_RPM);
#endif
#endif
#else
  printf("Finite open-loop SVPWM waveform capture, approximately 500 ns dead time\r\n");
  printf("Startup: 1 s fixed-angle alignment, then 0.5 eHz toward 500 eHz at 2 eHz/s\r\n");
  printf("Modulation: 0.020 to 0.040 over 4 s, then to 0.100 by 50 s\r\n");
  printf("Waveform capture: %u samples at 1 kHz, beginning %u ms into the ramp\r\n",
         SVPWM_LOG_CAPACITY, SVPWM_CAPTURE_START_MS);
#if MOTOR_PHASE_CURRENT_LIMIT_ENABLE
  printf("Synchronized software phase-current cutoff: %.1f A\r\n",
         MOTOR_PHASE_CURRENT_LIMIT_A);
#else
  printf("WARNING: software phase-current cutoff is disabled\r\n");
#endif
  printf("DRV8353S SPI used only for verified CSA calibration; full configuration unchanged\r\n");
#endif
#endif
#if MOTOR_FIELD_STEP_DEMO_ENABLE
  Motor_FieldOrientationDemo(MOTOR_DEADTIME_500NS_TICKS);
  printf("Field-orientation demonstration finished; PWM remains inhibited\r\n");
#elif MOTOR_FOC_DEMO_ENABLE
  Motor_FOC_Demo(MOTOR_DEADTIME_500NS_TICKS);
  printf("FOC bring-up demonstration finished; PWM remains inhibited\r\n");
#else
  BLDC_OpenLoop_SVPWM_RampLoop(MOTOR_DEADTIME_500NS_TICKS, true);
  printf("Current capture complete; PWM remains inhibited\r\n");
#endif
#endif /* MOTOR_ENCODER_TEST_ENABLE */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 28;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  /* Required by this HAL to build a multi-rank injected JSQR sequence. */
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  /* JEOS interrupt only after all three injected phase-current ranks complete. */
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* TIM1 TRGO2 starts one deterministic A/B/C injected sequence per PWM period. */
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.InjecOversamplingMode = DISABLE;

  sConfigInjected.InjectedChannel = ADC_CHANNEL_6;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) Error_Handler();
  sConfigInjected.InjectedChannel = ADC_CHANNEL_7;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) Error_Handler();
  sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) Error_Handler();

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 1;
  hfdcan1.Init.NominalTimeSeg2 = 1;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x20D192D5;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  /* 112 MHz / 16 = 7 MHz. The AS5048A allows up to 10 MHz; /8 would
   * produce 14 MHz and violate its 100 ns minimum SPI clock period. */
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2799;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 50;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* Internal PWM2 OC4REF rises on the up-count, shortly before the ARR peak. */
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 2799U - CURRENT_SAMPLE_BEFORE_TIM1_PEAK_TICKS;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = MOTOR_TIM1_DEADTIME_TICKS;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DRV_SCS_N_GPIO_Port, DRV_SCS_N_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STATUS_1_Pin|STATUS_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DRV_FAULT_Pin */
  GPIO_InitStruct.Pin = DRV_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRV_FAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DRV_ENABLE_Pin */
  GPIO_InitStruct.Pin = DRV_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DRV_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DRV_SCS_N_Pin */
  GPIO_InitStruct.Pin = DRV_SCS_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DRV_SCS_N_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STATUS_1_Pin STATUS_2_Pin */
  GPIO_InitStruct.Pin = STATUS_1_Pin|STATUS_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VSENSE_Pin */
  GPIO_InitStruct.Pin = USB_VSENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VSENSE_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
