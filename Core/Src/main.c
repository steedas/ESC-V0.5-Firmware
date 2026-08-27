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
#define CURRENT_SENSE_DEBUG_PRINT 0U
#define MOTOR_PHASE_CURRENT_LIMIT_ENABLE 1U
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
#define CAN_FIRMWARE_ROLE_ESC              1U
#define CAN_FIRMWARE_ROLE_DEMO_CONTROLLER  2U
#ifndef CAN_FIRMWARE_ROLE
#define CAN_FIRMWARE_ROLE 0U
#endif
#define CAN_CONTROL_ENABLE \
    (CAN_FIRMWARE_ROLE == CAN_FIRMWARE_ROLE_ESC)
#define CAN_DEMO_DEVICE2_ENABLE \
    (CAN_FIRMWARE_ROLE == CAN_FIRMWARE_ROLE_DEMO_CONTROLLER)
#define MOTOR_SVPWM_CONTINUOUS_RUN 0U
#define MOTOR_FIELD_STEP_DEMO_ENABLE 0U
#define MOTOR_FOC_DEMO_ENABLE        1U
#define MOTOR_ENCODER_TEST_ENABLE    0U
#define WILL_SVPWM_PROBE_ELECTRICAL_HZ      100
#define WILL_SVPWM_PROBE_DURATION_MS      120000
#define WILL_SVPWM_PROBE_MODULATION_STRENGTH 0.05f
#define WILL_SVPWM_PROBE_START_ELECTRICAL_HZ 0.10f
#define WILL_SVPWM_PROBE_START_MODULATION    0.025f
#define WILL_SVPWM_PROBE_ALIGNMENT_MS        1000U
#define WILL_SVPWM_PROBE_RAMP_MS            10000U
#define FOC_POSITION_DEMO_ENABLE     1U
#define FOC_TORQUE_ONLY_DEMO_ENABLE  0U
#define FOC_IMPEDANCE_DEMO_ENABLE    0U
#define FOC_PRE_POSITION_IMPEDANCE_ENABLE 0U
#define FOC_FORCE_SCALE_TEST_ENABLE  1U
#define FOC_COMPOSITE_DEMO_ENABLE    0U
#define FOC_VELOCITY_HEAT_TEST_ENABLE 0U
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
#define MOTOR_SVPWM_RUN_TIME_MS  10000U
#define MOTOR_SIX_STEP_RUN_TIME_MS 10000U
#define MOTOR_DEMO_HANDOFF_PAUSE_MS 1000U
#define MOTOR_SIX_STEP_HANDOFF_PAUSE_MS 2000U
#define MOTOR_DEMO_OPEN_LOOP_TARGET_EHZ 200.0f
#define MOTOR_DEMO_OPEN_LOOP_RAMP_EHZ_PER_SEC 20.0f
#define MOTOR_SIX_STEP_ALIGNMENT_HOLD_MS 750U
#define MOTOR_SIX_STEP_ALIGNMENT_DUTY_FRACTION 0.025f
#define MOTOR_SIX_STEP_START_ELECTRICAL_HZ 2.0f
#define MOTOR_SIX_STEP_DUTY_START_FRACTION 0.040f
#define MOTOR_SIX_STEP_DUTY_END_FRACTION 0.150f
#define SVPWM_CAPTURE_START_MS   5000U
#define SVPWM_LOG_INTERVAL_MS    1U
#if FOC_COMPOSITE_DEMO_ENABLE
/* The composite demo does not dump SVPWM current samples.  Avoid retaining
 * its separate 16 kB capture buffer alongside the FOC telemetry buffer. */
#define SVPWM_LOG_CAPACITY       1U
#else
#define SVPWM_LOG_CAPACITY       500U
#endif
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
/* Position targets, trajectory limits, and outer PID gains are expressed at
 * the gearbox output. The motor reference is multiplied by the ratio only at
 * the boundary to the motor-side encoder/current FOC. */
#define FOC_POSITION_KP_A_PER_OUTPUT_DEG     1.500f
#define FOC_POSITION_KD_A_PER_OUTPUT_RPM     0.350f
#define FOC_POSITION_KI_A_PER_OUTPUT_DEG_S   0.250f
#define FOC_POSITION_INTEGRAL_LIMIT_A        8.000f
#define FOC_POSITION_INTEGRAL_UNWIND_MULTIPLIER 5.000f
#define FOC_POSITION_DISTURBANCE_ONSET_OUTPUT_DEG   1.000f
#define FOC_POSITION_DISTURBANCE_FULL_OUTPUT_DEG    3.000f
#define FOC_POSITION_DISTURBANCE_FADE_OUTPUT_RPM   40.000f
#define FOC_POSITION_DISTURBANCE_FULL_SPEED_ERROR_RPM 10.000f
#define FOC_POSITION_DISTURBANCE_BOOST_A           18.000f
#define FOC_POSITION_TRAJECTORY_MAX_OUTPUT_RPM      400.0f
#define FOC_POSITION_TRAJECTORY_ACCEL_OUTPUT_RPM_S  600.0f
#define FOC_POSITION_TRAJECTORY_DECEL_OUTPUT_RPM_S  600.0f
/* MN6007II KV320 high-current position-test profile.  T-Motor publishes a
 * 44.2 A/180 s motor peak and recommends a 60 A ESC.  The operating command
 * stays below the motor peak; the two higher values are fault thresholds, not
 * normal commands.  With 1 mOhm shunts and 20 V/V CSA gain, the ADC range is
 * approximately +/-82 A, so all software thresholds remain measurable. */
#define FOC_POSITION_OVERSPEED_RPM       4000.0f
#define FOC_POSITION_IQ_LIMIT_A            30.0f
#define FOC_POSITION_DQ_FAULT_LIMIT_A       50.0f
#define FOC_POSITION_HARD_CURRENT_LIMIT_A  60.0f
#define FOC_OUTPUT_POSITION_TOLERANCE_DEG    3.0f
#define FOC_OUTPUT_SPEED_TOLERANCE_RPM       3.0f
#define FOC_POSITION_HOLD_MS              300U
#define FOC_POSITION_STEP_TIMEOUT_MS      6000U
#define FOC_POSITION_TEST_DURATION_MS    15000U
#define FOC_VELOCITY_DEMO_DURATION_MS    15000U
#define FOC_VELOCITY_SETTLE_TIMEOUT_MS    3000U
#define FOC_VELOCITY_DECEL_START_MS      11000U
#define FOC_VELOCITY_TARGET_OUTPUT_RPM     400.0f
#define FOC_VELOCITY_ACCEL_OUTPUT_RPM_S    100.0f
#define FOC_VELOCITY_DECEL_OUTPUT_RPM_S    100.0f
#define FOC_VELOCITY_KP_A_PER_OUTPUT_RPM     0.050f
#define FOC_VELOCITY_KI_A_PER_OUTPUT_RPM_S   0.400f
#define FOC_VELOCITY_INTEGRAL_LIMIT_A       20.000f
#define FOC_VELOCITY_INTEGRAL_UNWIND_MULTIPLIER 3.000f
#define FOC_VELOCITY_STOP_TOLERANCE_OUTPUT_RPM   5.0f
#define FOC_VELOCITY_OVERSPEED_MOTOR_RPM       5000.0f
#define FOC_HEAT_TEST_TARGET_MOTOR_RPM          5000.0f
#define FOC_HEAT_TEST_TARGET_OUTPUT_RPM         \
    (FOC_HEAT_TEST_TARGET_MOTOR_RPM / FOC_MOTOR_TO_OUTPUT_GEAR_RATIO)
#define FOC_HEAT_TEST_ACCEL_OUTPUT_RPM_S         100.0f
#define FOC_HEAT_TEST_DECEL_OUTPUT_RPM_S         100.0f
#define FOC_HEAT_TEST_HOLD_MS                   60000U
#define FOC_HEAT_TEST_MEASUREMENT_START_MS       5000U
#define FOC_HEAT_TEST_DECEL_START_MS            65000U
#define FOC_HEAT_TEST_DURATION_MS               70000U
#define FOC_HEAT_TEST_SETTLE_TIMEOUT_MS          3000U
#define FOC_HEAT_TEST_TIMING_MARGIN_MS           1000U
#define FOC_HEAT_TEST_ARMING_PAUSE_SEC              5U
#define FOC_HEAT_TEST_OVERSPEED_MOTOR_RPM        5600.0f

#if FOC_VELOCITY_HEAT_TEST_ENABLE
#define FOC_ACTIVE_VELOCITY_DEMO_DURATION_MS \
    FOC_HEAT_TEST_DURATION_MS
#define FOC_ACTIVE_VELOCITY_SETTLE_TIMEOUT_MS \
    FOC_HEAT_TEST_SETTLE_TIMEOUT_MS
#define FOC_ACTIVE_VELOCITY_DECEL_START_MS \
    FOC_HEAT_TEST_DECEL_START_MS
#define FOC_ACTIVE_VELOCITY_TARGET_OUTPUT_RPM \
    FOC_HEAT_TEST_TARGET_OUTPUT_RPM
#define FOC_ACTIVE_VELOCITY_ACCEL_OUTPUT_RPM_S \
    FOC_HEAT_TEST_ACCEL_OUTPUT_RPM_S
#define FOC_ACTIVE_VELOCITY_DECEL_OUTPUT_RPM_S \
    FOC_HEAT_TEST_DECEL_OUTPUT_RPM_S
#define FOC_ACTIVE_VELOCITY_OVERSPEED_MOTOR_RPM \
    FOC_HEAT_TEST_OVERSPEED_MOTOR_RPM
#else
#define FOC_ACTIVE_VELOCITY_DEMO_DURATION_MS \
    FOC_VELOCITY_DEMO_DURATION_MS
#define FOC_ACTIVE_VELOCITY_SETTLE_TIMEOUT_MS \
    FOC_VELOCITY_SETTLE_TIMEOUT_MS
#define FOC_ACTIVE_VELOCITY_DECEL_START_MS \
    FOC_VELOCITY_DECEL_START_MS
#define FOC_ACTIVE_VELOCITY_TARGET_OUTPUT_RPM \
    FOC_VELOCITY_TARGET_OUTPUT_RPM
#define FOC_ACTIVE_VELOCITY_ACCEL_OUTPUT_RPM_S \
    FOC_VELOCITY_ACCEL_OUTPUT_RPM_S
#define FOC_ACTIVE_VELOCITY_DECEL_OUTPUT_RPM_S \
    FOC_VELOCITY_DECEL_OUTPUT_RPM_S
#define FOC_ACTIVE_VELOCITY_OVERSPEED_MOTOR_RPM \
    FOC_VELOCITY_OVERSPEED_MOTOR_RPM
#endif
#define FOC_TORQUE_DEMO_DURATION_MS           10000U
#define FOC_TORQUE_SETTLE_TIMEOUT_MS           1000U
#define FOC_TORQUE_POSITIVE_END_MS             3000U
#define FOC_TORQUE_REVERSE_START_MS            5000U
#define FOC_TORQUE_REVERSE_END_MS              8000U
#define FOC_MOTOR_ESTIMATED_KT_NM_PER_A            0.02984f
#define FOC_TORQUE_TARGET_MOTOR_NM                  0.240f
#define FOC_TORQUE_RAMP_MOTOR_NM_PER_S              0.240f
#define FOC_TORQUE_LOAD_PAUSE_SEC                        5U
#define FOC_TORQUE_OVERSPEED_MOTOR_RPM          1500.0f
#define FOC_FORCE_TEST_CCW_SIGN                      -1.0f
#define FOC_FORCE_TEST_CONTACT_OUTPUT_DEG            45.0f
#define FOC_FORCE_TEST_APPROACH_MAX_OUTPUT_RPM        2.0f
#define FOC_FORCE_TEST_APPROACH_ACCEL_OUTPUT_RPM_S    4.0f
#define FOC_FORCE_TEST_APPROACH_IQ_LIMIT_A            6.0f
#define FOC_FORCE_TEST_CONTACT_SETTLE_MS             1000U
#define FOC_FORCE_TEST_CONTACT_MIN_TRAVEL_DEG          0.25f
#define FOC_FORCE_TEST_CONTACT_MAX_OUTPUT_RPM          0.25f
#define FOC_FORCE_TEST_CONTACT_MIN_IQ_A                5.5f
#define FOC_FORCE_TEST_CONTACT_MIN_REMAINING_DEG       3.0f
#define FOC_FORCE_TEST_CONTACT_CONFIRM_MS             500U
#define FOC_FORCE_TEST_TARGET_IQ_A                    70.0f
#define FOC_FORCE_TEST_IQ_RAMP_A_PER_S                10.0f
#define FOC_FORCE_TEST_TORQUE_RAMP_MS                 7000U
#define FOC_FORCE_TEST_TORQUE_HOLD_MS                  250U
#define FOC_FORCE_TEST_TORQUE_DURATION_MS \
    ((2U * FOC_FORCE_TEST_TORQUE_RAMP_MS) + \
     FOC_FORCE_TEST_TORQUE_HOLD_MS)
#define FOC_FORCE_TEST_TORQUE_SETTLE_TIMEOUT_MS       1000U
#define FOC_FORCE_TEST_ARMING_PAUSE_SEC                   5U
#define FOC_FORCE_TEST_APPROACH_OVERSPEED_MOTOR_RPM   120.0f
#define FOC_FORCE_TEST_TORQUE_OVERSPEED_MOTOR_RPM     150.0f
#define FOC_FORCE_TEST_APPROACH_HARD_CURRENT_LIMIT_A    12.0f
#define FOC_FORCE_TEST_APPROACH_DQ_FAULT_LIMIT_A        10.0f
#define FOC_FORCE_TEST_HARD_CURRENT_LIMIT_A            75.0f
#define FOC_FORCE_TEST_DQ_FAULT_LIMIT_A                72.0f
/* Update this to the measured shaft-center to scale-contact distance. */
#define FOC_FORCE_TEST_LEVER_ARM_MM                    238.1f
#if FOC_FORCE_SCALE_TEST_ENABLE
#define FOC_ACTIVE_POSITION_MAX_OUTPUT_RPM \
    FOC_FORCE_TEST_APPROACH_MAX_OUTPUT_RPM
#define FOC_ACTIVE_POSITION_ACCEL_OUTPUT_RPM_S \
    FOC_FORCE_TEST_APPROACH_ACCEL_OUTPUT_RPM_S
#define FOC_ACTIVE_POSITION_DECEL_OUTPUT_RPM_S \
    FOC_FORCE_TEST_APPROACH_ACCEL_OUTPUT_RPM_S
#define FOC_ACTIVE_POSITION_HOLD_MS \
    FOC_FORCE_TEST_CONTACT_SETTLE_MS
#define FOC_ACTIVE_POSITION_KP_A_PER_OUTPUT_DEG       0.500f
#define FOC_ACTIVE_POSITION_KI_A_PER_OUTPUT_DEG_S     0.000f
#define FOC_ACTIVE_POSITION_KD_A_PER_OUTPUT_RPM       0.500f
#define FOC_ACTIVE_TORQUE_TARGET_MOTOR_NM \
    (FOC_FORCE_TEST_TARGET_IQ_A * FOC_MOTOR_ESTIMATED_KT_NM_PER_A)
#define FOC_ACTIVE_TORQUE_RAMP_MOTOR_NM_PER_S \
    (FOC_FORCE_TEST_IQ_RAMP_A_PER_S * FOC_MOTOR_ESTIMATED_KT_NM_PER_A)
#define FOC_ACTIVE_TORQUE_DURATION_MS \
    FOC_FORCE_TEST_TORQUE_DURATION_MS
#define FOC_ACTIVE_TORQUE_SETTLE_TIMEOUT_MS \
    FOC_FORCE_TEST_TORQUE_SETTLE_TIMEOUT_MS
#else
#define FOC_ACTIVE_POSITION_MAX_OUTPUT_RPM \
    FOC_POSITION_TRAJECTORY_MAX_OUTPUT_RPM
#define FOC_ACTIVE_POSITION_ACCEL_OUTPUT_RPM_S \
    FOC_POSITION_TRAJECTORY_ACCEL_OUTPUT_RPM_S
#define FOC_ACTIVE_POSITION_DECEL_OUTPUT_RPM_S \
    FOC_POSITION_TRAJECTORY_DECEL_OUTPUT_RPM_S
#define FOC_ACTIVE_POSITION_HOLD_MS FOC_POSITION_HOLD_MS
#define FOC_ACTIVE_POSITION_KP_A_PER_OUTPUT_DEG \
    FOC_POSITION_KP_A_PER_OUTPUT_DEG
#define FOC_ACTIVE_POSITION_KI_A_PER_OUTPUT_DEG_S \
    FOC_POSITION_KI_A_PER_OUTPUT_DEG_S
#define FOC_ACTIVE_POSITION_KD_A_PER_OUTPUT_RPM \
    FOC_POSITION_KD_A_PER_OUTPUT_RPM
#define FOC_ACTIVE_TORQUE_TARGET_MOTOR_NM FOC_TORQUE_TARGET_MOTOR_NM
#define FOC_ACTIVE_TORQUE_RAMP_MOTOR_NM_PER_S \
    FOC_TORQUE_RAMP_MOTOR_NM_PER_S
#define FOC_ACTIVE_TORQUE_DURATION_MS FOC_TORQUE_DEMO_DURATION_MS
#define FOC_ACTIVE_TORQUE_SETTLE_TIMEOUT_MS \
    FOC_TORQUE_SETTLE_TIMEOUT_MS
#endif
#define FOC_IMPEDANCE_TARGET_OUTPUT_DEG                 0.0f
#define FOC_IMPEDANCE_DEMO_DURATION_MS                60000U
#define FOC_COMPOSITE_IMPEDANCE_DURATION_MS           15000U
#define FOC_COMPOSITE_TIMING_MARGIN_MS                 1000U
#define FOC_IMPEDANCE_RELEASE_RAMP_MS                  3000U
#define FOC_IMPEDANCE_SETTLE_TIMEOUT_MS                 500U
#define FOC_IMPEDANCE_ARMING_PAUSE_SEC                    5U
#define FOC_IMPEDANCE_STIFFNESS_NM_PER_OUTPUT_DEG      0.500f
#define FOC_IMPEDANCE_DAMPING_A_PER_OUTPUT_RPM         0.300f
#define FOC_IMPEDANCE_IQ_LIMIT_A                      50.000f
#define FOC_IMPEDANCE_DQ_FAULT_LIMIT_A                55.000f
#define FOC_IMPEDANCE_HARD_CURRENT_LIMIT_A            60.000f
#define FOC_IMPEDANCE_OVERSPEED_MOTOR_RPM            3000.0f
#define FOC_TEST_DURATION_MS             12000U
#define FOC_ENCODER_UPDATE_US             125U
#define FOC_ENCODER_SENSOR_DELAY_US        200U
#define FOC_ENCODER_PREDICTION_MAX_US     1200U
#define FOC_ENCODER_MAX_INNOVATION_RAD    3.141593f
#define FOC_ENCODER_CORRECTION_GAIN       0.200f
#define FOC_ENCODER_MAX_CORRECTION_RAD    0.020f
#define FOC_ENCODER_VELOCITY_GAIN         0.025f
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
#define FOC_IMPEDANCE_LOG_INTERVAL_MS     500U
#define FOC_VELOCITY_LOG_INTERVAL_MS     1000U
#define FOC_TORQUE_LOG_INTERVAL_MS        250U
#if FOC_CURRENT_STEP_TEST_ENABLE
#define FOC_LOG_CAPACITY                  105U
#elif FOC_POSITION_DEMO_ENABLE
#define FOC_LOG_CAPACITY                  140U
#elif FOC_LOW_SPEED_VELOCITY_TEST_ENABLE
#define FOC_LOG_CAPACITY                  160U
#else
#define FOC_LOG_CAPACITY                  202U
#endif
#define FOC_MIN_CURRENT_SAMPLES_PER_LOG   900U
#define FOC_POSITION_MIN_CURRENT_SAMPLES_PER_LOG 3000U
#define FOC_IMPEDANCE_MIN_CURRENT_SAMPLES_PER_LOG 7500U
#define FOC_VELOCITY_MIN_CURRENT_SAMPLES_PER_LOG 15000U
#define FOC_TORQUE_MIN_CURRENT_SAMPLES_PER_LOG 3750U
#define FOC_PREFAULT_CAPTURE_ENABLE         0U
#define FOC_PREFAULT_LOG_CAPACITY           32U
#define ENCODER_TEST_INTERVAL_MS           20U
#define ENCODER_TEST_PRINT_INTERVAL_MS     100U
#define MOTOR_DEADTIME_100NS_TICKS 11U
#define MOTOR_DEADTIME_500NS_TICKS 56U
#define MOTOR_DEADTIME_1US_TICKS   112U
#define MOTOR_TIM1_DEADTIME_TICKS  MOTOR_DEADTIME_100NS_TICKS
#define MOTOR_TEST_COOLDOWN_SEC    120U

/* Minimal external-control protocol, revision 1.  Lower CAN identifiers have
 * higher arbitration priority, so the global E-stop is intentionally ID 0. */
#define CAN_PROTOCOL_VERSION          1U
#define CAN_NODE_ID                   1U
#define CAN_GLOBAL_ESTOP_ID           0x000U
#define CAN_HEARTBEAT_BASE_ID         0x080U
#define CAN_STATE_COMMAND_BASE_ID     0x100U
#define CAN_HEARTBEAT_ID              (CAN_HEARTBEAT_BASE_ID + CAN_NODE_ID)
#define CAN_STATE_COMMAND_ID          (CAN_STATE_COMMAND_BASE_ID + CAN_NODE_ID)
#define CAN_HEARTBEAT_PERIOD_MS       100U

#define CAN_STATUS_CAN_STARTED        (1U << 0)
#define CAN_STATUS_COMMAND_SEEN       (1U << 1)
#define CAN_STATUS_OUTPUT_ENABLED     (1U << 2)
#define CAN_STATUS_INVALID_COMMAND    (1U << 3)
#define CAN_STATUS_ESTOP_LATCHED      (1U << 4)
#define CAN_STATUS_CONTROL_PLANE_ONLY (1U << 5)

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
  uint32_t time_ms;
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

typedef enum
{
  CAN_DRIVE_STATE_BOOT = 0U,
  CAN_DRIVE_STATE_DISABLED = 1U,
  CAN_DRIVE_STATE_ARMED = 2U,
  CAN_DRIVE_STATE_ACTIVE = 3U,
  CAN_DRIVE_STATE_FAULT = 4U
} CAN_DriveState_t;

typedef enum
{
  CAN_STATE_REQUEST_DISABLED = 1U,
  CAN_STATE_REQUEST_ARMED = 2U,
  CAN_STATE_REQUEST_ACTIVE = 3U,
  CAN_STATE_REQUEST_CLEAR_FAULT = 4U
} CAN_StateRequest_t;

typedef enum
{
  CAN_COMMAND_RESULT_NONE = 0U,
  CAN_COMMAND_RESULT_ACCEPTED = 1U,
  CAN_COMMAND_RESULT_INVALID_LENGTH = 2U,
  CAN_COMMAND_RESULT_INVALID_REQUEST = 3U,
  CAN_COMMAND_RESULT_INVALID_TRANSITION = 4U,
  CAN_COMMAND_RESULT_FAULT_STILL_PRESENT = 5U,
  CAN_COMMAND_RESULT_ESTOP = 6U
} CAN_CommandResult_t;

typedef enum
{
  CAN_DRIVE_FAULT_NONE = 0U,
  CAN_DRIVE_FAULT_ESTOP = 1U,
  CAN_DRIVE_FAULT_LOCAL_BASE = 0x80U
} CAN_DriveFault_t;

#if CAN_DEMO_DEVICE2_ENABLE
typedef struct
{
  const char *name;
  uint32_t id;
  uint8_t length;
  uint8_t data[2];
  uint8_t expected_state;
  uint8_t expected_fault;
  uint8_t expected_result;
  uint8_t expected_sequence;
  uint16_t delay_after_ack_ms;
} CAN_DemoStep_t;
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
#if FOC_FORCE_SCALE_TEST_ENABLE
static const float g_foc_output_position_demo_targets_deg[] =
{
  FOC_FORCE_TEST_CCW_SIGN * FOC_FORCE_TEST_CONTACT_OUTPUT_DEG
};
#else
static const float g_foc_output_position_demo_targets_deg[] =
{
  0.0f, 45.0f, 90.0f, 135.0f, 180.0f,
  225.0f, 270.0f, 315.0f, 360.0f,
  315.0f, 270.0f, 225.0f, 180.0f, 135.0f,
  90.0f, 45.0f, 0.0f
};
#endif
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
static CAN_DriveState_t g_can_drive_state = CAN_DRIVE_STATE_BOOT;
static uint8_t g_can_drive_fault = CAN_DRIVE_FAULT_NONE;
static uint8_t g_can_status_flags = CAN_STATUS_CONTROL_PLANE_ONLY;
static uint8_t g_can_last_command_sequence = 0U;
static CAN_CommandResult_t g_can_last_command_result = CAN_COMMAND_RESULT_NONE;
static uint8_t g_can_tx_drop_count = 0U;
static uint8_t g_can_invalid_rx_count = 0U;
static uint32_t g_can_last_heartbeat_tick = 0U;

#if WILL_SVPWM_EXPERIMENT_ENABLE
volatile uint8_t svpwm_update_due = 0U;
volatile uint32_t svpwm_update_count = 0U;
#endif

#if CAN_DEMO_DEVICE2_ENABLE
static const CAN_DemoStep_t g_can_demo_steps[] =
{
  {"ARMED", CAN_STATE_COMMAND_ID, 2U,
   {CAN_STATE_REQUEST_ARMED, 1U},
   CAN_DRIVE_STATE_ARMED, CAN_DRIVE_FAULT_NONE,
   CAN_COMMAND_RESULT_ACCEPTED, 1U, 750U},
  {"ACTIVE", CAN_STATE_COMMAND_ID, 2U,
   {CAN_STATE_REQUEST_ACTIVE, 2U},
   CAN_DRIVE_STATE_ACTIVE, CAN_DRIVE_FAULT_NONE,
   CAN_COMMAND_RESULT_ACCEPTED, 2U, 2000U},
  {"DISABLED", CAN_STATE_COMMAND_ID, 2U,
   {CAN_STATE_REQUEST_DISABLED, 3U},
   CAN_DRIVE_STATE_DISABLED, CAN_DRIVE_FAULT_NONE,
   CAN_COMMAND_RESULT_ACCEPTED, 3U, 750U},
  {"E-STOP", CAN_GLOBAL_ESTOP_ID, 0U, {0U, 0U},
   CAN_DRIVE_STATE_FAULT, CAN_DRIVE_FAULT_ESTOP,
   CAN_COMMAND_RESULT_ESTOP, 0xFFU, 750U},
  {"CLEAR FAULT", CAN_STATE_COMMAND_ID, 2U,
   {CAN_STATE_REQUEST_CLEAR_FAULT, 4U},
   CAN_DRIVE_STATE_DISABLED, CAN_DRIVE_FAULT_NONE,
   CAN_COMMAND_RESULT_ACCEPTED, 4U, 0U}
};
static uint8_t g_can_demo_step_index = 0U;
static uint8_t g_can_demo_started = 0U;
static uint8_t g_can_demo_waiting_for_ack = 0U;
static uint8_t g_can_demo_retry_count = 0U;
static uint8_t g_can_demo_complete = 0U;
static uint8_t g_can_demo_error = 0U;
static uint8_t g_can_demo_seen_heartbeat = 0U;
static uint8_t g_can_demo_last_state = 0xFFU;
static uint8_t g_can_demo_last_fault = 0xFFU;
static uint8_t g_can_demo_last_sequence = 0xFFU;
static uint8_t g_can_demo_last_result = 0xFFU;
static uint32_t g_can_demo_next_send_tick = 0U;
static uint32_t g_can_demo_last_send_tick = 0U;
static uint32_t g_can_demo_last_rx_tick = 0U;
static uint32_t g_can_demo_last_print_tick = 0U;
#endif
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
static void Motor_ReportDRVFaultAndShutdown(const char *stage);
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
static bool BLDC_SixStep_FiniteDemo(void);
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
#if WILL_SVPWM_EXPERIMENT_ENABLE
static inline float SVPWM_LookupSin(const float *sine_lut, float radians);
uint32_t Get_PWM_FrequencyHz(void);
void W_SVPWM_Update(int *CCR1, int *CCR2, int *CCR3,
                    float electrical_angle_rad, float modulation_strength,
                    const float *sine_lut);
void W_Assert_SVPWM_(int CCR1, int CCR2, int CCR3);
void W_SVPWM_Start(int electrical_frequency_hz, int duration_ms,
                   int SVPWM_update_frequency_hz, float modulation_strength);
/* Original, direct one-update-flag version kept intentionally simple. */
void W_SVPWM_Start_Efficient(int electrical_frequency_hz, int duration_ms,
                             int SVPWM_update_frequency_hz,
                             float modulation_strength);
/* Copy of the user ramp that uses the monotonic timer-event counter. */
void W_SVPWM_Start_Efficient_Ramp_Counter(
    int electrical_frequency_hz, int duration_ms,
    int SVPWM_update_frequency_hz, float modulation_strength);
/* Alternate scheduler with startup ramp and missed-update accounting. */
void W_SVPWM_Start_Efficient_Copilot(int electrical_frequency_hz,
                                     int duration_ms,
                                     int SVPWM_update_frequency_hz,
                                     float modulation_strength);
void Will_SVPWM_Implementation(int frequency_hz, int duration_ms);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void Start_PWM(void);
#endif
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

static void Motor_ReportDRVFaultAndShutdown(const char *stage)
{
  uint16_t fault_status1 = 0U;
  uint16_t vgs_status2 = 0U;
  bool status_valid;

  /* Remove switching first, but leave the driver powered long enough to
   * preserve and read its latched diagnostic registers. */
  Motor_PWM_Off();
  status_valid =
      DRV8353_ReadSPI_Safe(FAULT_STATUS1_REG_ADDR, &fault_status1) &&
      DRV8353_ReadSPI_Safe(VGS_STATUS2_REG_ADDR, &vgs_status2);
  if (status_valid)
  {
    g_drv_fault_status1 = fault_status1;
    g_drv_vgs_status2 = vgs_status2;
    printf("%s DRV8353S fault: status1=0x%03X, status2=0x%03X; disabling gate driver\r\n",
           stage, fault_status1, vgs_status2);
    printf("DRV fault decode: VDS_OCP=%u GDF=%u UVLO=%u OTSD=%u; SA_OC=%u SB_OC=%u SC_OC=%u OTW=%u GDUV=%u\r\n",
           (unsigned int)((fault_status1 >> 9) & 1U),
           (unsigned int)((fault_status1 >> 8) & 1U),
           (unsigned int)((fault_status1 >> 7) & 1U),
           (unsigned int)((fault_status1 >> 6) & 1U),
           (unsigned int)((vgs_status2 >> 10) & 1U),
           (unsigned int)((vgs_status2 >> 9) & 1U),
           (unsigned int)((vgs_status2 >> 8) & 1U),
           (unsigned int)((vgs_status2 >> 7) & 1U),
           (unsigned int)((vgs_status2 >> 6) & 1U));
    printf("DRV phase detail: VDS_HA=%u VDS_LA=%u VDS_HB=%u VDS_LB=%u VDS_HC=%u VDS_LC=%u; VGS_HA=%u VGS_LA=%u VGS_HB=%u VGS_LB=%u VGS_HC=%u VGS_LC=%u\r\n",
           (unsigned int)((fault_status1 >> 5) & 1U),
           (unsigned int)((fault_status1 >> 4) & 1U),
           (unsigned int)((fault_status1 >> 3) & 1U),
           (unsigned int)((fault_status1 >> 2) & 1U),
           (unsigned int)((fault_status1 >> 1) & 1U),
           (unsigned int)(fault_status1 & 1U),
           (unsigned int)((vgs_status2 >> 5) & 1U),
           (unsigned int)((vgs_status2 >> 4) & 1U),
           (unsigned int)((vgs_status2 >> 3) & 1U),
           (unsigned int)((vgs_status2 >> 2) & 1U),
           (unsigned int)((vgs_status2 >> 1) & 1U),
           (unsigned int)(vgs_status2 & 1U));
  }
  else
  {
    printf("%s DRV8353S nFAULT asserted; status-register read failed; disabling gate driver\r\n",
           stage);
  }
  HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
  Debug_Status1_Set(GPIO_PIN_SET);
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

#if WILL_SVPWM_EXPERIMENT_ENABLE
void W_SVPWM_Start(int electrical_frequency_hz, int duration_ms,
                   int SVPWM_update_frequency_hz, float modulation_strength)
{
  uint32_t start_tick = HAL_GetTick();
  uint32_t elapsed_ms = 0U;
  uint32_t elapsed_since_last_update_ms = 0U;
  float electrical_angle_rad = 0.0f;
  float electrical_velocity_rad_s =
      TWO_PI_F * (float)electrical_frequency_hz;

  FOC_SineLUTInit();
  HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
  Motor_PWM_Enable();

  while (elapsed_ms < (uint32_t)duration_ms)
  {
    elapsed_ms = HAL_GetTick() - start_tick;

    if ((float)(HAL_GetTick() - elapsed_since_last_update_ms) >=
        (1000.0f / (float)SVPWM_update_frequency_hz))
    {
      elapsed_since_last_update_ms = HAL_GetTick();
      electrical_angle_rad +=
          electrical_velocity_rad_s *
          (1.0f / (float)SVPWM_update_frequency_hz);
      electrical_angle_rad = WrapRadians(electrical_angle_rad);
      int CCR1 = 0;
      int CCR2 = 0;
      int CCR3 = 0;
      W_SVPWM_Update(&CCR1, &CCR2, &CCR3, electrical_angle_rad,
                     modulation_strength, g_foc_sine_lut);
      W_Assert_SVPWM_(CCR1, CCR2, CCR3);
    }
    else
    {
      continue;
    }
  }

  Motor_PWM_Off();
}

void W_SVPWM_Start_Efficient(int electrical_frequency_hz, int duration_ms,
                             int SVPWM_update_frequency_hz,
                             float modulation_strength)
{
  uint32_t start_tick = HAL_GetTick();
  uint32_t elapsed_ms = 0U;
  float electrical_angle_rad = 0.0f;
  float electrical_velocity_rad_s =
      TWO_PI_F * (float)electrical_frequency_hz;
  float dt = 1.0f / (float)SVPWM_update_frequency_hz;

  FOC_SineLUTInit();
  HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
  Motor_PWM_Enable();
  svpwm_update_due = 0U;
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC4);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);

  while (elapsed_ms < (uint32_t)duration_ms)
  {
    elapsed_ms = HAL_GetTick() - start_tick;

    if (HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin) ==
        GPIO_PIN_RESET)
    {
      __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
      Motor_ReportDRVFaultAndShutdown("W_SVPWM run");
      return;
    }

    if (svpwm_update_due)
    {
      svpwm_update_due = 0U;
      electrical_angle_rad += electrical_velocity_rad_s * dt;
      electrical_angle_rad = WrapRadians(electrical_angle_rad);

      int CCR1 = 0;
      int CCR2 = 0;
      int CCR3 = 0;

      W_SVPWM_Update(&CCR1, &CCR2, &CCR3, electrical_angle_rad,
                     modulation_strength, g_foc_sine_lut);
      W_Assert_SVPWM_(CCR1, CCR2, CCR3);
    }
  }

  __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
  Motor_PWM_Off();
}

void W_SVPWM_Start_Efficient_Ramp(int electrical_frequency_hz, int duration_ms,
                             int SVPWM_update_frequency_hz,
                             float modulation_strength)
{
  uint32_t start_tick = HAL_GetTick();
  uint32_t elapsed_ms = 0U;
  float electrical_angle_rad = 0.0f;
  float electrical_velocity_rad_s = 0.0f;
  float electrical_velocity_rad_s_max = TWO_PI_F * (float)electrical_frequency_hz;
  float dt = 1.0f / (float)SVPWM_update_frequency_hz;
  int ramp_complete = 0;

  FOC_SineLUTInit();
  HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
  Motor_PWM_Enable();
  svpwm_update_due = 0U;
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC4);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);

  while (elapsed_ms < (uint32_t)duration_ms)
  {
    elapsed_ms = HAL_GetTick() - start_tick;

    if (HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin) ==
        GPIO_PIN_RESET)
    {
      __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
      Motor_ReportDRVFaultAndShutdown("W_SVPWM run");
      return;
    }

    if (svpwm_update_due)
    {
      svpwm_update_due = 0U;
      if(ramp_complete == 0)
      {
        electrical_velocity_rad_s += 0.0005f; // Ramp up the velocity
        if(electrical_velocity_rad_s >= electrical_velocity_rad_s_max)
        {
          electrical_velocity_rad_s = electrical_velocity_rad_s_max;
          ramp_complete = 1;
        }
      }

      electrical_angle_rad += electrical_velocity_rad_s * dt;
      electrical_angle_rad = WrapRadians(electrical_angle_rad);

      int CCR1 = 0;
      int CCR2 = 0;
      int CCR3 = 0;

      W_SVPWM_Update(&CCR1, &CCR2, &CCR3, electrical_angle_rad,
                     modulation_strength, g_foc_sine_lut);
      W_Assert_SVPWM_(CCR1, CCR2, CCR3);
    }
  }

  __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
  Motor_PWM_Off();
}

void W_SVPWM_Start_Efficient_Ramp_Counter(
    int electrical_frequency_hz, int duration_ms,
    int SVPWM_update_frequency_hz, float modulation_strength)
{
  uint32_t start_tick = HAL_GetTick();
  uint32_t elapsed_ms = 0U;
  uint32_t last_update_count = 0U;
  uint32_t backlogged_update_count = 0U;
  uint32_t max_updates_per_service = 0U;
  float electrical_angle_rad = 0.0f;
  float electrical_velocity_rad_s = 0.0f;
  float electrical_velocity_rad_s_max = TWO_PI_F * (float)electrical_frequency_hz;
  float dt = 1.0f / (float)SVPWM_update_frequency_hz;
  int ramp_complete = 0;

  FOC_SineLUTInit();
  HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
  __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
  Motor_PWM_Enable();
  svpwm_update_count = 0U;
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC4);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);

  while (elapsed_ms < (uint32_t)duration_ms)
  {
    elapsed_ms = HAL_GetTick() - start_tick;

    if (HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin) ==
        GPIO_PIN_RESET)
    {
      __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
      Motor_ReportDRVFaultAndShutdown("W_SVPWM run");
      return;
    }

    uint32_t current_update_count = svpwm_update_count;
    uint32_t elapsed_updates = current_update_count - last_update_count;

    if (elapsed_updates != 0U)
    {
      uint32_t updates_to_apply = elapsed_updates;
      last_update_count = current_update_count;

      if (elapsed_updates > max_updates_per_service)
      {
        max_updates_per_service = elapsed_updates;
      }
      if (elapsed_updates > 1U)
      {
        backlogged_update_count += elapsed_updates - 1U;
      }

      while (updates_to_apply > 0U)
      {
        if(ramp_complete == 0)
        {
          electrical_velocity_rad_s += 0.0005f; // Ramp up the velocity
          if(electrical_velocity_rad_s >= electrical_velocity_rad_s_max)
          {
            electrical_velocity_rad_s = electrical_velocity_rad_s_max;
            ramp_complete = 1;
          }
        }

        electrical_angle_rad += electrical_velocity_rad_s * dt;
        electrical_angle_rad = WrapRadians(electrical_angle_rad);
        updates_to_apply--;
      }

      int CCR1 = 0;
      int CCR2 = 0;
      int CCR3 = 0;

      W_SVPWM_Update(&CCR1, &CCR2, &CCR3, electrical_angle_rad,
                     modulation_strength, g_foc_sine_lut);
      W_Assert_SVPWM_(CCR1, CCR2, CCR3);
    }
  }

  __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
  Motor_PWM_Off();
#if UART_OUTPUT_ENABLE
  printf("W_SVPWM counter scheduler: backlogged updates=%lu, maximum updates per service=%lu\r\n",
         (unsigned long)backlogged_update_count,
         (unsigned long)max_updates_per_service);
#endif
}

void W_SVPWM_Start_Efficient_Copilot(int electrical_frequency_hz,
                                     int duration_ms,
                                     int SVPWM_update_frequency_hz,
                                     float modulation_strength)
{
  uint32_t start_tick;
  uint32_t elapsed_ms = 0U;
  uint32_t first_update_count;
  uint32_t last_update_count;
  float electrical_angle_rad = 0.0f;
  float previous_electrical_cycles = 0.0f;
  float target_electrical_frequency_hz = (float)electrical_frequency_hz;
  float start_electrical_frequency_hz =
      WILL_SVPWM_PROBE_START_ELECTRICAL_HZ;
  float start_modulation = WILL_SVPWM_PROBE_START_MODULATION;
  float dt;
  int CCR1 = 0;
  int CCR2 = 0;
  int CCR3 = 0;

  if (duration_ms <= 0 || SVPWM_update_frequency_hz <= 0)
  {
    Motor_PWM_Off();
    return;
  }

  if (target_electrical_frequency_hz < 0.0f)
  {
    start_electrical_frequency_hz = -start_electrical_frequency_hz;
    if (target_electrical_frequency_hz > start_electrical_frequency_hz)
    {
      start_electrical_frequency_hz = target_electrical_frequency_hz;
    }
  }
  else if (target_electrical_frequency_hz < start_electrical_frequency_hz)
  {
    start_electrical_frequency_hz = target_electrical_frequency_hz;
  }

  if (modulation_strength < start_modulation)
  {
    start_modulation = modulation_strength;
  }

  dt = 1.0f / (float)SVPWM_update_frequency_hz;

  FOC_SineLUTInit();
  W_SVPWM_Update(&CCR1, &CCR2, &CCR3, electrical_angle_rad,
                 start_modulation, g_foc_sine_lut);
  W_Assert_SVPWM_(CCR1, CCR2, CCR3);
  HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
  __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
  svpwm_update_count = 0U;
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC4);
  Motor_PWM_Enable();
  first_update_count = svpwm_update_count;
  last_update_count = first_update_count;
  start_tick = HAL_GetTick();
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);

  while (elapsed_ms < (uint32_t)duration_ms)
  {
    elapsed_ms = HAL_GetTick() - start_tick;

    if (HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin) ==
        GPIO_PIN_RESET)
    {
      __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
      Motor_ReportDRVFaultAndShutdown("W_SVPWM run");
      return;
    }

    uint32_t current_update_count = svpwm_update_count;

    if (current_update_count != last_update_count)
    {
      uint32_t profile_update_count =
          current_update_count - first_update_count;
      float total_elapsed_s = (float)profile_update_count * dt;
      float alignment_duration_s =
          (float)WILL_SVPWM_PROBE_ALIGNMENT_MS * 0.001f;
      float ramp_duration_s =
          (float)WILL_SVPWM_PROBE_RAMP_MS * 0.001f;
      float profile_elapsed_s = 0.0f;
      float ramp_progress = 0.0f;
      float electrical_cycles = 0.0f;
      float commanded_modulation = start_modulation;

      last_update_count = current_update_count;

      if (total_elapsed_s > alignment_duration_s)
      {
        profile_elapsed_s = total_elapsed_s - alignment_duration_s;
        ramp_progress = profile_elapsed_s / ramp_duration_s;
        if (ramp_progress > 1.0f)
        {
          ramp_progress = 1.0f;
        }

        commanded_modulation = start_modulation +
            ((modulation_strength - start_modulation) * ramp_progress);

        if (profile_elapsed_s < ramp_duration_s)
        {
          electrical_cycles =
              (start_electrical_frequency_hz * profile_elapsed_s) +
              (0.5f *
               (target_electrical_frequency_hz -
                start_electrical_frequency_hz) *
               profile_elapsed_s * ramp_progress);
        }
        else
        {
          electrical_cycles =
              (0.5f *
               (start_electrical_frequency_hz +
                target_electrical_frequency_hz) *
               ramp_duration_s) +
              (target_electrical_frequency_hz *
               (profile_elapsed_s - ramp_duration_s));
        }
      }

      electrical_angle_rad += TWO_PI_F *
          (electrical_cycles - previous_electrical_cycles);
      electrical_angle_rad = WrapRadians(electrical_angle_rad);
      previous_electrical_cycles = electrical_cycles;

      W_SVPWM_Update(&CCR1, &CCR2, &CCR3, electrical_angle_rad,
                     commanded_modulation, g_foc_sine_lut);
      W_Assert_SVPWM_(CCR1, CCR2, CCR3);
    }
  }

  __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
  Motor_PWM_Off();
}

void W_SVPWM_Update(int *CCR1, int *CCR2, int *CCR3,
                    float electrical_angle_rad, float modulation_strength,
                    const float *sine_lut)
{
  /* V1=(100), V2=(110), V3=(010), V4=(011), V5=(001), V6=(101). */
  if (electrical_angle_rad <= PI_F / 3.0f &&
      electrical_angle_rad >= 0.0f)
  {
    float t1_normalized = modulation_strength *
        SVPWM_LookupSin(sine_lut, PI_F / 3.0f - electrical_angle_rad);
    float t2_normalized = modulation_strength *
        SVPWM_LookupSin(sine_lut, electrical_angle_rad);
    float t0_normalized = 1.0f - t1_normalized - t2_normalized;
    *CCR1 = (t1_normalized + t2_normalized + t0_normalized / 2.0f) *
        FOC_TIM1_PERIOD_TICKS;
    *CCR2 = (t2_normalized + t0_normalized / 2.0f) *
        FOC_TIM1_PERIOD_TICKS;
    *CCR3 = (t0_normalized / 2.0f) * FOC_TIM1_PERIOD_TICKS;
  }
  else if (electrical_angle_rad <= 2.0f * PI_F / 3.0f &&
           electrical_angle_rad > PI_F / 3.0f)
  {
    float t1_normalized = modulation_strength *
        SVPWM_LookupSin(sine_lut,
                        2.0f * PI_F / 3.0f - electrical_angle_rad);
    float t2_normalized = modulation_strength *
        SVPWM_LookupSin(sine_lut, electrical_angle_rad - PI_F / 3.0f);
    float t0_normalized = 1.0f - t1_normalized - t2_normalized;
    *CCR1 = (t1_normalized + t0_normalized / 2.0f) *
        FOC_TIM1_PERIOD_TICKS;
    *CCR2 = (t1_normalized + t2_normalized + t0_normalized / 2.0f) *
        FOC_TIM1_PERIOD_TICKS;
    *CCR3 = (t0_normalized / 2.0f) * FOC_TIM1_PERIOD_TICKS;
  }
  else if (electrical_angle_rad <= PI_F &&
           electrical_angle_rad > 2.0f * PI_F / 3.0f)
  {
    float t1_normalized = modulation_strength *
        SVPWM_LookupSin(sine_lut, PI_F - electrical_angle_rad);
    float t2_normalized = modulation_strength *
        SVPWM_LookupSin(sine_lut,
                        electrical_angle_rad - 2.0f * PI_F / 3.0f);
    float t0_normalized = 1.0f - t1_normalized - t2_normalized;
    *CCR1 = (t0_normalized / 2.0f) * FOC_TIM1_PERIOD_TICKS;
    *CCR2 = (t1_normalized + t2_normalized + t0_normalized / 2.0f) *
        FOC_TIM1_PERIOD_TICKS;
    *CCR3 = (t2_normalized + t0_normalized / 2.0f) *
        FOC_TIM1_PERIOD_TICKS;
  }
  else if (electrical_angle_rad <= 4.0f * PI_F / 3.0f &&
           electrical_angle_rad > PI_F)
  {
    float t1_normalized = modulation_strength *
        SVPWM_LookupSin(sine_lut,
                        4.0f * PI_F / 3.0f - electrical_angle_rad);
    float t2_normalized = modulation_strength *
        SVPWM_LookupSin(sine_lut, electrical_angle_rad - PI_F);
    float t0_normalized = 1.0f - t1_normalized - t2_normalized;
    *CCR1 = (t0_normalized / 2.0f) * FOC_TIM1_PERIOD_TICKS;
    *CCR2 = (t1_normalized + t0_normalized / 2.0f) *
        FOC_TIM1_PERIOD_TICKS;
    *CCR3 = (t1_normalized + t2_normalized + t0_normalized / 2.0f) *
        FOC_TIM1_PERIOD_TICKS;
  }
  else if (electrical_angle_rad <= 5.0f * PI_F / 3.0f &&
           electrical_angle_rad > 4.0f * PI_F / 3.0f)
  {
    float t1_normalized = modulation_strength *
        SVPWM_LookupSin(sine_lut,
                        5.0f * PI_F / 3.0f - electrical_angle_rad);
    float t2_normalized = modulation_strength *
        SVPWM_LookupSin(sine_lut,
                        electrical_angle_rad - 4.0f * PI_F / 3.0f);
    float t0_normalized = 1.0f - t1_normalized - t2_normalized;
    *CCR1 = (t2_normalized + t0_normalized / 2.0f) *
        FOC_TIM1_PERIOD_TICKS;
    *CCR2 = (t0_normalized / 2.0f) * FOC_TIM1_PERIOD_TICKS;
    *CCR3 = (t1_normalized + t2_normalized + t0_normalized / 2.0f) *
        FOC_TIM1_PERIOD_TICKS;
  }
  else if (electrical_angle_rad <= TWO_PI_F &&
           electrical_angle_rad > 5.0f * PI_F / 3.0f)
  {
    float t1_normalized = modulation_strength *
        SVPWM_LookupSin(sine_lut, TWO_PI_F - electrical_angle_rad);
    float t2_normalized = modulation_strength *
        SVPWM_LookupSin(sine_lut,
                        electrical_angle_rad - 5.0f * PI_F / 3.0f);
    float t0_normalized = 1.0f - t1_normalized - t2_normalized;
    *CCR1 = (t1_normalized + t2_normalized + t0_normalized / 2.0f) *
        FOC_TIM1_PERIOD_TICKS;
    *CCR2 = (t0_normalized / 2.0f) * FOC_TIM1_PERIOD_TICKS;
    *CCR3 = (t1_normalized + t0_normalized / 2.0f) *
        FOC_TIM1_PERIOD_TICKS;
  }
}

void Will_SVPWM_Implementation(int frequency_hz, int duration_ms)
{
  __HAL_TIM_DISABLE(&htim1);
  __HAL_TIM_SET_AUTORELOAD(&htim1, FOC_TIM1_PERIOD_TICKS);
  TIM1->CR1 = (TIM1->CR1 & ~(TIM_CR1_CMS | TIM_CR1_DIR)) |
      TIM_COUNTERMODE_CENTERALIGNED1;
  TIM1->BDTR = (TIM1->BDTR & ~TIM_BDTR_DTG) | g_motor_deadtime_ticks;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;

  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  TIM1->EGR = TIM_EGR_UG;
  __HAL_TIM_ENABLE(&htim1);

  int SVPWM_update_frequency_hz = (int)Get_PWM_FrequencyHz();
  float modulation_strength = WILL_SVPWM_PROBE_MODULATION_STRENGTH;
  Start_PWM();
  // W_SVPWM_Start_Efficient_Copilot(frequency_hz, duration_ms,
  //                                 SVPWM_update_frequency_hz,
  //                                 modulation_strength);
  // W_SVPWM_Start_Efficient_Ramp(frequency_hz, duration_ms,
  //                              SVPWM_update_frequency_hz,
  //                              modulation_strength);
  W_SVPWM_Start_Efficient_Ramp_Counter(frequency_hz, duration_ms,
                                       SVPWM_update_frequency_hz,
                                       modulation_strength);
}

void W_Assert_SVPWM_(int CCR1, int CCR2, int CCR3)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, CCR1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, CCR2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, CCR3);
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1 &&
      htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
    svpwm_update_due = 1U;
    svpwm_update_count++;
  }
}
#endif /* WILL_SVPWM_EXPERIMENT_ENABLE */

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

#if WILL_SVPWM_EXPERIMENT_ENABLE
static inline float SVPWM_LookupSin(const float *sine_lut, float radians)
{
  float position;
  float fraction;
  uint32_t index;

  if (radians >= TWO_PI_F)
  {
    radians -= TWO_PI_F;
  }
  else if (radians < 0.0f)
  {
    radians += TWO_PI_F;
  }

  position = radians * ((float)FOC_SINE_LUT_SIZE / TWO_PI_F);
  index = (uint32_t)position;
  fraction = position - (float)index;

  return sine_lut[index] +
      (fraction * (sine_lut[index + 1U] - sine_lut[index]));
}
#endif

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
  sample->time_ms = elapsed_ms;
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
  printf("time_ms,raw_a,raw_b,raw_c,ia_mA,ib_control_mA,ib_sensed_raw_mA,ib_sensed_corrected_mA,ic_mA,reconstructed_sum_mA,sensed_sum_mA,valid_a,valid_b,valid_c,id_mA,iq_mA,electrical_angle_mrad,electrical_velocity_mrad_s,motor_rpm_filtered_x100,motor_rpm_window_x100,motor_speed_reference_rpm_x100,motor_speed_error_rpm_x100,motor_position_mdeg,output_position_mdeg,output_trajectory_position_mdeg,output_trajectory_error_mdeg,output_position_target_mdeg,encoder_window_counts,encoder_innovation_mrad,encoder_correction_mrad,encoder_age_us,id_reference_mA,iq_reference_mA,outer_integrator_mA,breakaway_current_mA,iq_saturated,vd_x10000,vq_x10000,id_average_mA,iq_average_mA,phase_rms_mA,phase_peak_mA,current_samples,current_isr_max_cycles,motor_torque_reference_mNm,motor_torque_estimate_mNm,ideal_output_torque_reference_mNm,ideal_output_torque_estimate_mNm\r\n");
  for (i = 0U; i < g_foc_log_count; ++i)
  {
    const FOC_LogSample *sample = &g_foc_log[i];

    printf("%lu,%u,%u,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%u,%u,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%lu,%ld,%ld,%ld,%ld,%u,%ld,%ld,%ld,%ld,%ld,%ld,%lu,%lu,%ld,%ld,%ld,%ld\r\n",
           (unsigned long)sample->time_ms,
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
           (unsigned long)sample->current_isr_max_cycles,
           (long)((float)sample->iq_reference_ma *
                  FOC_MOTOR_ESTIMATED_KT_NM_PER_A),
           (long)((float)sample->iq_average_ma *
                  FOC_MOTOR_ESTIMATED_KT_NM_PER_A),
           (long)((float)sample->iq_reference_ma *
                  FOC_MOTOR_ESTIMATED_KT_NM_PER_A *
                  FOC_MOTOR_TO_OUTPUT_GEAR_RATIO),
           (long)((float)sample->iq_average_ma *
                  FOC_MOTOR_ESTIMATED_KT_NM_PER_A *
                  FOC_MOTOR_TO_OUTPUT_GEAR_RATIO));
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
  uint32_t encoder_accepted_samples = 0U;
  uint32_t encoder_limited_corrections = 0U;
  uint32_t encoder_limited_velocity_corrections = 0U;
  uint32_t encoder_service_count = 0U;
  uint32_t encoder_max_service_interval_cycles = 0U;
  uint32_t encoder_consecutive_rejections = 0U;
  float encoder_max_rejected_innovation_rad = 0.0f;
  float encoder_max_accepted_innovation_rad = 0.0f;
  int32_t speed_window_counts = 0;
  uint8_t encoder_errors = 0U;
  float mechanical_rpm_filtered = 0.0f;
  float mechanical_rpm_window = 0.0f;
  float foc_iq_limit_a = FOC_IQ_TARGET_A;
#if FOC_POSITION_DEMO_ENABLE
#if FOC_IMPEDANCE_DEMO_ENABLE
  const uint32_t foc_test_duration_ms =
      FOC_IMPEDANCE_DEMO_DURATION_MS +
      FOC_IMPEDANCE_RELEASE_RAMP_MS +
      FOC_IMPEDANCE_SETTLE_TIMEOUT_MS;
  uint32_t foc_log_interval_ms = FOC_IMPEDANCE_LOG_INTERVAL_MS;
  uint32_t foc_min_current_samples_per_log =
      FOC_IMPEDANCE_MIN_CURRENT_SAMPLES_PER_LOG;
  float foc_overspeed_rpm = FOC_IMPEDANCE_OVERSPEED_MOTOR_RPM;
#elif FOC_FORCE_SCALE_TEST_ENABLE
  const uint32_t foc_test_duration_ms =
      FOC_POSITION_TEST_DURATION_MS +
      FOC_FORCE_TEST_TORQUE_DURATION_MS +
      FOC_FORCE_TEST_TORQUE_SETTLE_TIMEOUT_MS;
  uint32_t foc_log_interval_ms = FOC_POSITION_LOG_INTERVAL_MS;
  uint32_t foc_min_current_samples_per_log =
      FOC_POSITION_MIN_CURRENT_SAMPLES_PER_LOG;
  float foc_overspeed_rpm =
      FOC_FORCE_TEST_APPROACH_OVERSPEED_MOTOR_RPM;
#elif FOC_TORQUE_ONLY_DEMO_ENABLE
  const uint32_t foc_test_duration_ms =
      FOC_TORQUE_DEMO_DURATION_MS + FOC_TORQUE_SETTLE_TIMEOUT_MS;
  uint32_t foc_log_interval_ms = FOC_TORQUE_LOG_INTERVAL_MS;
  uint32_t foc_min_current_samples_per_log =
      FOC_TORQUE_MIN_CURRENT_SAMPLES_PER_LOG;
  float foc_overspeed_rpm = FOC_TORQUE_OVERSPEED_MOTOR_RPM;
#elif FOC_VELOCITY_HEAT_TEST_ENABLE
  const uint32_t foc_test_duration_ms =
      FOC_HEAT_TEST_DURATION_MS +
      FOC_HEAT_TEST_SETTLE_TIMEOUT_MS +
      FOC_HEAT_TEST_TIMING_MARGIN_MS;
  uint32_t foc_log_interval_ms = FOC_VELOCITY_LOG_INTERVAL_MS;
  uint32_t foc_min_current_samples_per_log =
      FOC_VELOCITY_MIN_CURRENT_SAMPLES_PER_LOG;
  float foc_overspeed_rpm = FOC_HEAT_TEST_OVERSPEED_MOTOR_RPM;
#else
#if FOC_COMPOSITE_DEMO_ENABLE
  const uint32_t foc_test_duration_ms =
      FOC_COMPOSITE_IMPEDANCE_DURATION_MS +
      FOC_POSITION_TEST_DURATION_MS +
      FOC_VELOCITY_DEMO_DURATION_MS +
      FOC_VELOCITY_SETTLE_TIMEOUT_MS +
      FOC_COMPOSITE_TIMING_MARGIN_MS;
  uint32_t foc_log_interval_ms = FOC_IMPEDANCE_LOG_INTERVAL_MS;
  uint32_t foc_min_current_samples_per_log =
      FOC_IMPEDANCE_MIN_CURRENT_SAMPLES_PER_LOG;
  float foc_overspeed_rpm = FOC_IMPEDANCE_OVERSPEED_MOTOR_RPM;
#elif FOC_PRE_POSITION_IMPEDANCE_ENABLE
  const uint32_t foc_test_duration_ms =
      FOC_COMPOSITE_IMPEDANCE_DURATION_MS +
      FOC_POSITION_TEST_DURATION_MS +
      FOC_VELOCITY_DEMO_DURATION_MS +
      FOC_VELOCITY_SETTLE_TIMEOUT_MS +
      FOC_TORQUE_DEMO_DURATION_MS +
      FOC_TORQUE_SETTLE_TIMEOUT_MS;
  uint32_t foc_log_interval_ms = FOC_IMPEDANCE_LOG_INTERVAL_MS;
  uint32_t foc_min_current_samples_per_log =
      FOC_IMPEDANCE_MIN_CURRENT_SAMPLES_PER_LOG;
  float foc_overspeed_rpm = FOC_IMPEDANCE_OVERSPEED_MOTOR_RPM;
#else
  const uint32_t foc_test_duration_ms =
      FOC_POSITION_TEST_DURATION_MS +
      FOC_VELOCITY_DEMO_DURATION_MS +
      FOC_VELOCITY_SETTLE_TIMEOUT_MS +
      FOC_TORQUE_DEMO_DURATION_MS +
      FOC_TORQUE_SETTLE_TIMEOUT_MS;
  uint32_t foc_log_interval_ms = FOC_POSITION_LOG_INTERVAL_MS;
  uint32_t foc_min_current_samples_per_log =
      FOC_POSITION_MIN_CURRENT_SAMPLES_PER_LOG;
  float foc_overspeed_rpm = FOC_POSITION_OVERSPEED_RPM;
#endif
#endif
#if FOC_IMPEDANCE_DEMO_ENABLE
  uint32_t impedance_control_updates = 0U;
  uint32_t impedance_iq_saturated_updates = 0U;
  float impedance_max_abs_position_error_deg = 0.0f;
  float impedance_peak_abs_iq_a = 0.0f;
  uint8_t impedance_demo_completed = 0U;
#else
#if FOC_COMPOSITE_DEMO_ENABLE || FOC_PRE_POSITION_IMPEDANCE_ENABLE
  uint32_t impedance_control_updates = 0U;
  uint32_t impedance_iq_saturated_updates = 0U;
  float impedance_max_abs_position_error_deg = 0.0f;
  float impedance_peak_abs_iq_a = 0.0f;
  uint8_t impedance_demo_completed = 0U;
  uint32_t impedance_stage_start_tick = 0U;
  uint32_t position_stage_start_tick = 0U;
#endif
  uint32_t position_hold_start_tick = 0U;
  uint32_t position_step_start_tick = 0U;
  uint32_t velocity_stage_start_tick = 0U;
  uint32_t velocity_stage_start_elapsed_ms = 0U;
  uint32_t torque_stage_start_tick = 0U;
#if !FOC_COMPOSITE_DEMO_ENABLE && !FOC_VELOCITY_HEAT_TEST_ENABLE
  uint32_t torque_stage_start_elapsed_ms = 0U;
#endif
  uint32_t position_control_updates = 0U;
  uint32_t position_boost_updates = 0U;
  uint32_t position_iq_saturated_updates = 0U;
  uint32_t velocity_control_updates = 0U;
  uint32_t velocity_iq_saturated_updates = 0U;
  uint32_t torque_control_updates = 0U;
  float position_max_abs_tracking_error_deg = 0.0f;
  float position_max_abs_boost_a = 0.0f;
  float position_demo_final_output_deg = 0.0f;
  float position_demo_final_motor_deg = 0.0f;
  float output_trajectory_speed_rpm = 0.0f;
  float position_integrator_a = 0.0f;
  float velocity_output_reference_rpm = 0.0f;
  float velocity_integrator_a = 0.0f;
  float velocity_final_output_rpm = 0.0f;
  float velocity_peak_abs_output_rpm = 0.0f;
  float velocity_max_abs_error_output_rpm = 0.0f;
  float torque_motor_reference_nm = 0.0f;
  float torque_peak_abs_iq_a = 0.0f;
  float torque_peak_abs_motor_rpm = 0.0f;
  uint8_t position_target_index = 0U;
#if FOC_FORCE_SCALE_TEST_ENABLE
  uint32_t force_contact_candidate_start_tick = 0U;
  uint32_t force_contact_elapsed_ms = 0U;
  float force_contact_output_deg = 0.0f;
  uint8_t force_contact_detected = 0U;
  uint8_t force_torque_stage_requested = 0U;
#endif
  uint8_t position_demo_completed =
      (FOC_TORQUE_ONLY_DEMO_ENABLE || FOC_IMPEDANCE_DEMO_ENABLE ||
       FOC_VELOCITY_HEAT_TEST_ENABLE) ? 1U : 0U;
  uint8_t velocity_demo_started =
      FOC_VELOCITY_HEAT_TEST_ENABLE ? 1U : 0U;
  uint8_t velocity_demo_completed =
      (FOC_FORCE_SCALE_TEST_ENABLE || FOC_TORQUE_ONLY_DEMO_ENABLE ||
       FOC_IMPEDANCE_DEMO_ENABLE) ? 1U : 0U;
#if !FOC_COMPOSITE_DEMO_ENABLE && !FOC_VELOCITY_HEAT_TEST_ENABLE
  uint8_t torque_demo_started =
      FOC_TORQUE_ONLY_DEMO_ENABLE ? 1U : 0U;
#endif
  uint8_t torque_demo_completed =
      (FOC_COMPOSITE_DEMO_ENABLE || FOC_VELOCITY_HEAT_TEST_ENABLE) ? 1U : 0U;
  const uint8_t position_target_count =
      (uint8_t)(sizeof(g_foc_output_position_demo_targets_deg) /
                sizeof(g_foc_output_position_demo_targets_deg[0]));
#endif
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

#if FOC_POSITION_DEMO_ENABLE && \
    (FOC_TORQUE_ONLY_DEMO_ENABLE || FOC_IMPEDANCE_DEMO_ENABLE || \
     FOC_PRE_POSITION_IMPEDANCE_ENABLE || FOC_FORCE_SCALE_TEST_ENABLE || \
     FOC_COMPOSITE_DEMO_ENABLE || \
     FOC_VELOCITY_HEAT_TEST_ENABLE)
  /* Alignment requires an energized, freely moving shaft.  Remove all gate
   * drive before giving the operator time to attach the torque load. */
  Motor_PWM_Off();
  Set_StatorField_SVPWM(0.0f, 0.0f);
#endif

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
#if FOC_IMPEDANCE_DEMO_ENABLE || FOC_PRE_POSITION_IMPEDANCE_ENABLE || \
    FOC_COMPOSITE_DEMO_ENABLE
  g_foc_output_position_target_deg = FOC_IMPEDANCE_TARGET_OUTPUT_DEG;
  g_foc_output_trajectory_position_deg = FOC_IMPEDANCE_TARGET_OUTPUT_DEG;
  foc_iq_limit_a = FOC_IMPEDANCE_IQ_LIMIT_A;
  g_foc_active_hard_current_limit_a =
      FOC_IMPEDANCE_HARD_CURRENT_LIMIT_A;
  g_foc_active_dq_fault_limit_a = FOC_IMPEDANCE_DQ_FAULT_LIMIT_A;
#elif FOC_FORCE_SCALE_TEST_ENABLE
  g_foc_output_position_target_deg =
      g_foc_output_position_demo_targets_deg[0];
  foc_iq_limit_a = FOC_FORCE_TEST_APPROACH_IQ_LIMIT_A;
  g_foc_active_hard_current_limit_a =
      FOC_FORCE_TEST_APPROACH_HARD_CURRENT_LIMIT_A;
  g_foc_active_dq_fault_limit_a =
      FOC_FORCE_TEST_APPROACH_DQ_FAULT_LIMIT_A;
#else
  g_foc_output_position_target_deg =
      g_foc_output_position_demo_targets_deg[0];
  foc_iq_limit_a = FOC_POSITION_IQ_LIMIT_A;
  g_foc_active_hard_current_limit_a = FOC_POSITION_HARD_CURRENT_LIMIT_A;
  g_foc_active_dq_fault_limit_a = FOC_POSITION_DQ_FAULT_LIMIT_A;
#endif
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
#if FOC_IMPEDANCE_DEMO_ENABLE
  printf("FOC fixed-angle single-stiffness impedance demo; position sequence, velocity, torque sequence, integral, and disturbance boost are disabled\r\n");
  printf("FOC impedance target=%ld mdeg output; stiffness=%ld mNm/output-deg for %lu ms, then %lu ms smooth release\r\n",
         (long)(FOC_IMPEDANCE_TARGET_OUTPUT_DEG * 1000.0f),
         (long)(FOC_IMPEDANCE_STIFFNESS_NM_PER_OUTPUT_DEG * 1000.0f),
         (unsigned long)FOC_IMPEDANCE_DEMO_DURATION_MS,
         (unsigned long)FOC_IMPEDANCE_RELEASE_RAMP_MS);
  printf("FOC impedance current gain=%ld mA/output-deg, damping=%ld mA/output-rpm, Iq clamp=%ld mA\r\n",
         (long)((FOC_IMPEDANCE_STIFFNESS_NM_PER_OUTPUT_DEG /
                 (FOC_MOTOR_ESTIMATED_KT_NM_PER_A *
                  FOC_MOTOR_TO_OUTPUT_GEAR_RATIO)) * 1000.0f),
         (long)(FOC_IMPEDANCE_DAMPING_A_PER_OUTPUT_RPM * 1000.0f),
         (long)(foc_iq_limit_a * 1000.0f));
  printf("FOC impedance schedule: one stiffness level from 0-%lu ms, release ramp to %lu ms; motor overspeed=%ld rpm\r\n",
         (unsigned long)FOC_IMPEDANCE_DEMO_DURATION_MS,
         (unsigned long)(FOC_IMPEDANCE_DEMO_DURATION_MS +
                         FOC_IMPEDANCE_RELEASE_RAMP_MS),
         (long)foc_overspeed_rpm);
  printf("FOC impedance protections: abnormal d/q current=%ld mA, hard phase current=%ld mA\r\n",
         (long)(g_foc_active_dq_fault_limit_a * 1000.0f),
         (long)(g_foc_active_hard_current_limit_a * 1000.0f));
  printf("FOC post-alignment arming pause=%lu seconds with PWM inhibited; use a controlled force gauge, not hands\r\n",
         (unsigned long)FOC_IMPEDANCE_ARMING_PAUSE_SEC);
#elif FOC_FORCE_SCALE_TEST_ENABLE
  printf("FOC dedicated force-scale test: bounded CCW approach, inferred contact, then one-direction Iq ramp\r\n");
  printf("FOC force-scale direction sign=%ld/1000 for CCW from the motor front; fallback travel target=%ld mdeg\r\n",
         (long)(FOC_FORCE_TEST_CCW_SIGN * 1000.0f),
         (long)(FOC_FORCE_TEST_CCW_SIGN *
                FOC_FORCE_TEST_CONTACT_OUTPUT_DEG * 1000.0f));
  printf("FOC approach: max=%ld mRPM output, accel/decel=%ld mRPM/s, Iq clamp=%ld mA; contact requires >=%ld mdeg travel, <=%ld mRPM, >=%ld mA for %lu ms\r\n",
         (long)(FOC_FORCE_TEST_APPROACH_MAX_OUTPUT_RPM * 1000.0f),
         (long)(FOC_FORCE_TEST_APPROACH_ACCEL_OUTPUT_RPM_S * 1000.0f),
         (long)(FOC_FORCE_TEST_APPROACH_IQ_LIMIT_A * 1000.0f),
         (long)(FOC_FORCE_TEST_CONTACT_MIN_TRAVEL_DEG * 1000.0f),
         (long)(FOC_FORCE_TEST_CONTACT_MAX_OUTPUT_RPM * 1000.0f),
         (long)(FOC_FORCE_TEST_CONTACT_MIN_IQ_A * 1000.0f),
         (unsigned long)FOC_FORCE_TEST_CONTACT_CONFIRM_MS);
  printf("FOC scale load: target Iq=%ld mA, ramp=%ld mA/s, hold=%lu ms, release by %lu ms, torque overspeed=%ld rpm\r\n",
         (long)(FOC_FORCE_TEST_TARGET_IQ_A * 1000.0f),
         (long)(FOC_FORCE_TEST_IQ_RAMP_A_PER_S * 1000.0f),
         (unsigned long)FOC_FORCE_TEST_TORQUE_HOLD_MS,
         (unsigned long)FOC_FORCE_TEST_TORQUE_DURATION_MS,
         (long)FOC_FORCE_TEST_TORQUE_OVERSPEED_MOTOR_RPM);
  printf("FOC ideal force estimate: output torque=%ld mNm at target Iq; lever-arm assumption=%ld mm (update macro before interpreting scale error)\r\n",
         (long)(FOC_FORCE_TEST_TARGET_IQ_A *
                FOC_MOTOR_ESTIMATED_KT_NM_PER_A *
                FOC_MOTOR_TO_OUTPUT_GEAR_RATIO * 1000.0f),
         (long)FOC_FORCE_TEST_LEVER_ARM_MM);
  printf("FOC ideal scale load at assumed lever arm=%ld mN (%ld gram-force), before gearbox and motor losses\r\n",
         (long)((FOC_FORCE_TEST_TARGET_IQ_A *
                 FOC_MOTOR_ESTIMATED_KT_NM_PER_A *
                 FOC_MOTOR_TO_OUTPUT_GEAR_RATIO * 1000000.0f) /
                FOC_FORCE_TEST_LEVER_ARM_MM),
         (long)((FOC_FORCE_TEST_TARGET_IQ_A *
                 FOC_MOTOR_ESTIMATED_KT_NM_PER_A *
                 FOC_MOTOR_TO_OUTPUT_GEAR_RATIO * 1000000.0f) /
                (FOC_FORCE_TEST_LEVER_ARM_MM * 9.80665f)));
  printf("FOC force-scale protections: approach phase/dq=%ld/%ld mA, load phase/dq=%ld/%ld mA, motor overspeed=%ld rpm, DRV nFAULT active\r\n",
         (long)(FOC_FORCE_TEST_APPROACH_HARD_CURRENT_LIMIT_A * 1000.0f),
         (long)(FOC_FORCE_TEST_APPROACH_DQ_FAULT_LIMIT_A * 1000.0f),
         (long)(FOC_FORCE_TEST_HARD_CURRENT_LIMIT_A * 1000.0f),
         (long)(FOC_FORCE_TEST_DQ_FAULT_LIMIT_A * 1000.0f),
         (long)FOC_FORCE_TEST_TORQUE_OVERSPEED_MOTOR_RPM);
  printf("FOC post-alignment arming pause=%lu seconds with PWM inhibited; keep hands clear of the bar\r\n",
         (unsigned long)FOC_FORCE_TEST_ARMING_PAUSE_SEC);
#elif FOC_TORQUE_ONLY_DEMO_ENABLE
  printf("FOC Kt-based torque-control-only demo; position and velocity stages are disabled\r\n");
  printf("FOC motor command: 0 -> +%ld -> 0 -> -%ld -> 0 mNm over %lu ms, ramp=%ld mNm/s\r\n",
         (long)(FOC_TORQUE_TARGET_MOTOR_NM * 1000.0f),
         (long)(FOC_TORQUE_TARGET_MOTOR_NM * 1000.0f),
         (unsigned long)FOC_TORQUE_DEMO_DURATION_MS,
         (long)(FOC_TORQUE_RAMP_MOTOR_NM_PER_S * 1000.0f));
  printf("FOC torque conversion: Kt=%ld uNm/A, target Iq=%ld mA, Id target=0 mA, ideal 11:1 output target=%ld mNm before losses\r\n",
         (long)(FOC_MOTOR_ESTIMATED_KT_NM_PER_A * 1000000.0f),
         (long)((FOC_TORQUE_TARGET_MOTOR_NM /
                 FOC_MOTOR_ESTIMATED_KT_NM_PER_A) * 1000.0f),
         (long)(FOC_TORQUE_TARGET_MOTOR_NM *
                FOC_MOTOR_TO_OUTPUT_GEAR_RATIO * 1000.0f));
  printf("FOC torque protections: Iq clamp=%ld mA, hard phase current=%ld mA, abnormal d/q current=%ld mA, motor overspeed=%ld rpm; load fixture required\r\n",
         (long)(foc_iq_limit_a * 1000.0f),
         (long)(g_foc_active_hard_current_limit_a * 1000.0f),
         (long)(g_foc_active_dq_fault_limit_a * 1000.0f),
         (long)foc_overspeed_rpm);
  printf("FOC post-alignment loading pause=%lu seconds with PWM inhibited\r\n",
         (unsigned long)FOC_TORQUE_LOAD_PAUSE_SEC);
#elif FOC_VELOCITY_HEAT_TEST_ENABLE
  printf("FOC velocity-only thermal test; position, impedance, torque, SVPWM, and six-step stages are disabled\r\n");
  printf("FOC thermal profile: ramp to %ld motor rpm (%ld mRPM output), hold approximately %lu seconds, then ramp to zero\r\n",
         (long)FOC_HEAT_TEST_TARGET_MOTOR_RPM,
         (long)(FOC_HEAT_TEST_TARGET_OUTPUT_RPM * 1000.0f),
         (unsigned long)(FOC_HEAT_TEST_HOLD_MS / 1000U));
  printf("FOC thermal timing: deceleration begins at %lu ms, test duration=%lu ms (+%lu ms settle); accel/decel=%ld/%ld mRPM/s\r\n",
         (unsigned long)FOC_HEAT_TEST_DECEL_START_MS,
         (unsigned long)FOC_HEAT_TEST_DURATION_MS,
         (unsigned long)FOC_HEAT_TEST_SETTLE_TIMEOUT_MS,
         (long)(FOC_HEAT_TEST_ACCEL_OUTPUT_RPM_S * 1000.0f),
         (long)(FOC_HEAT_TEST_DECEL_OUTPUT_RPM_S * 1000.0f));
  printf("FOC thermal protections: Iq clamp=%ld mA, hard phase current=%ld mA, abnormal d/q current=%ld mA, overspeed=%ld motor rpm\r\n",
         (long)(foc_iq_limit_a * 1000.0f),
         (long)(g_foc_active_hard_current_limit_a * 1000.0f),
         (long)(g_foc_active_dq_fault_limit_a * 1000.0f),
         (long)foc_overspeed_rpm);
  printf("FOC post-alignment safety pause=%lu seconds with PWM inhibited; keep clear of the rotating shaft and measure without contact\r\n",
         (unsigned long)FOC_HEAT_TEST_ARMING_PAUSE_SEC);
  printf("STATUS_2 is ON during the target-speed measurement window (%lu-%lu ms)\r\n",
         (unsigned long)FOC_HEAT_TEST_MEASUREMENT_START_MS,
         (unsigned long)FOC_HEAT_TEST_DECEL_START_MS);
#else
#if FOC_COMPOSITE_DEMO_ENABLE || FOC_PRE_POSITION_IMPEDANCE_ENABLE
#if FOC_COMPOSITE_DEMO_ENABLE
  printf("FOC composite stages 1-3: fixed-position impedance, profiled position PID, then velocity PI\r\n");
#else
  printf("FOC sequence: fixed-position impedance, profiled position PID, velocity PI, then Kt torque\r\n");
#endif
  printf("FOC impedance holds %ld mdeg output for %lu ms; stiffness=%ld mNm/output-deg, damping=%ld mA/output-rpm, Iq clamp=%ld mA\r\n",
         (long)(FOC_IMPEDANCE_TARGET_OUTPUT_DEG * 1000.0f),
         (unsigned long)FOC_COMPOSITE_IMPEDANCE_DURATION_MS,
         (long)(FOC_IMPEDANCE_STIFFNESS_NM_PER_OUTPUT_DEG * 1000.0f),
         (long)(FOC_IMPEDANCE_DAMPING_A_PER_OUTPUT_RPM * 1000.0f),
         (long)(FOC_IMPEDANCE_IQ_LIMIT_A * 1000.0f));
#endif
  printf("FOC profiled output-position-to-Iq PID demo: %u targets, disturbance integral enabled\r\n",
         (unsigned int)position_target_count);
  printf("FOC gearbox=%ld/1000 motor rev/output rev; output 360 deg requires %ld/1000 motor turns\r\n",
         (long)(FOC_MOTOR_TO_OUTPUT_GEAR_RATIO * 1000.0f),
         (long)(FOC_MOTOR_TO_OUTPUT_GEAR_RATIO * 1000.0f));
  printf("FOC output position is inferred from the motor encoder; gearbox backlash is not measured\r\n");
  printf("FOC output targets relative to aligned zero: 0,45,90,135,180,225,270,315,360,315,270,225,180,135,90,45,0 deg\r\n");
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
#if FOC_COMPOSITE_DEMO_ENABLE
  printf("FOC stages: impedance=%lu ms, position allowance=%lu ms, velocity=%lu ms (+%lu settle); overall safety window=%lu ms\r\n",
         (unsigned long)FOC_COMPOSITE_IMPEDANCE_DURATION_MS,
         (unsigned long)FOC_POSITION_TEST_DURATION_MS,
         (unsigned long)FOC_VELOCITY_DEMO_DURATION_MS,
         (unsigned long)FOC_VELOCITY_SETTLE_TIMEOUT_MS,
         (unsigned long)foc_test_duration_ms);
  printf("FOC CSV intervals: impedance=%lu ms, position=%lu ms, velocity=%lu ms\r\n",
         (unsigned long)FOC_IMPEDANCE_LOG_INTERVAL_MS,
         (unsigned long)FOC_POSITION_LOG_INTERVAL_MS,
         (unsigned long)FOC_VELOCITY_LOG_INTERVAL_MS);
#elif FOC_PRE_POSITION_IMPEDANCE_ENABLE
  printf("FOC stages: impedance=%lu ms, position allowance=%lu ms, velocity=%lu ms (+%lu settle), torque=%lu ms (+%lu settle); overall safety window=%lu ms\r\n",
         (unsigned long)FOC_COMPOSITE_IMPEDANCE_DURATION_MS,
         (unsigned long)FOC_POSITION_TEST_DURATION_MS,
         (unsigned long)FOC_VELOCITY_DEMO_DURATION_MS,
         (unsigned long)FOC_VELOCITY_SETTLE_TIMEOUT_MS,
         (unsigned long)FOC_TORQUE_DEMO_DURATION_MS,
         (unsigned long)FOC_TORQUE_SETTLE_TIMEOUT_MS,
         (unsigned long)foc_test_duration_ms);
  printf("FOC CSV intervals: impedance=%lu ms, position=%lu ms, velocity=%lu ms, torque=%lu ms\r\n",
         (unsigned long)FOC_IMPEDANCE_LOG_INTERVAL_MS,
         (unsigned long)FOC_POSITION_LOG_INTERVAL_MS,
         (unsigned long)FOC_VELOCITY_LOG_INTERVAL_MS,
         (unsigned long)FOC_TORQUE_LOG_INTERVAL_MS);
#else
  printf("FOC stages: position allowance=%lu ms, velocity=%lu ms (+%lu settle), torque=%lu ms (+%lu settle); overall safety window=%lu ms\r\n",
         (unsigned long)FOC_POSITION_TEST_DURATION_MS,
         (unsigned long)FOC_VELOCITY_DEMO_DURATION_MS,
         (unsigned long)FOC_VELOCITY_SETTLE_TIMEOUT_MS,
         (unsigned long)FOC_TORQUE_DEMO_DURATION_MS,
         (unsigned long)FOC_TORQUE_SETTLE_TIMEOUT_MS,
         (unsigned long)foc_test_duration_ms);
  printf("FOC CSV intervals: position=%lu ms, velocity=%lu ms, torque=%lu ms\r\n",
         (unsigned long)FOC_POSITION_LOG_INTERVAL_MS,
         (unsigned long)FOC_VELOCITY_LOG_INTERVAL_MS,
         (unsigned long)FOC_TORQUE_LOG_INTERVAL_MS);
#endif
  printf("FOC position step timeout=%lu ms\r\n",
         (unsigned long)FOC_POSITION_STEP_TIMEOUT_MS);
  printf("FOC output-frame PID: P=%ld/1000 A/output-deg, I=%ld/1000 A/(output-deg*s), D=%ld/1000 A/output-rpm; integral_limit=%ld mA, Iq_limit=%ld mA\r\n",
         (long)(FOC_POSITION_KP_A_PER_OUTPUT_DEG * 1000.0f),
         (long)(FOC_POSITION_KI_A_PER_OUTPUT_DEG_S * 1000.0f),
         (long)(FOC_POSITION_KD_A_PER_OUTPUT_RPM * 1000.0f),
         (long)(FOC_POSITION_INTEGRAL_LIMIT_A * 1000.0f),
         (long)(foc_iq_limit_a * 1000.0f));
  printf("FOC output tolerance=%ld mdeg, fixed per-target dwell=%lu ms; conditional anti-windup, reverse-error unwind=%ld/1000 x\r\n",
         (long)(FOC_OUTPUT_POSITION_TOLERANCE_DEG * 1000.0f),
         (unsigned long)FOC_POSITION_HOLD_MS,
         (long)(FOC_POSITION_INTEGRAL_UNWIND_MULTIPLIER * 1000.0f));
  printf("FOC inferred-disturbance boost: onset=%ld mdeg, full=%ld mdeg, low-speed fade=%ld mRPM, full velocity deficit=%ld mRPM, maximum=%ld mA\r\n",
         (long)(FOC_POSITION_DISTURBANCE_ONSET_OUTPUT_DEG * 1000.0f),
         (long)(FOC_POSITION_DISTURBANCE_FULL_OUTPUT_DEG * 1000.0f),
         (long)(FOC_POSITION_DISTURBANCE_FADE_OUTPUT_RPM * 1000.0f),
         (long)(FOC_POSITION_DISTURBANCE_FULL_SPEED_ERROR_RPM * 1000.0f),
         (long)(FOC_POSITION_DISTURBANCE_BOOST_A * 1000.0f));
  printf("FOC velocity stage: output target=%ld mRPM (motor=%ld rpm), accel/decel=%ld/%ld mRPM/s, decel begins at %lu ms\r\n",
         (long)(FOC_VELOCITY_TARGET_OUTPUT_RPM * 1000.0f),
         (long)(FOC_VELOCITY_TARGET_OUTPUT_RPM *
                FOC_MOTOR_TO_OUTPUT_GEAR_RATIO),
         (long)(FOC_VELOCITY_ACCEL_OUTPUT_RPM_S * 1000.0f),
         (long)(FOC_VELOCITY_DECEL_OUTPUT_RPM_S * 1000.0f),
         (unsigned long)FOC_VELOCITY_DECEL_START_MS);
  printf("FOC output-frame velocity PI: P=%ld/1000 A/output-rpm, I=%ld/1000 A/(output-rpm*s), integral_limit=%ld mA, Iq_limit=%ld mA, motor overspeed=%ld rpm\r\n",
         (long)(FOC_VELOCITY_KP_A_PER_OUTPUT_RPM * 1000.0f),
         (long)(FOC_VELOCITY_KI_A_PER_OUTPUT_RPM_S * 1000.0f),
         (long)(FOC_VELOCITY_INTEGRAL_LIMIT_A * 1000.0f),
         (long)(foc_iq_limit_a * 1000.0f),
         (long)FOC_VELOCITY_OVERSPEED_MOTOR_RPM);
#if !FOC_COMPOSITE_DEMO_ENABLE
  printf("FOC Kt torque stage: motor command 0 -> +%ld -> 0 -> -%ld -> 0 mNm over %lu ms, ramp=%ld mNm/s, motor overspeed=%ld rpm\r\n",
         (long)(FOC_TORQUE_TARGET_MOTOR_NM * 1000.0f),
         (long)(FOC_TORQUE_TARGET_MOTOR_NM * 1000.0f),
         (unsigned long)FOC_TORQUE_DEMO_DURATION_MS,
         (long)(FOC_TORQUE_RAMP_MOTOR_NM_PER_S * 1000.0f),
         (long)FOC_TORQUE_OVERSPEED_MOTOR_RPM);
  printf("FOC torque conversion: Kt=%ld uNm/A, target Iq=%ld mA, ideal gearbox-output target=%ld mNm before losses; Id target=0 mA; load fixture required\r\n",
         (long)(FOC_MOTOR_ESTIMATED_KT_NM_PER_A * 1000000.0f),
         (long)((FOC_TORQUE_TARGET_MOTOR_NM /
                 FOC_MOTOR_ESTIMATED_KT_NM_PER_A) * 1000.0f),
         (long)(FOC_TORQUE_TARGET_MOTOR_NM *
                FOC_MOTOR_TO_OUTPUT_GEAR_RATIO * 1000.0f));
#endif
#endif
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

#if FOC_POSITION_DEMO_ENABLE && \
    (FOC_TORQUE_ONLY_DEMO_ENABLE || FOC_IMPEDANCE_DEMO_ENABLE || \
     FOC_PRE_POSITION_IMPEDANCE_ENABLE || FOC_FORCE_SCALE_TEST_ENABLE || \
     FOC_COMPOSITE_DEMO_ENABLE || \
     FOC_VELOCITY_HEAT_TEST_ENABLE)
  {
    uint32_t remaining_seconds;
    uint32_t arming_pause_seconds;
    uint16_t torque_ready_angle;
    float torque_ready_electrical_angle_rad;

#if FOC_FORCE_SCALE_TEST_ENABLE
    arming_pause_seconds = FOC_FORCE_TEST_ARMING_PAUSE_SEC;
    printf("FOC alignment complete; PWM is OFF. Keep clear and verify the bar/scale fixture\r\n");
#elif FOC_IMPEDANCE_DEMO_ENABLE || FOC_PRE_POSITION_IMPEDANCE_ENABLE || \
    FOC_COMPOSITE_DEMO_ENABLE
    arming_pause_seconds = FOC_IMPEDANCE_ARMING_PAUSE_SEC;
    printf("FOC alignment complete; PWM is OFF. Keep clear and prepare the controlled-force test\r\n");
#elif FOC_VELOCITY_HEAT_TEST_ENABLE
    arming_pause_seconds = FOC_HEAT_TEST_ARMING_PAUSE_SEC;
    printf("FOC alignment complete; PWM is OFF. Step clear and prepare the non-contact temperature measurement\r\n");
#else
    arming_pause_seconds = FOC_TORQUE_LOAD_PAUSE_SEC;
    printf("FOC alignment complete; PWM is OFF. Secure the shaft/load fixture now\r\n");
#endif
    for (remaining_seconds = arming_pause_seconds;
         remaining_seconds > 0U;
         --remaining_seconds)
    {
#if FOC_FORCE_SCALE_TEST_ENABLE
      printf("FOC force-scale approach starts in %lu seconds\r\n",
             (unsigned long)remaining_seconds);
#elif FOC_IMPEDANCE_DEMO_ENABLE || FOC_PRE_POSITION_IMPEDANCE_ENABLE || \
    FOC_COMPOSITE_DEMO_ENABLE
      printf("FOC impedance control starts in %lu seconds\r\n",
             (unsigned long)remaining_seconds);
#elif FOC_VELOCITY_HEAT_TEST_ENABLE
      printf("FOC 5000 rpm thermal run starts in %lu seconds\r\n",
             (unsigned long)remaining_seconds);
#else
      printf("FOC torque starts in %lu seconds\r\n",
             (unsigned long)remaining_seconds);
#endif
      HAL_Delay(1000U);
    }

    /* The fixture may move the rotor after alignment. Refresh its angle so
     * the first q-axis command is still correctly oriented. */
    if (!AS5048A_ReadAngle(&torque_ready_angle))
    {
      g_foc_fault = 3U;
      printf("FOC startup failed: encoder read failed after loading pause\r\n");
      goto foc_stop;
    }
    previous_angle = torque_ready_angle;
    g_foc_mechanical_position_counts =
        AS5048A_SignedDelta(torque_ready_angle, angle_zero_final);
    torque_ready_electrical_angle_rad =
        FOC_ElectricalAngleFromEncoder(torque_ready_angle);
    g_foc_electrical_angle_rad = torque_ready_electrical_angle_rad;
    g_foc_encoder_observed_angle_rad = torque_ready_electrical_angle_rad;
    g_foc_electrical_velocity_rad_s = 0.0f;
    g_foc_encoder_observation_cycles = DWT->CYCCNT;
    g_foc_encoder_observation_sequence = 0U;
    g_foc_id_integrator = 0.0f;
    g_foc_iq_integrator = 0.0f;
    g_foc_id_reference_a = 0.0f;
    g_foc_iq_reference_a = 0.0f;
#if FOC_FORCE_SCALE_TEST_ENABLE
    printf("FOC arming pause complete; refreshed rotor angle=%ld mdeg. Starting bounded CCW scale approach\r\n",
#elif FOC_IMPEDANCE_DEMO_ENABLE || FOC_PRE_POSITION_IMPEDANCE_ENABLE || \
    FOC_COMPOSITE_DEMO_ENABLE
    printf("FOC arming pause complete; refreshed rotor angle=%ld mdeg. Starting fixed-angle impedance control\r\n",
#elif FOC_VELOCITY_HEAT_TEST_ENABLE
    printf("FOC safety pause complete; refreshed rotor angle=%ld mdeg. Starting velocity-only thermal run\r\n",
#else
    printf("FOC loading pause complete; refreshed rotor angle=%ld mdeg. Starting torque ramp\r\n",
#endif
           (long)(((uint32_t)torque_ready_angle * 360000UL) / 16384UL));
  }
#endif

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
#if FOC_IMPEDANCE_DEMO_ENABLE
  /* The impedance schedule is referenced directly to start_tick. */
#elif FOC_COMPOSITE_DEMO_ENABLE || FOC_PRE_POSITION_IMPEDANCE_ENABLE
  impedance_stage_start_tick = start_tick;
#elif FOC_TORQUE_ONLY_DEMO_ENABLE
  torque_stage_start_tick = start_tick;
  torque_stage_start_elapsed_ms = 0U;
  torque_demo_started = 1U;
#elif FOC_VELOCITY_HEAT_TEST_ENABLE
  velocity_stage_start_tick = start_tick;
  velocity_stage_start_elapsed_ms = 0U;
  velocity_demo_started = 1U;
#else
  position_step_start_tick = start_tick;
#endif
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
#if FOC_POSITION_DEMO_ENABLE && \
    (FOC_TORQUE_ONLY_DEMO_ENABLE || FOC_IMPEDANCE_DEMO_ENABLE || \
     FOC_PRE_POSITION_IMPEDANCE_ENABLE || FOC_FORCE_SCALE_TEST_ENABLE || \
     FOC_COMPOSITE_DEMO_ENABLE || \
     FOC_VELOCITY_HEAT_TEST_ENABLE)
  /* PWM was deliberately inhibited throughout the loading window.  Compare
   * registers hold a zero voltage vector until the current ISR takes over. */
  Motor_PWM_Enable();
#endif
  g_foc_enabled = 1U;

  while ((HAL_GetTick() - start_tick) < foc_test_duration_ms &&
         g_foc_enabled != 0U)
  {
    uint32_t now_tick = HAL_GetTick();
    uint32_t now_cycles = DWT->CYCCNT;

#if FOC_POSITION_DEMO_ENABLE
#if !FOC_IMPEDANCE_DEMO_ENABLE
    if (position_demo_completed == 0U &&
#if FOC_COMPOSITE_DEMO_ENABLE || FOC_PRE_POSITION_IMPEDANCE_ENABLE
        impedance_demo_completed != 0U &&
        (now_tick - position_stage_start_tick) >=
            FOC_POSITION_TEST_DURATION_MS)
#else
        (now_tick - start_tick) >= FOC_POSITION_TEST_DURATION_MS)
#endif
    {
      g_foc_fault = 10U;
      g_foc_speed_reference_rpm = 0.0f;
      g_foc_iq_reference_a = 0.0f;
      g_foc_enabled = 0U;
      Motor_PWM_Off();
      continue;
    }
#endif
#endif

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
            float accepted_magnitude = AbsFloat(encoder_innovation_rad);
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
            float velocity_correction_unclamped =
                FOC_ENCODER_VELOCITY_GAIN *
                    (electrical_velocity_instant -
                     g_foc_electrical_velocity_rad_s);
            float velocity_correction = ClampFloat(
                velocity_correction_unclamped,
                -FOC_ENCODER_MAX_VELOCITY_STEP_RAD_S,
                FOC_ENCODER_MAX_VELOCITY_STEP_RAD_S);
            float electrical_velocity_filtered =
                g_foc_electrical_velocity_rad_s + velocity_correction;

            encoder_accepted_samples++;
            if (accepted_magnitude > encoder_max_accepted_innovation_rad)
            {
              encoder_max_accepted_innovation_rad = accepted_magnitude;
            }
            if (angle_correction != angle_correction_unclamped)
            {
              encoder_limited_corrections++;
            }
            if (velocity_correction != velocity_correction_unclamped)
            {
              encoder_limited_velocity_corrections++;
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
#if !FOC_IMPEDANCE_DEMO_ENABLE
                float position_dt_s =
                    (float)elapsed_speed_cycles / (float)SystemCoreClock;
#endif

#if FOC_IMPEDANCE_DEMO_ENABLE
                {
                  uint32_t impedance_elapsed_ms = now_tick - start_tick;

                  if (impedance_elapsed_ms <
                      (FOC_IMPEDANCE_DEMO_DURATION_MS +
                       FOC_IMPEDANCE_RELEASE_RAMP_MS))
                  {
                    float stiffness_a_per_output_deg;
                    float control_scale = 1.0f;
                    float output_position_error_deg =
                        FOC_IMPEDANCE_TARGET_OUTPUT_DEG -
                        output_position_deg;
                    float iq_unclamped_a;

                    if (impedance_elapsed_ms >=
                        FOC_IMPEDANCE_DEMO_DURATION_MS)
                    {
                      uint32_t release_elapsed_ms =
                          impedance_elapsed_ms -
                          FOC_IMPEDANCE_DEMO_DURATION_MS;
                      control_scale =
                          1.0f -
                          ((float)release_elapsed_ms /
                           (float)FOC_IMPEDANCE_RELEASE_RAMP_MS);
                    }

                    /* Convert requested ideal output stiffness to motor Iq.
                     * No integral or disturbance boost is allowed here: a
                     * sustained applied torque must create a proportional
                     * displacement, like a real spring. */
                    stiffness_a_per_output_deg =
                        FOC_IMPEDANCE_STIFFNESS_NM_PER_OUTPUT_DEG /
                        (FOC_MOTOR_ESTIMATED_KT_NM_PER_A *
                         FOC_MOTOR_TO_OUTPUT_GEAR_RATIO);
                    iq_unclamped_a =
                        FOC_OUTPUT_DIRECTION_SIGN *
                        control_scale *
                        ((stiffness_a_per_output_deg *
                          output_position_error_deg) -
                         (FOC_IMPEDANCE_DAMPING_A_PER_OUTPUT_RPM *
                          output_rpm_filtered));

                    g_foc_output_position_target_deg =
                        FOC_IMPEDANCE_TARGET_OUTPUT_DEG;
                    g_foc_output_trajectory_position_deg =
                        FOC_IMPEDANCE_TARGET_OUTPUT_DEG;
                    g_foc_speed_reference_rpm = 0.0f;
                    g_foc_debug_position_error_deg =
                        output_position_error_deg;
                    g_foc_debug_speed_error_rpm =
                        -mechanical_rpm_filtered;
                    g_foc_debug_speed_integrator_a = 0.0f;
                    g_foc_debug_breakaway_current_a = 0.0f;
                    g_foc_debug_iq_saturated =
                        (AbsFloat(iq_unclamped_a) >=
                         foc_iq_limit_a) ? 1U : 0U;
                    g_foc_iq_reference_a = ClampFloat(
                        iq_unclamped_a,
                        -foc_iq_limit_a,
                        foc_iq_limit_a);

                    impedance_control_updates++;
                    if (AbsFloat(output_position_error_deg) >
                        impedance_max_abs_position_error_deg)
                    {
                      impedance_max_abs_position_error_deg =
                          AbsFloat(output_position_error_deg);
                    }
                    if (AbsFloat(g_foc_iq_a) >
                        impedance_peak_abs_iq_a)
                    {
                      impedance_peak_abs_iq_a =
                          AbsFloat(g_foc_iq_a);
                    }
                    if (g_foc_debug_iq_saturated != 0U)
                    {
                      impedance_iq_saturated_updates++;
                    }
                  }
                  else
                  {
                    /* Remove commanded spring torque before inhibiting PWM so
                     * the final state is a controlled zero-current release. */
                    g_foc_iq_reference_a = 0.0f;
                    g_foc_debug_iq_saturated = 0U;
                    if (AbsFloat(g_foc_iq_a) < 0.5f)
                    {
                      impedance_demo_completed = 1U;
                      g_foc_enabled = 0U;
                      Motor_PWM_Off();
                    }
                  }
                }
#else
#if FOC_COMPOSITE_DEMO_ENABLE || FOC_PRE_POSITION_IMPEDANCE_ENABLE
                if (impedance_demo_completed == 0U)
                {
                  uint32_t impedance_elapsed_ms =
                      now_tick - impedance_stage_start_tick;
                  float stiffness_a_per_output_deg =
                      FOC_IMPEDANCE_STIFFNESS_NM_PER_OUTPUT_DEG /
                      (FOC_MOTOR_ESTIMATED_KT_NM_PER_A *
                       FOC_MOTOR_TO_OUTPUT_GEAR_RATIO);
                  float output_position_error_deg =
                      FOC_IMPEDANCE_TARGET_OUTPUT_DEG - output_position_deg;
                  float iq_unclamped_a =
                      FOC_OUTPUT_DIRECTION_SIGN *
                      ((stiffness_a_per_output_deg *
                        output_position_error_deg) -
                       (FOC_IMPEDANCE_DAMPING_A_PER_OUTPUT_RPM *
                        output_rpm_filtered));

                  g_foc_output_position_target_deg =
                      FOC_IMPEDANCE_TARGET_OUTPUT_DEG;
                  g_foc_output_trajectory_position_deg =
                      FOC_IMPEDANCE_TARGET_OUTPUT_DEG;
                  g_foc_speed_reference_rpm = 0.0f;
                  g_foc_debug_position_error_deg =
                      output_position_error_deg;
                  g_foc_debug_speed_error_rpm = -mechanical_rpm_filtered;
                  g_foc_debug_speed_integrator_a = 0.0f;
                  g_foc_debug_breakaway_current_a = 0.0f;
                  g_foc_debug_iq_saturated =
                      (AbsFloat(iq_unclamped_a) >= foc_iq_limit_a) ? 1U : 0U;
                  g_foc_iq_reference_a = ClampFloat(
                      iq_unclamped_a, -foc_iq_limit_a, foc_iq_limit_a);

                  impedance_control_updates++;
                  if (AbsFloat(output_position_error_deg) >
                      impedance_max_abs_position_error_deg)
                  {
                    impedance_max_abs_position_error_deg =
                        AbsFloat(output_position_error_deg);
                  }
                  if (AbsFloat(g_foc_iq_a) > impedance_peak_abs_iq_a)
                  {
                    impedance_peak_abs_iq_a = AbsFloat(g_foc_iq_a);
                  }
                  if (g_foc_debug_iq_saturated != 0U)
                  {
                    impedance_iq_saturated_updates++;
                  }

                  if (impedance_elapsed_ms >=
                      FOC_COMPOSITE_IMPEDANCE_DURATION_MS)
                  {
                    /* Start the position trajectory at the measured output
                     * angle.  This avoids a current step if the operator is
                     * still releasing a displaced impedance-test shaft. */
                    impedance_demo_completed = 1U;
                    position_stage_start_tick = now_tick;
                    position_step_start_tick = now_tick;
                    position_hold_start_tick = 0U;
                    position_target_index = 0U;
                    output_trajectory_speed_rpm = 0.0f;
                    position_integrator_a = 0.0f;
                    g_foc_output_position_target_deg =
                        g_foc_output_position_demo_targets_deg[0];
                    g_foc_output_trajectory_position_deg =
                        output_position_deg;
                    g_foc_speed_reference_rpm = 0.0f;
                    g_foc_iq_reference_a = 0.0f;
                    g_foc_debug_position_error_deg = 0.0f;
                    g_foc_debug_speed_error_rpm = 0.0f;
                    g_foc_debug_speed_integrator_a = 0.0f;
                    g_foc_debug_breakaway_current_a = 0.0f;
                    g_foc_debug_iq_saturated = 0U;
                    foc_iq_limit_a = FOC_POSITION_IQ_LIMIT_A;
                    g_foc_active_hard_current_limit_a =
                        FOC_POSITION_HARD_CURRENT_LIMIT_A;
                    g_foc_active_dq_fault_limit_a =
                        FOC_POSITION_DQ_FAULT_LIMIT_A;
                    foc_overspeed_rpm = FOC_POSITION_OVERSPEED_RPM;
                    FOC_LogSampleCapture(now_tick - start_tick,
                                         mechanical_rpm_filtered);
                    last_log_tick = now_tick;
                    foc_log_interval_ms = FOC_POSITION_LOG_INTERVAL_MS;
                    foc_min_current_samples_per_log =
                        FOC_POSITION_MIN_CURRENT_SAMPLES_PER_LOG;
                  }
                }
                else
#endif
                if (position_demo_completed == 0U)
                {
                float output_trajectory_remaining_deg =
                    g_foc_output_position_target_deg -
                    g_foc_output_trajectory_position_deg;
                float output_trajectory_stop_rpm = sqrtf(
                    FOC_ACTIVE_POSITION_DECEL_OUTPUT_RPM_S *
                    AbsFloat(output_trajectory_remaining_deg) / 3.0f);
                float output_trajectory_desired_rpm =
                    (output_trajectory_remaining_deg > 0.0f) ?
                        ClampFloat(output_trajectory_stop_rpm, 0.0f,
                                   FOC_ACTIVE_POSITION_MAX_OUTPUT_RPM) :
                    (output_trajectory_remaining_deg < 0.0f) ?
                        -ClampFloat(output_trajectory_stop_rpm, 0.0f,
                                    FOC_ACTIVE_POSITION_MAX_OUTPUT_RPM) :
                        0.0f;
                float output_trajectory_old_rpm =
                    output_trajectory_speed_rpm;
                float output_trajectory_rate_rpm_s =
                    ((output_trajectory_old_rpm *
                      output_trajectory_desired_rpm) < 0.0f ||
                     AbsFloat(output_trajectory_desired_rpm) <
                         AbsFloat(output_trajectory_old_rpm)) ?
                        FOC_ACTIVE_POSITION_DECEL_OUTPUT_RPM_S :
                        FOC_ACTIVE_POSITION_ACCEL_OUTPUT_RPM_S;
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
                  /* This timeout covers the initial move and recovery from a
                   * disturbance, not time already spent stably dwelling. */
                  position_step_start_tick = now_tick;
                  if (position_hold_start_tick == 0U)
                  {
                    position_hold_start_tick = now_tick;
                  }
                  else if ((now_tick - position_hold_start_tick) >=
                           FOC_ACTIVE_POSITION_HOLD_MS)
                  {
                    if ((uint8_t)(position_target_index + 1U) <
                        position_target_count)
                    {
                      position_target_index++;
                      g_foc_output_position_target_deg =
                          g_foc_output_position_demo_targets_deg[
                              position_target_index];
                      position_hold_start_tick = 0U;
                      position_step_start_tick = now_tick;
                    }
                    else
                    {
                      position_demo_final_output_deg = output_position_deg;
                      position_demo_final_motor_deg = motor_position_deg;
                      position_demo_completed = 1U;
#if FOC_FORCE_SCALE_TEST_ENABLE
                      force_torque_stage_requested = 1U;
#else
                      velocity_demo_started = 1U;
                      velocity_stage_start_tick = now_tick;
                      velocity_stage_start_elapsed_ms =
                          now_tick - start_tick;
                      /* Retain 200 ms resolution through the short position
                       * moves, then use one-second buckets so the complete
                       * 50-second velocity stage fits in the same RAM log. */
                      FOC_LogSampleCapture(now_tick - start_tick,
                                           mechanical_rpm_filtered);
                      last_log_tick = now_tick;
                      foc_log_interval_ms =
                          FOC_VELOCITY_LOG_INTERVAL_MS;
                      foc_min_current_samples_per_log =
                          FOC_VELOCITY_MIN_CURRENT_SAMPLES_PER_LOG;
                      velocity_output_reference_rpm = 0.0f;
                      velocity_integrator_a = 0.0f;
                      foc_overspeed_rpm =
                          FOC_ACTIVE_VELOCITY_OVERSPEED_MOTOR_RPM;
                      g_foc_speed_reference_rpm = 0.0f;
                      g_foc_iq_reference_a = 0.0f;
                      g_foc_debug_position_error_deg = 0.0f;
                      g_foc_debug_speed_error_rpm = 0.0f;
                      g_foc_debug_speed_integrator_a = 0.0f;
                      g_foc_debug_breakaway_current_a = 0.0f;
                      g_foc_debug_iq_saturated = 0U;
#endif
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
                /* Once the target has first settled, a recoverable
                 * disturbance does not restart the fixed dwell indefinitely.
                 * The sequence advances as soon as the dwell has elapsed and
                 * the output is stable again.  The recovery timeout above is
                 * still measured from the last stable observation. */

                if (position_demo_completed == 0U &&
                    g_foc_enabled != 0U)
                {
                  float integrator_delta_a =
                      FOC_OUTPUT_DIRECTION_SIGN *
                      FOC_ACTIVE_POSITION_KI_A_PER_OUTPUT_DEG_S *
                      output_tracking_position_error_deg * position_dt_s;
                  float integrator_candidate_a;
                  float disturbance_boost_a = 0.0f;
                  float iq_pd_a =
                      FOC_OUTPUT_DIRECTION_SIGN *
                      ((FOC_ACTIVE_POSITION_KP_A_PER_OUTPUT_DEG *
                        output_tracking_position_error_deg) +
                       (FOC_ACTIVE_POSITION_KD_A_PER_OUTPUT_RPM *
                        output_speed_error_rpm));
                  float iq_candidate_a;
                  float iq_candidate_clamped_a;
                  float iq_unclamped;

                  /* There is no load-side force sensor, so infer an external
                   * impedance from position error while the output is moving
                   * slowly.  This proportional boost is immediate and fades
                   * smoothly with both recovered position and shaft speed;
                   * the integral then learns only the sustained load torque. */
#if !FOC_FORCE_SCALE_TEST_ENABLE
                  if (AbsFloat(output_tracking_position_error_deg) >
                      FOC_POSITION_DISTURBANCE_ONSET_OUTPUT_DEG)
                  {
                    float disturbance_error_fraction = ClampFloat(
                        (AbsFloat(output_tracking_position_error_deg) -
                         FOC_POSITION_DISTURBANCE_ONSET_OUTPUT_DEG) /
                            (FOC_POSITION_DISTURBANCE_FULL_OUTPUT_DEG -
                             FOC_POSITION_DISTURBANCE_ONSET_OUTPUT_DEG),
                        0.0f, 1.0f);
                    float disturbance_speed_fraction = ClampFloat(
                        1.0f -
                            (AbsFloat(output_rpm_filtered) /
                             FOC_POSITION_DISTURBANCE_FADE_OUTPUT_RPM),
                        0.0f, 1.0f);
                    float disturbance_velocity_deficit_fraction = ClampFloat(
                        AbsFloat(output_speed_error_rpm) /
                            FOC_POSITION_DISTURBANCE_FULL_SPEED_ERROR_RPM,
                        0.0f, 1.0f);
                    float disturbance_direction =
                        (output_tracking_position_error_deg > 0.0f) ?
                            1.0f : -1.0f;

                    if (disturbance_velocity_deficit_fraction >
                        disturbance_speed_fraction)
                    {
                      disturbance_speed_fraction =
                          disturbance_velocity_deficit_fraction;
                    }

                    disturbance_boost_a =
                        FOC_OUTPUT_DIRECTION_SIGN * disturbance_direction *
                        FOC_POSITION_DISTURBANCE_BOOST_A *
                        disturbance_error_fraction *
                        disturbance_speed_fraction;
                    iq_pd_a += disturbance_boost_a;
                  }
#endif

                  /* When a disturbance is released, the position error reverses
                   * before the stored load-torque estimate does.  Unload that
                   * stale estimate quickly so it cannot hold the shaft past the
                   * target for several seconds. */
                  if ((position_integrator_a * integrator_delta_a) < 0.0f)
                  {
                    integrator_delta_a *=
                        FOC_POSITION_INTEGRAL_UNWIND_MULTIPLIER;
                  }
                  integrator_candidate_a = ClampFloat(
                      position_integrator_a + integrator_delta_a,
                      -FOC_POSITION_INTEGRAL_LIMIT_A,
                      FOC_POSITION_INTEGRAL_LIMIT_A);
                  iq_candidate_a = iq_pd_a + integrator_candidate_a;
                  iq_candidate_clamped_a = ClampFloat(
                      iq_candidate_a, -foc_iq_limit_a, foc_iq_limit_a);

                  /* A constant load previously balanced the proportional
                   * torque at a fixed position error.  The integral term now
                   * builds the additional holding/drive torque needed to
                   * restore the trajectory.  Only accept integration that is
                   * inside the Iq limit or that drives a saturated command back
                   * toward the available range. */
                  if (iq_candidate_clamped_a == iq_candidate_a ||
                      (iq_candidate_clamped_a >= foc_iq_limit_a &&
                       integrator_delta_a < 0.0f) ||
                      (iq_candidate_clamped_a <= -foc_iq_limit_a &&
                       integrator_delta_a > 0.0f))
                  {
                    position_integrator_a = integrator_candidate_a;
                  }
                  iq_unclamped = iq_pd_a + position_integrator_a;

                  /* The outer PID is output-referenced. This prevents the gear
                   * conversion from multiplying its effective stiffness and
                   * damping by the ratio; current/electrical FOC stays motor-side. */
                  g_foc_debug_speed_integrator_a = position_integrator_a;
                  g_foc_debug_breakaway_current_a = disturbance_boost_a;
                  g_foc_debug_iq_saturated =
                      (AbsFloat(iq_unclamped) >= foc_iq_limit_a) ? 1U : 0U;
                  g_foc_iq_reference_a = ClampFloat(
                      iq_unclamped,
                      -foc_iq_limit_a,
                      foc_iq_limit_a);

                  position_control_updates++;
                  if (AbsFloat(output_tracking_position_error_deg) >
                      position_max_abs_tracking_error_deg)
                  {
                    position_max_abs_tracking_error_deg =
                        AbsFloat(output_tracking_position_error_deg);
                  }
                  if (AbsFloat(disturbance_boost_a) > 0.001f)
                  {
                    position_boost_updates++;
                  }
                  if (AbsFloat(disturbance_boost_a) >
                      position_max_abs_boost_a)
                  {
                    position_max_abs_boost_a =
                        AbsFloat(disturbance_boost_a);
                  }
                  if (g_foc_debug_iq_saturated != 0U)
                  {
                    position_iq_saturated_updates++;
                  }

#if FOC_FORCE_SCALE_TEST_ENABLE
                  /* The rigid scale stops the bar before the nominal travel
                   * target, so a normal position-settle transition cannot
                   * occur. Treat sustained near-zero output speed at the
                   * bounded approach-current clamp as contact. Requiring
                   * directional travel and remaining trajectory error keeps
                   * normal acceleration and arrival at the fallback target
                   * out of this path. */
                  if ((FOC_FORCE_TEST_CCW_SIGN * output_position_deg) >=
                          FOC_FORCE_TEST_CONTACT_MIN_TRAVEL_DEG &&
                      (FOC_FORCE_TEST_CCW_SIGN *
                       output_final_position_error_deg) >=
                          FOC_FORCE_TEST_CONTACT_MIN_REMAINING_DEG &&
                      AbsFloat(output_rpm_filtered) <=
                          FOC_FORCE_TEST_CONTACT_MAX_OUTPUT_RPM &&
                      AbsFloat(g_foc_iq_reference_a) >=
                          FOC_FORCE_TEST_CONTACT_MIN_IQ_A)
                  {
                    if (force_contact_candidate_start_tick == 0U)
                    {
                      force_contact_candidate_start_tick = now_tick;
                    }
                    else if ((now_tick -
                              force_contact_candidate_start_tick) >=
                             FOC_FORCE_TEST_CONTACT_CONFIRM_MS)
                    {
                      force_contact_detected = 1U;
                      force_contact_elapsed_ms = now_tick - start_tick;
                      force_contact_output_deg = output_position_deg;
                      position_demo_final_output_deg = output_position_deg;
                      position_demo_final_motor_deg = motor_position_deg;
                      position_demo_completed = 1U;
                      force_torque_stage_requested = 1U;
                    }
                  }
                  else
                  {
                    force_contact_candidate_start_tick = 0U;
                  }
#endif
                }

#if FOC_FORCE_SCALE_TEST_ENABLE
                if (force_torque_stage_requested != 0U &&
                    g_foc_enabled != 0U)
                {
                  force_torque_stage_requested = 0U;
                  torque_demo_started = 1U;
                  torque_stage_start_tick = now_tick;
                  torque_stage_start_elapsed_ms = now_tick - start_tick;
                  FOC_LogSampleCapture(now_tick - start_tick,
                                       mechanical_rpm_filtered);
                  last_log_tick = now_tick;
                  foc_log_interval_ms = FOC_TORQUE_LOG_INTERVAL_MS;
                  foc_min_current_samples_per_log =
                      FOC_TORQUE_MIN_CURRENT_SAMPLES_PER_LOG;
                  foc_overspeed_rpm =
                      FOC_FORCE_TEST_TORQUE_OVERSPEED_MOTOR_RPM;
                  foc_iq_limit_a = FOC_FORCE_TEST_TARGET_IQ_A;
                  g_foc_active_hard_current_limit_a =
                      FOC_FORCE_TEST_HARD_CURRENT_LIMIT_A;
                  g_foc_active_dq_fault_limit_a =
                      FOC_FORCE_TEST_DQ_FAULT_LIMIT_A;
                  /* Preserve the approach torque at contact. Resetting Iq to
                   * zero here lets the compressed scale/fixture rebound
                   * before the force ramp begins, which can create a false
                   * overspeed fault during an otherwise valid handoff. */
                  torque_motor_reference_nm =
                      (g_foc_iq_reference_a /
                       FOC_OUTPUT_DIRECTION_SIGN) *
                      FOC_MOTOR_ESTIMATED_KT_NM_PER_A;
                  g_foc_speed_reference_rpm = 0.0f;
                  g_foc_debug_position_error_deg = 0.0f;
                  g_foc_debug_speed_error_rpm = 0.0f;
                  g_foc_debug_speed_integrator_a = 0.0f;
                  g_foc_debug_breakaway_current_a = 0.0f;
                  g_foc_debug_iq_saturated = 0U;
                }
#endif
                }
                else if (velocity_demo_completed == 0U &&
                         g_foc_enabled != 0U)
                {
                  uint32_t velocity_elapsed_ms =
                      now_tick - velocity_stage_start_tick;
                  float velocity_desired_output_rpm =
                      (velocity_elapsed_ms <
                       FOC_ACTIVE_VELOCITY_DECEL_START_MS) ?
                          FOC_ACTIVE_VELOCITY_TARGET_OUTPUT_RPM : 0.0f;
                  float velocity_ramp_rate_output_rpm_s =
                      (velocity_desired_output_rpm <
                       velocity_output_reference_rpm) ?
                          FOC_ACTIVE_VELOCITY_DECEL_OUTPUT_RPM_S :
                          FOC_ACTIVE_VELOCITY_ACCEL_OUTPUT_RPM_S;
                  float velocity_error_output_rpm;
                  float velocity_integrator_delta_a;
                  float velocity_integrator_candidate_a;
                  float velocity_iq_unclamped_a;
                  float velocity_iq_command_a;

#if FOC_VELOCITY_HEAT_TEST_ENABLE
                  Debug_Status2_Set(
                      (velocity_elapsed_ms >=
                           FOC_HEAT_TEST_MEASUREMENT_START_MS &&
                       velocity_elapsed_ms <
                           FOC_HEAT_TEST_DECEL_START_MS) ?
                          GPIO_PIN_SET : GPIO_PIN_RESET);
#endif

                  velocity_output_reference_rpm = RampToward(
                      velocity_output_reference_rpm,
                      velocity_desired_output_rpm,
                      velocity_ramp_rate_output_rpm_s,
                      position_dt_s);
                  velocity_error_output_rpm =
                      velocity_output_reference_rpm - output_rpm_filtered;
                  velocity_integrator_delta_a =
                      FOC_OUTPUT_DIRECTION_SIGN *
                      FOC_VELOCITY_KI_A_PER_OUTPUT_RPM_S *
                      velocity_error_output_rpm * position_dt_s;

                  /* Remove stored acceleration torque promptly during the
                   * commanded deceleration without sacrificing steady-speed
                   * load rejection. */
                  if ((velocity_integrator_a *
                       velocity_integrator_delta_a) < 0.0f)
                  {
                    velocity_integrator_delta_a *=
                        FOC_VELOCITY_INTEGRAL_UNWIND_MULTIPLIER;
                  }
                  velocity_integrator_candidate_a = ClampFloat(
                      velocity_integrator_a +
                          velocity_integrator_delta_a,
                      -FOC_VELOCITY_INTEGRAL_LIMIT_A,
                      FOC_VELOCITY_INTEGRAL_LIMIT_A);
                  velocity_iq_unclamped_a =
                      (FOC_OUTPUT_DIRECTION_SIGN *
                       FOC_VELOCITY_KP_A_PER_OUTPUT_RPM *
                       velocity_error_output_rpm) +
                      velocity_integrator_candidate_a;
                  velocity_iq_command_a = ClampFloat(
                      velocity_iq_unclamped_a,
                      -foc_iq_limit_a,
                      foc_iq_limit_a);

                  /* Conditional integration keeps the speed PI from winding
                   * up against the shared 30 A torque-current ceiling. */
                  if (velocity_iq_command_a == velocity_iq_unclamped_a ||
                      (velocity_iq_command_a >= foc_iq_limit_a &&
                       velocity_integrator_delta_a < 0.0f) ||
                      (velocity_iq_command_a <= -foc_iq_limit_a &&
                       velocity_integrator_delta_a > 0.0f))
                  {
                    velocity_integrator_a =
                        velocity_integrator_candidate_a;
                  }
                  velocity_iq_unclamped_a =
                      (FOC_OUTPUT_DIRECTION_SIGN *
                       FOC_VELOCITY_KP_A_PER_OUTPUT_RPM *
                       velocity_error_output_rpm) +
                      velocity_integrator_a;

                  g_foc_speed_reference_rpm =
                      FOC_OUTPUT_DIRECTION_SIGN *
                      FOC_MOTOR_TO_OUTPUT_GEAR_RATIO *
                      velocity_output_reference_rpm;
                  g_foc_debug_speed_error_rpm =
                      g_foc_speed_reference_rpm - mechanical_rpm_filtered;
                  g_foc_debug_position_error_deg = 0.0f;
                  g_foc_debug_speed_integrator_a = velocity_integrator_a;
                  g_foc_debug_breakaway_current_a = 0.0f;
                  g_foc_debug_iq_saturated =
                      (AbsFloat(velocity_iq_unclamped_a) >=
                       foc_iq_limit_a) ? 1U : 0U;
                  g_foc_iq_reference_a = ClampFloat(
                      velocity_iq_unclamped_a,
                      -foc_iq_limit_a,
                      foc_iq_limit_a);

                  velocity_control_updates++;
                  velocity_final_output_rpm = output_rpm_filtered;
                  if (AbsFloat(output_rpm_filtered) >
                      velocity_peak_abs_output_rpm)
                  {
                    velocity_peak_abs_output_rpm =
                        AbsFloat(output_rpm_filtered);
                  }
                  if (AbsFloat(velocity_error_output_rpm) >
                      velocity_max_abs_error_output_rpm)
                  {
                    velocity_max_abs_error_output_rpm =
                        AbsFloat(velocity_error_output_rpm);
                  }
                  if (g_foc_debug_iq_saturated != 0U)
                  {
                    velocity_iq_saturated_updates++;
                  }

                  if (velocity_elapsed_ms >=
                          FOC_ACTIVE_VELOCITY_DEMO_DURATION_MS &&
                      AbsFloat(velocity_output_reference_rpm) < 0.1f &&
                      AbsFloat(output_rpm_filtered) <=
                          FOC_VELOCITY_STOP_TOLERANCE_OUTPUT_RPM)
                  {
                    velocity_demo_completed = 1U;
#if FOC_COMPOSITE_DEMO_ENABLE || FOC_VELOCITY_HEAT_TEST_ENABLE
                    FOC_LogSampleCapture(now_tick - start_tick,
                                         mechanical_rpm_filtered);
                    g_foc_speed_reference_rpm = 0.0f;
                    g_foc_iq_reference_a = 0.0f;
                    g_foc_debug_speed_error_rpm = 0.0f;
                    g_foc_debug_speed_integrator_a = 0.0f;
                    g_foc_debug_breakaway_current_a = 0.0f;
                    g_foc_debug_iq_saturated = 0U;
                    g_foc_enabled = 0U;
                    Motor_PWM_Off();
#else
                    torque_demo_started = 1U;
                    torque_stage_start_tick = now_tick;
                    torque_stage_start_elapsed_ms =
                        now_tick - start_tick;
                    FOC_LogSampleCapture(now_tick - start_tick,
                                         mechanical_rpm_filtered);
                    last_log_tick = now_tick;
                    foc_log_interval_ms = FOC_TORQUE_LOG_INTERVAL_MS;
                    foc_min_current_samples_per_log =
                        FOC_TORQUE_MIN_CURRENT_SAMPLES_PER_LOG;
                    foc_overspeed_rpm =
                        FOC_TORQUE_OVERSPEED_MOTOR_RPM;
                    torque_motor_reference_nm = 0.0f;
                    g_foc_speed_reference_rpm = 0.0f;
                    g_foc_iq_reference_a = 0.0f;
                    g_foc_debug_speed_error_rpm = 0.0f;
                    g_foc_debug_speed_integrator_a = 0.0f;
                    g_foc_debug_breakaway_current_a = 0.0f;
                    g_foc_debug_iq_saturated = 0U;
#endif
                  }
                  else if (velocity_elapsed_ms >=
                           (FOC_ACTIVE_VELOCITY_DEMO_DURATION_MS +
                            FOC_ACTIVE_VELOCITY_SETTLE_TIMEOUT_MS))
                  {
                    g_foc_fault = 11U;
                    g_foc_speed_reference_rpm = 0.0f;
                    g_foc_iq_reference_a = 0.0f;
                    g_foc_enabled = 0U;
                    Motor_PWM_Off();
                  }
                }
                else if (torque_demo_completed == 0U &&
                         g_foc_enabled != 0U)
                {
                  uint32_t torque_elapsed_ms =
                      now_tick - torque_stage_start_tick;
                  float torque_desired_motor_nm;
                  float torque_iq_reference_a;

#if FOC_FORCE_SCALE_TEST_ENABLE
                  if (torque_elapsed_ms <
                      (FOC_FORCE_TEST_TORQUE_RAMP_MS +
                       FOC_FORCE_TEST_TORQUE_HOLD_MS))
                  {
                    torque_desired_motor_nm =
                        FOC_FORCE_TEST_CCW_SIGN *
                        FOC_ACTIVE_TORQUE_TARGET_MOTOR_NM;
                  }
                  else
                  {
                    torque_desired_motor_nm = 0.0f;
                  }
#else
                  if (torque_elapsed_ms <
                      FOC_TORQUE_POSITIVE_END_MS)
                  {
                    torque_desired_motor_nm = FOC_TORQUE_TARGET_MOTOR_NM;
                  }
                  else if (torque_elapsed_ms <
                           FOC_TORQUE_REVERSE_START_MS)
                  {
                    torque_desired_motor_nm = 0.0f;
                  }
                  else if (torque_elapsed_ms <
                           FOC_TORQUE_REVERSE_END_MS)
                  {
                    torque_desired_motor_nm = -FOC_TORQUE_TARGET_MOTOR_NM;
                  }
                  else
                  {
                    torque_desired_motor_nm = 0.0f;
                  }
#endif

                  torque_motor_reference_nm = RampToward(
                      torque_motor_reference_nm,
                      torque_desired_motor_nm,
                      FOC_ACTIVE_TORQUE_RAMP_MOTOR_NM_PER_S,
                      position_dt_s);
                  torque_iq_reference_a =
                      torque_motor_reference_nm /
                      FOC_MOTOR_ESTIMATED_KT_NM_PER_A;

                  /* In surface-PMSM FOC, Iq is the torque-producing current.
                   * Commanded motor torque is converted by Iq_ref=torque/Kt.
                   * Id remains zero. Torque mode bypasses position and
                   * velocity outer loops while retaining the 20 kHz current
                   * PI, modulation clamp, and all current protections. */
                  g_foc_speed_reference_rpm = 0.0f;
                  g_foc_debug_speed_error_rpm = 0.0f;
                  g_foc_debug_position_error_deg = 0.0f;
                  g_foc_debug_speed_integrator_a = 0.0f;
                  g_foc_debug_breakaway_current_a = 0.0f;
                  g_foc_debug_iq_saturated =
                      (AbsFloat(torque_iq_reference_a) >=
                       foc_iq_limit_a) ? 1U : 0U;
                  g_foc_iq_reference_a = ClampFloat(
                      FOC_OUTPUT_DIRECTION_SIGN *
                          torque_iq_reference_a,
                      -foc_iq_limit_a,
                      foc_iq_limit_a);

                  torque_control_updates++;
                  if (AbsFloat(g_foc_iq_a) > torque_peak_abs_iq_a)
                  {
                    torque_peak_abs_iq_a = AbsFloat(g_foc_iq_a);
                  }
                  if (AbsFloat(mechanical_rpm_filtered) >
                      torque_peak_abs_motor_rpm)
                  {
                    torque_peak_abs_motor_rpm =
                        AbsFloat(mechanical_rpm_filtered);
                  }

                  if (torque_elapsed_ms >=
                          FOC_ACTIVE_TORQUE_DURATION_MS &&
                      AbsFloat(torque_motor_reference_nm) < 0.002f &&
                      AbsFloat(g_foc_iq_a) < 0.5f)
                  {
                    torque_demo_completed = 1U;
                    g_foc_iq_reference_a = 0.0f;
                    g_foc_enabled = 0U;
                    Motor_PWM_Off();
                  }
                  else if (torque_elapsed_ms >=
                           (FOC_ACTIVE_TORQUE_DURATION_MS +
                            FOC_ACTIVE_TORQUE_SETTLE_TIMEOUT_MS))
                  {
                    g_foc_fault = 12U;
                    g_foc_iq_reference_a = 0.0f;
                    g_foc_enabled = 0U;
                    Motor_PWM_Off();
                  }
                }
#endif
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

      /* The active stage supplies its own motor-side overspeed limit:
       * position, velocity, and direct-Iq torque use different envelopes. */
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
      Motor_ReportDRVFaultAndShutdown("FOC run");
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
#if FOC_IMPEDANCE_DEMO_ENABLE
  if (g_foc_fault == 0U && impedance_demo_completed == 0U)
  {
    g_foc_fault = 13U;
    g_foc_iq_reference_a = 0.0f;
    g_foc_enabled = 0U;
    Motor_PWM_Off();
  }
#else
#if FOC_COMPOSITE_DEMO_ENABLE || FOC_PRE_POSITION_IMPEDANCE_ENABLE
  if (g_foc_fault == 0U && impedance_demo_completed == 0U)
  {
    g_foc_fault = 13U;
    g_foc_iq_reference_a = 0.0f;
    g_foc_enabled = 0U;
    Motor_PWM_Off();
  }
  else
#endif
  if (g_foc_fault == 0U && position_demo_completed == 0U)
  {
    g_foc_fault = 10U;
    g_foc_speed_reference_rpm = 0.0f;
    g_foc_iq_reference_a = 0.0f;
    g_foc_enabled = 0U;
    Motor_PWM_Off();
  }
  else if (g_foc_fault == 0U && velocity_demo_completed == 0U)
  {
    g_foc_fault = 11U;
    g_foc_speed_reference_rpm = 0.0f;
    g_foc_iq_reference_a = 0.0f;
    g_foc_enabled = 0U;
    Motor_PWM_Off();
  }
#if !FOC_COMPOSITE_DEMO_ENABLE && !FOC_VELOCITY_HEAT_TEST_ENABLE
  else if (g_foc_fault == 0U && torque_demo_completed == 0U)
  {
    g_foc_fault = 12U;
    g_foc_iq_reference_a = 0.0f;
    g_foc_enabled = 0U;
    Motor_PWM_Off();
  }
#endif
#endif
#endif

  {
    uint32_t final_elapsed_ms = HAL_GetTick() - start_tick;

    if (g_foc_log_count == 0U ||
        g_foc_log[g_foc_log_count - 1U].time_ms != final_elapsed_ms)
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
  printf("FOC duplicate encoder counts observed=%lu\r\n",
         (unsigned long)encoder_duplicate_samples);
  printf("FOC encoder innovation rejections=%lu, max=%ld mrad\r\n",
         (unsigned long)encoder_rejected_samples,
         (long)(encoder_max_rejected_innovation_rad * 1000.0f));
  printf("FOC encoder accepted observations=%lu, max innovation=%ld mrad\r\n",
         (unsigned long)encoder_accepted_samples,
         (long)(encoder_max_accepted_innovation_rad * 1000.0f));
  printf("FOC encoder bounded angle corrections=%lu/%lu, maximum phase step=%ld mrad; bounded velocity corrections=%lu\r\n",
         (unsigned long)encoder_limited_corrections,
         (unsigned long)encoder_accepted_samples,
         (long)(FOC_ENCODER_MAX_CORRECTION_RAD * 1000.0f),
         (unsigned long)encoder_limited_velocity_corrections);
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
#if FOC_IMPEDANCE_DEMO_ENABLE
  printf("FOC impedance result: completed=%u, target=%ld mdeg output, controller_updates=%lu, Iq_saturated=%lu, max_abs_error=%ld mdeg, peak_abs_Iq=%ld mA => %ld mNm ideal output torque\r\n",
         (unsigned int)impedance_demo_completed,
         (long)(FOC_IMPEDANCE_TARGET_OUTPUT_DEG * 1000.0f),
         (unsigned long)impedance_control_updates,
         (unsigned long)impedance_iq_saturated_updates,
         (long)(impedance_max_abs_position_error_deg * 1000.0f),
         (long)(impedance_peak_abs_iq_a * 1000.0f),
         (long)(impedance_peak_abs_iq_a *
                FOC_MOTOR_ESTIMATED_KT_NM_PER_A *
                FOC_MOTOR_TO_OUTPUT_GEAR_RATIO * 1000.0f));
  printf("FOC impedance law: Iq=K*(target-output_position)-D*output_speed; integral=0, disturbance_boost=0, Iq_limit=%ld mA\r\n",
         (long)(FOC_IMPEDANCE_IQ_LIMIT_A * 1000.0f));
  if (g_foc_fault == 13U)
  {
    printf("FOC stopped: impedance current did not settle to zero before timeout\r\n");
  }
#else
#if FOC_COMPOSITE_DEMO_ENABLE || FOC_PRE_POSITION_IMPEDANCE_ENABLE
  printf("FOC impedance result: completed=%u, duration=%lu ms, target=%ld mdeg output, controller_updates=%lu, Iq_saturated=%lu, max_abs_error=%ld mdeg, peak_abs_Iq=%ld mA => %ld mNm ideal output torque\r\n",
         (unsigned int)impedance_demo_completed,
         (unsigned long)FOC_COMPOSITE_IMPEDANCE_DURATION_MS,
         (long)(FOC_IMPEDANCE_TARGET_OUTPUT_DEG * 1000.0f),
         (unsigned long)impedance_control_updates,
         (unsigned long)impedance_iq_saturated_updates,
         (long)(impedance_max_abs_position_error_deg * 1000.0f),
         (long)(impedance_peak_abs_iq_a * 1000.0f),
         (long)(impedance_peak_abs_iq_a *
                FOC_MOTOR_ESTIMATED_KT_NM_PER_A *
                FOC_MOTOR_TO_OUTPUT_GEAR_RATIO * 1000.0f));
  printf("FOC impedance law: Iq=K*(target-output_position)-D*output_speed; integral=0, disturbance_boost=0, Iq_limit=%ld mA\r\n",
         (long)(FOC_IMPEDANCE_IQ_LIMIT_A * 1000.0f));
#endif
#if !FOC_TORQUE_ONLY_DEMO_ENABLE
#if !FOC_VELOCITY_HEAT_TEST_ENABLE
  printf("FOC position controller updates=%lu, boost active=%lu, Iq saturated=%lu; max tracking error=%ld mdeg, max boost=%ld mA\r\n",
         (unsigned long)position_control_updates,
         (unsigned long)position_boost_updates,
         (unsigned long)position_iq_saturated_updates,
         (long)(position_max_abs_tracking_error_deg * 1000.0f),
         (long)(position_max_abs_boost_a * 1000.0f));
  printf("FOC output position result: completed=%u, step=%u/%u, target=%ld mdeg, output=%ld mdeg, motor=%ld mdeg\r\n",
         (unsigned int)position_demo_completed,
         (unsigned int)(position_target_index + 1U),
         (unsigned int)position_target_count,
         (long)(g_foc_output_position_target_deg * 1000.0f),
         (long)((position_demo_completed != 0U) ?
                    (position_demo_final_output_deg * 1000.0f) :
                    ((((float)g_foc_mechanical_position_counts * 360000.0f) /
                      16384.0f) * FOC_OUTPUT_DIRECTION_SIGN /
                     FOC_MOTOR_TO_OUTPUT_GEAR_RATIO)),
         (long)((position_demo_completed != 0U) ?
                    (position_demo_final_motor_deg * 1000.0f) :
                    (((float)g_foc_mechanical_position_counts * 360000.0f) /
                     16384.0f)));
#endif
#if !FOC_FORCE_SCALE_TEST_ENABLE
  printf("FOC velocity result: started=%u at %lu ms, completed=%u, target=%ld mRPM output (%ld rpm motor), final_reference=%ld mRPM, final_speed=%ld mRPM, peak_speed=%ld mRPM\r\n",
         (unsigned int)velocity_demo_started,
         (unsigned long)velocity_stage_start_elapsed_ms,
         (unsigned int)velocity_demo_completed,
         (long)(FOC_ACTIVE_VELOCITY_TARGET_OUTPUT_RPM * 1000.0f),
         (long)(FOC_ACTIVE_VELOCITY_TARGET_OUTPUT_RPM *
                FOC_MOTOR_TO_OUTPUT_GEAR_RATIO),
         (long)(velocity_output_reference_rpm * 1000.0f),
         (long)(velocity_final_output_rpm * 1000.0f),
         (long)(velocity_peak_abs_output_rpm * 1000.0f));
  printf("FOC velocity controller updates=%lu, Iq saturated=%lu; max output-speed error=%ld mRPM, final integral=%ld mA\r\n",
         (unsigned long)velocity_control_updates,
         (unsigned long)velocity_iq_saturated_updates,
         (long)(velocity_max_abs_error_output_rpm * 1000.0f),
         (long)(velocity_integrator_a * 1000.0f));
#endif
#endif
#if !FOC_COMPOSITE_DEMO_ENABLE && !FOC_VELOCITY_HEAT_TEST_ENABLE
#if FOC_FORCE_SCALE_TEST_ENABLE
  printf("FOC force-scale contact: detected=%u at %lu ms, output=%ld mdeg\r\n",
         (unsigned int)force_contact_detected,
         (unsigned long)force_contact_elapsed_ms,
         (long)(force_contact_output_deg * 1000.0f));
  printf("FOC velocity stage intentionally skipped for force-scale mode: started=%u, start=%lu ms, completed=%u, final_speed=%ld mRPM\r\n",
         (unsigned int)velocity_demo_started,
         (unsigned long)velocity_stage_start_elapsed_ms,
         (unsigned int)velocity_demo_completed,
         (long)(velocity_final_output_rpm * 1000.0f));
  printf("FOC force-scale result: started=%u at %lu ms, completed=%u, target_Iq=%ld mA, measured_peak_abs_Iq=%ld mA, ideal_output_torque=%ld mNm, peak_abs_motor_speed=%ld rpm, final_speed=%ld rpm\r\n",
         (unsigned int)torque_demo_started,
         (unsigned long)torque_stage_start_elapsed_ms,
         (unsigned int)torque_demo_completed,
         (long)(FOC_FORCE_TEST_TARGET_IQ_A * 1000.0f),
         (long)(torque_peak_abs_iq_a * 1000.0f),
         (long)(torque_peak_abs_iq_a *
                FOC_MOTOR_ESTIMATED_KT_NM_PER_A *
                FOC_MOTOR_TO_OUTPUT_GEAR_RATIO * 1000.0f),
         (long)torque_peak_abs_motor_rpm,
         (long)mechanical_rpm_filtered);
  printf("FOC force-scale law: contact-gated one-direction Iq ramp, no position/velocity outer loop during loading; assumed lever arm=%ld mm\r\n",
         (long)FOC_FORCE_TEST_LEVER_ARM_MM);
#else
  printf("FOC torque result: started=%u at %lu ms, completed=%u, command_peak=%ld mNm motor (%ld mNm ideal output), measured_peak_abs_Iq=%ld mA => %ld mNm motor (%ld mNm ideal output), peak_abs_motor_speed=%ld rpm, final_speed=%ld rpm\r\n",
         (unsigned int)torque_demo_started,
         (unsigned long)torque_stage_start_elapsed_ms,
         (unsigned int)torque_demo_completed,
         (long)(FOC_TORQUE_TARGET_MOTOR_NM * 1000.0f),
         (long)(FOC_TORQUE_TARGET_MOTOR_NM *
                FOC_MOTOR_TO_OUTPUT_GEAR_RATIO * 1000.0f),
         (long)(torque_peak_abs_iq_a * 1000.0f),
         (long)(torque_peak_abs_iq_a *
                FOC_MOTOR_ESTIMATED_KT_NM_PER_A * 1000.0f),
         (long)(torque_peak_abs_iq_a *
                FOC_MOTOR_ESTIMATED_KT_NM_PER_A *
                FOC_MOTOR_TO_OUTPUT_GEAR_RATIO * 1000.0f),
         (long)torque_peak_abs_motor_rpm,
         (long)mechanical_rpm_filtered);
  printf("FOC torque controller updates=%lu; Iq*=torque/Kt, Id*=0, with no position/velocity outer-loop integral\r\n",
         (unsigned long)torque_control_updates);
#endif
#endif
  if (g_foc_fault == 10U)
  {
    printf("FOC stopped: position sequence did not settle before the step or overall timeout\r\n");
  }
  else if (g_foc_fault == 11U)
  {
    printf("FOC stopped: velocity test did not decelerate inside the output-speed tolerance before timeout\r\n");
  }
  else if (g_foc_fault == 12U)
  {
    printf("FOC stopped: Kt-based torque command did not return to zero before timeout\r\n");
  }
  else if (g_foc_fault == 13U)
  {
    printf("FOC stopped: pre-position impedance stage did not complete before timeout\r\n");
  }
#endif
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
#if FOC_COMPOSITE_DEMO_ENABLE || FOC_VELOCITY_HEAT_TEST_ENABLE
  printf("FOC CSV dump omitted for this demonstration; summary telemetry is shown above\r\n");
#else
  FOC_LogDump();
#endif

foc_stop:
  g_foc_enabled = 0U;
  g_foc_id_reference_a = 0.0f;
  g_foc_iq_reference_a = 0.0f;
  g_foc_speed_reference_rpm = 0.0f;
  Set_DQ_SVPWM(g_foc_electrical_angle_rad, 0.0f, 0.0f);
  Motor_PWM_Off();
  Debug_Status2_Set(GPIO_PIN_RESET);
  (void)HAL_ADCEx_InjectedStop_IT(&hadc1);
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

static bool BLDC_SixStep_FiniteDemo(void)
{
  uint8_t step = 1U;
  uint32_t start_tick;
  uint32_t last_tick;
  uint32_t alignment_start_tick;
  uint32_t timer_period_ticks;
  uint16_t alignment_duty_ticks;

  printf("Stage 5/5: six-step open-loop velocity for %lu ms after %lu ms low-duty alignment; target=%ld mHz, ramp=%ld mHz/s, duty=%ld%% to %ld%%\r\n",
         (unsigned long)MOTOR_SIX_STEP_RUN_TIME_MS,
         (unsigned long)MOTOR_SIX_STEP_ALIGNMENT_HOLD_MS,
         (long)(MOTOR_DEMO_OPEN_LOOP_TARGET_EHZ * 1000.0f),
         (long)(MOTOR_DEMO_OPEN_LOOP_RAMP_EHZ_PER_SEC * 1000.0f),
         (long)(MOTOR_SIX_STEP_DUTY_START_FRACTION * 100.0f),
         (long)(MOTOR_SIX_STEP_DUTY_END_FRACTION * 100.0f));

  TIM1_Set_EdgeAligned_For_OpenLoopPWM();
  timer_period_ticks = __HAL_TIM_GET_AUTORELOAD(&htim1);
  alignment_duty_ticks = Clamp_PWM_Ticks((uint32_t)(
      MOTOR_SIX_STEP_ALIGNMENT_DUTY_FRACTION *
      (float)timer_period_ticks));
  Set_PWM_DutyTicks(alignment_duty_ticks);
  g_target_electrical_hz = 0.0f;
  g_actual_electrical_hz = 0.0f;

  /* Lock the rotor to sector zero at a voltage comparable to the proven
   * SVPWM alignment level.  The first moving sector is then deterministic
   * instead of applying a large vector to an arbitrary rotor angle. */
  CommutateStep(0U, g_pwm_duty_ticks);
  alignment_start_tick = HAL_GetTick();
  while ((HAL_GetTick() - alignment_start_tick) <
         MOTOR_SIX_STEP_ALIGNMENT_HOLD_MS)
  {
    if (Motor_Check_CurrentLimit() != 0U)
    {
      Motor_PWM_Off();
      printf("Six-step alignment stopped by phase-current limit\r\n");
      return false;
    }
    if (HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin) ==
        GPIO_PIN_RESET)
    {
      Motor_ReportDRVFaultAndShutdown("Six-step alignment");
      return false;
    }
    HAL_Delay(1U);
  }

  g_target_electrical_hz = MOTOR_DEMO_OPEN_LOOP_TARGET_EHZ;
  g_actual_electrical_hz = MOTOR_SIX_STEP_START_ELECTRICAL_HZ;
  start_tick = HAL_GetTick();
  last_tick = start_tick;

  while ((HAL_GetTick() - start_tick) < MOTOR_SIX_STEP_RUN_TIME_MS)
  {
    uint32_t now_tick = HAL_GetTick();
    uint32_t elapsed_ms = now_tick - start_tick;
    float dt_sec = (float)(now_tick - last_tick) / 1000.0f;
    float duty_progress = ClampFloat(
        (float)elapsed_ms / (float)MOTOR_SIX_STEP_RUN_TIME_MS,
        0.0f, 1.0f);
    float duty_fraction =
        MOTOR_SIX_STEP_DUTY_START_FRACTION +
        ((MOTOR_SIX_STEP_DUTY_END_FRACTION -
          MOTOR_SIX_STEP_DUTY_START_FRACTION) * duty_progress);

    last_tick = now_tick;
    g_actual_electrical_hz = RampToward(
        g_actual_electrical_hz,
        g_target_electrical_hz,
        MOTOR_DEMO_OPEN_LOOP_RAMP_EHZ_PER_SEC,
        dt_sec);
    Set_PWM_DutyTicks((uint16_t)(duty_fraction *
                                 (float)timer_period_ticks));
    CommutateStep(step++, g_pwm_duty_ticks);

    if (Motor_Check_CurrentLimit() != 0U)
    {
      Motor_PWM_Off();
      printf("Six-step stage stopped by phase-current limit\r\n");
      return false;
    }
    if (HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin) ==
        GPIO_PIN_RESET)
    {
      Motor_ReportDRVFaultAndShutdown("Six-step run");
      return false;
    }

    delay_us(ElectricalHz_ToStepDelayUs(g_actual_electrical_hz));
  }

  Motor_PWM_Off();
  printf("Stage 5/5 complete: final command=%ld mHz electrical (%ld mRPM motor), duty=%u/%lu; PWM OFF\r\n",
         (long)(g_actual_electrical_hz * 1000.0f),
         (long)((g_actual_electrical_hz * 60.0f /
                 (float)MOTOR_POLE_PAIRS) * 1000.0f),
         (unsigned int)g_pwm_duty_ticks,
         (unsigned long)timer_period_ticks);
  return true;
}

bool BLDC_OpenLoop_SVPWM_RampLoop(uint8_t deadtime_ticks, bool dump_log)
{
  float electrical_angle_rad = 0.0f;
  float electrical_hz = MOTOR_START_ELECTRICAL_HZ;
#if FOC_COMPOSITE_DEMO_ENABLE
  const float open_loop_target_ehz = MOTOR_DEMO_OPEN_LOOP_TARGET_EHZ;
  const float open_loop_ramp_ehz_per_sec =
      MOTOR_DEMO_OPEN_LOOP_RAMP_EHZ_PER_SEC;
#else
  const float open_loop_target_ehz = MOTOR_HIGH_ELECTRICAL_HZ;
  const float open_loop_ramp_ehz_per_sec = MOTOR_RAMP_EHZ_PER_SEC;
#endif
  uint32_t last_update_cycles = DWT->CYCCNT;
  uint32_t last_tick = HAL_GetTick();
  uint32_t start_tick = last_tick;
  uint32_t last_fault_poll_tick = last_tick;
  uint32_t last_current_poll_tick = last_tick;
#if CURRENT_LED_DIAGNOSTIC_ENABLE
  uint32_t last_current_led_tick = last_tick;
#endif
  const float update_period_sec = (float)MOTOR_SVPWM_UPDATE_US / 1000000.0f;
  const uint32_t update_period_cycles = (SystemCoreClock / 1000000U) * MOTOR_SVPWM_UPDATE_US;

  g_motor_deadtime_ticks = deadtime_ticks;
#if UART_OUTPUT_ENABLE
#if MOTOR_SVPWM_CONTINUOUS_RUN
  printf("Open-loop SVPWM probe demo: continuous run after %u ms rotor alignment; target=%ld mHz, ramp=%ld mHz/s, dead-time ticks=%u\r\n",
         (unsigned int)MOTOR_ALIGNMENT_HOLD_MS,
         (long)(open_loop_target_ehz * 1000.0f),
         (long)(open_loop_ramp_ehz_per_sec * 1000.0f),
         deadtime_ticks);
#else
  printf("Open-loop SVPWM demo: %lu ms run after %u ms rotor alignment; target=%ld mHz, ramp=%ld mHz/s, dead-time ticks=%u\r\n",
         (unsigned long)MOTOR_SVPWM_RUN_TIME_MS,
         (unsigned int)MOTOR_ALIGNMENT_HOLD_MS,
         (long)(open_loop_target_ehz * 1000.0f),
         (long)(open_loop_ramp_ehz_per_sec * 1000.0f),
         deadtime_ticks);
#endif
#endif
  g_target_electrical_hz = open_loop_target_ehz;
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
        (void)HAL_ADCEx_InjectedStop_IT(&hadc1);
        printf("SVPWM stopped by phase-current limit during rotor alignment\r\n");
        return false;
      }
      if (HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin) ==
          GPIO_PIN_RESET)
      {
        Set_OpenLoop_SVPWM(electrical_angle_rad, 0.0f);
        (void)HAL_ADCEx_InjectedStop_IT(&hadc1);
        Motor_ReportDRVFaultAndShutdown("SVPWM alignment");
        return false;
      }
      HAL_Delay(1U);
    }
  }

  /* Start the timed run only after alignment; keep the aligned electrical angle. */
  electrical_hz = MOTOR_START_ELECTRICAL_HZ;
  g_actual_electrical_hz = electrical_hz;
  g_target_electrical_hz = open_loop_target_ehz;
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

      g_target_electrical_hz = open_loop_target_ehz;

      electrical_hz = RampToward(electrical_hz,
                                 g_target_electrical_hz,
                                 open_loop_ramp_ehz_per_sec,
                                 ramp_dt_sec);

      g_actual_electrical_hz = electrical_hz;
      electrical_angle_rad = WrapRadians(electrical_angle_rad +
                                          (TWO_PI_F * electrical_hz * update_period_sec));
      modulation_index = OpenLoop_ModulationForElapsedMs(now_tick - start_tick);
      g_open_loop_modulation = modulation_index;

#if SVPWM_CURRENT_LOG_ENABLE
      if (dump_log && now_tick >= g_svpwm_log_start_tick)
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
          (void)HAL_ADCEx_InjectedStop_IT(&hadc1);
          printf("SVPWM stopped by phase-current limit\r\n");
          if (dump_log)
          {
            SVPWM_Log_Dump();
          }
          return false;
        }
        if (HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin) ==
            GPIO_PIN_RESET)
        {
          Set_OpenLoop_SVPWM(electrical_angle_rad, 0.0f);
          (void)HAL_ADCEx_InjectedStop_IT(&hadc1);
          Motor_ReportDRVFaultAndShutdown("SVPWM run");
          return false;
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

      if (dump_log && g_svpwm_log_count >= SVPWM_LOG_CAPACITY)
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
          (void)HAL_ADCEx_InjectedStop_IT(&hadc1);
          printf("SVPWM stopped by DRV8353S fault: status1=0x%03X, status2=0x%03X\r\n",
                 g_drv_fault_status1, g_drv_vgs_status2);
          if (dump_log)
          {
            SVPWM_Log_Dump();
          }
          return false;
        }
      }
#else
      (void)last_fault_poll_tick;
#endif

    }
  }

  Set_OpenLoop_SVPWM(electrical_angle_rad, 0.0f);
  Motor_PWM_Off();
  (void)HAL_ADCEx_InjectedStop_IT(&hadc1);
  if (dump_log)
  {
    printf("SVPWM waveform capture complete: %u samples at %u ms intervals\r\n",
           g_svpwm_log_count, SVPWM_LOG_INTERVAL_MS);
    SVPWM_Log_Dump();
  }
  else
  {
    printf("Open-loop SVPWM complete: final command=%ld mHz electrical (%ld mRPM motor); current-log capture disabled, PWM OFF\r\n",
           (long)(g_actual_electrical_hz * 1000.0f),
           (long)((g_actual_electrical_hz * 60.0f /
                   (float)MOTOR_POLE_PAIRS) * 1000.0f));
  }
  return true;
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

void CAN_Start(uint32_t filter_id_1, uint32_t filter_id_2)
{
  FDCAN_FilterTypeDef filter = {0};

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_DUAL;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = filter_id_1;
  filter.FilterID2 = filter_id_2;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  g_can_status_flags |= CAN_STATUS_CAN_STARTED;
}

uint8_t CAN_Tx(uint32_t id, const uint8_t *data, uint8_t len)
{
  FDCAN_TxHeaderTypeDef txHeader = {0};

  if (id > 0x7FFU || len > 8U ||
      HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
  {
    return 0U;
  }

  txHeader.Identifier = id;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = fdcan_dlc_from_len(len);
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader,
                                    (uint8_t *)data) != HAL_OK)
  {
    return 0U;
  }

  return 1U;
}

uint8_t CAN_Rx(uint32_t *id, uint8_t *data, uint8_t *len)
{
  FDCAN_RxHeaderTypeDef rxHeader = {0};

  if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0U)
  {
    return 0U;
  }

  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, data) != HAL_OK)
  {
    return 0U;
  }

  if (rxHeader.IdType != FDCAN_STANDARD_ID ||
      rxHeader.RxFrameType != FDCAN_DATA_FRAME)
  {
    return 0U;
  }

  *id = rxHeader.Identifier;
  *len = len_from_fdcan_dlc(rxHeader.DataLength);

  return 1U;
}

static void CAN_IncrementSaturated(uint8_t *value)
{
  if (*value < 0xFFU)
  {
    (*value)++;
  }
}

static void CAN_ForceMotorOutputsSafe(void)
{
  g_foc_enabled = 0U;
  g_foc_id_reference_a = 0.0f;
  g_foc_iq_reference_a = 0.0f;
  Motor_PWM_Off();
  HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
  g_can_status_flags &= (uint8_t)~CAN_STATUS_OUTPUT_ENABLED;
}

static void CAN_EnterFault(uint8_t fault)
{
  CAN_ForceMotorOutputsSafe();
  g_can_drive_fault = fault;
  g_can_drive_state = CAN_DRIVE_STATE_FAULT;
}

static void CAN_SendHeartbeat(void)
{
  uint8_t data[8];

  data[0] = CAN_PROTOCOL_VERSION;
  data[1] = (uint8_t)g_can_drive_state;
  data[2] = g_can_drive_fault;
  data[3] = g_can_status_flags;
  data[4] = g_can_last_command_sequence;
  data[5] = (uint8_t)g_can_last_command_result;
  data[6] = g_can_tx_drop_count;
  data[7] = g_can_invalid_rx_count;

  if (CAN_Tx(CAN_HEARTBEAT_ID, data, sizeof(data)) == 0U)
  {
    CAN_IncrementSaturated(&g_can_tx_drop_count);
  }
}

static void CAN_ProcessStateCommand(const uint8_t *data, uint8_t len,
                                    uint8_t local_fault)
{
  CAN_StateRequest_t request;

  g_can_status_flags |= CAN_STATUS_COMMAND_SEEN;

  if (len != 2U)
  {
    g_can_last_command_result = CAN_COMMAND_RESULT_INVALID_LENGTH;
    g_can_status_flags |= CAN_STATUS_INVALID_COMMAND;
    CAN_IncrementSaturated(&g_can_invalid_rx_count);
    return;
  }

  request = (CAN_StateRequest_t)data[0];
  g_can_last_command_sequence = data[1];

  switch (request)
  {
    case CAN_STATE_REQUEST_DISABLED:
      if (g_can_drive_state == CAN_DRIVE_STATE_FAULT)
      {
        g_can_last_command_result = CAN_COMMAND_RESULT_INVALID_TRANSITION;
        break;
      }
      CAN_ForceMotorOutputsSafe();
      g_can_drive_state = CAN_DRIVE_STATE_DISABLED;
      g_can_last_command_result = CAN_COMMAND_RESULT_ACCEPTED;
      break;

    case CAN_STATE_REQUEST_ARMED:
      if ((g_can_drive_state != CAN_DRIVE_STATE_DISABLED &&
           g_can_drive_state != CAN_DRIVE_STATE_ARMED) ||
          g_can_drive_fault != CAN_DRIVE_FAULT_NONE)
      {
        g_can_last_command_result = CAN_COMMAND_RESULT_INVALID_TRANSITION;
        break;
      }
      CAN_ForceMotorOutputsSafe();
      g_can_drive_state = CAN_DRIVE_STATE_ARMED;
      g_can_last_command_result = CAN_COMMAND_RESULT_ACCEPTED;
      break;

    case CAN_STATE_REQUEST_ACTIVE:
      if ((g_can_drive_state != CAN_DRIVE_STATE_ARMED &&
           g_can_drive_state != CAN_DRIVE_STATE_ACTIVE) ||
          g_can_drive_fault != CAN_DRIVE_FAULT_NONE)
      {
        g_can_last_command_result = CAN_COMMAND_RESULT_INVALID_TRANSITION;
        break;
      }
      /* Revision 1 implements the control plane only.  ACTIVE is observable,
       * but PWM and the gate driver remain inhibited until setpoint support is
       * added in the next protocol increment. */
      CAN_ForceMotorOutputsSafe();
      g_can_drive_state = CAN_DRIVE_STATE_ACTIVE;
      g_can_last_command_result = CAN_COMMAND_RESULT_ACCEPTED;
      break;

    case CAN_STATE_REQUEST_CLEAR_FAULT:
      if (local_fault != 0U)
      {
        g_can_last_command_result = CAN_COMMAND_RESULT_FAULT_STILL_PRESENT;
        break;
      }
      CAN_ForceMotorOutputsSafe();
      g_can_drive_fault = CAN_DRIVE_FAULT_NONE;
      g_can_drive_state = CAN_DRIVE_STATE_DISABLED;
      g_can_status_flags &= (uint8_t)~CAN_STATUS_ESTOP_LATCHED;
      g_can_last_command_result = CAN_COMMAND_RESULT_ACCEPTED;
      break;

    default:
      g_can_last_command_result = CAN_COMMAND_RESULT_INVALID_REQUEST;
      break;
  }

  if (g_can_last_command_result == CAN_COMMAND_RESULT_ACCEPTED)
  {
    g_can_status_flags &= (uint8_t)~CAN_STATUS_INVALID_COMMAND;
  }
  else
  {
    g_can_status_flags |= CAN_STATUS_INVALID_COMMAND;
    CAN_IncrementSaturated(&g_can_invalid_rx_count);
  }
}

static void CAN_ProtocolInit(void)
{
  CAN_ForceMotorOutputsSafe();
  g_can_drive_state = CAN_DRIVE_STATE_DISABLED;
  g_can_drive_fault = CAN_DRIVE_FAULT_NONE;
  g_can_last_command_result = CAN_COMMAND_RESULT_NONE;
  CAN_Start(CAN_GLOBAL_ESTOP_ID, CAN_STATE_COMMAND_ID);
  g_can_last_heartbeat_tick = HAL_GetTick() - CAN_HEARTBEAT_PERIOD_MS;
}

static void CAN_ProtocolPoll(uint8_t local_fault)
{
  uint32_t id;
  uint8_t data[8];
  uint8_t len;
  uint32_t now_tick;

  if (local_fault != 0U &&
      g_can_drive_fault < CAN_DRIVE_FAULT_LOCAL_BASE)
  {
    CAN_EnterFault((uint8_t)(CAN_DRIVE_FAULT_LOCAL_BASE |
                            (local_fault & 0x7FU)));
  }

  while (CAN_Rx(&id, data, &len) != 0U)
  {
    if (id == CAN_GLOBAL_ESTOP_ID)
    {
      CAN_EnterFault(CAN_DRIVE_FAULT_ESTOP);
      g_can_status_flags |= (CAN_STATUS_COMMAND_SEEN |
                             CAN_STATUS_ESTOP_LATCHED);
      g_can_last_command_result = CAN_COMMAND_RESULT_ESTOP;
    }
    else if (id == CAN_STATE_COMMAND_ID)
    {
      CAN_ProcessStateCommand(data, len, local_fault);
    }
    else
    {
      CAN_IncrementSaturated(&g_can_invalid_rx_count);
    }
  }

  now_tick = HAL_GetTick();
  if ((now_tick - g_can_last_heartbeat_tick) >= CAN_HEARTBEAT_PERIOD_MS)
  {
    g_can_last_heartbeat_tick = now_tick;
    CAN_SendHeartbeat();
  }
}

#if CAN_DEMO_DEVICE2_ENABLE
static void CAN_DemoPrintHeartbeat(const uint8_t *data, uint32_t now_tick)
{
  if (data[1] != g_can_demo_last_state ||
      data[2] != g_can_demo_last_fault ||
      data[4] != g_can_demo_last_sequence ||
      data[5] != g_can_demo_last_result ||
      (now_tick - g_can_demo_last_print_tick) >= 1000U)
  {
    printf("Device 2 RX heartbeat: protocol=%u state=%u fault=%u flags=0x%02X sequence=%u result=%u tx_drop=%u invalid_rx=%u\r\n",
           data[0], data[1], data[2], data[3], data[4], data[5], data[6],
           data[7]);
    g_can_demo_last_state = data[1];
    g_can_demo_last_fault = data[2];
    g_can_demo_last_sequence = data[4];
    g_can_demo_last_result = data[5];
    g_can_demo_last_print_tick = now_tick;
  }
}

static uint8_t CAN_DemoHeartbeatMatchesStep(const uint8_t *data,
                                            const CAN_DemoStep_t *step)
{
  if (data[1] != step->expected_state ||
      data[2] != step->expected_fault ||
      data[5] != step->expected_result ||
      (data[3] & CAN_STATUS_OUTPUT_ENABLED) != 0U)
  {
    return 0U;
  }

  if (step->expected_sequence != 0xFFU &&
      data[4] != step->expected_sequence)
  {
    return 0U;
  }

  return 1U;
}

static void CAN_DemoSendCurrentStep(uint32_t now_tick)
{
  const CAN_DemoStep_t *step;

  if (g_can_demo_step_index >=
      (uint8_t)(sizeof(g_can_demo_steps) / sizeof(g_can_demo_steps[0])))
  {
    return;
  }

  step = &g_can_demo_steps[g_can_demo_step_index];
  if (CAN_Tx(step->id, step->data, step->length) == 0U)
  {
    return;
  }

  g_can_demo_retry_count++;
  g_can_demo_waiting_for_ack = 1U;
  g_can_demo_last_send_tick = now_tick;
  if (step->length == 0U)
  {
    printf("Device 2 TX %s: id=0x%03lX, empty payload (attempt %u/3)\r\n",
           step->name, (unsigned long)step->id, g_can_demo_retry_count);
  }
  else
  {
    printf("Device 2 TX %s: id=0x%03lX data=%02X %02X (attempt %u/3)\r\n",
           step->name, (unsigned long)step->id, step->data[0], step->data[1],
           g_can_demo_retry_count);
  }
}

static void CAN_DemoControllerInit(void)
{
  CAN_ForceMotorOutputsSafe();
  CAN_Start(CAN_HEARTBEAT_ID, CAN_HEARTBEAT_ID);
  printf("Device 2 CAN command demo ready; waiting for device 1 heartbeat 0x%03X\r\n",
         CAN_HEARTBEAT_ID);
}

static void CAN_DemoControllerPoll(void)
{
  uint32_t id;
  uint8_t data[8];
  uint8_t len;
  uint32_t now_tick = HAL_GetTick();

  while (CAN_Rx(&id, data, &len) != 0U)
  {
    if (id != CAN_HEARTBEAT_ID || len != 8U)
    {
      CAN_IncrementSaturated(&g_can_invalid_rx_count);
      continue;
    }

    g_can_demo_seen_heartbeat = 1U;
    g_can_demo_last_rx_tick = now_tick;
    CAN_DemoPrintHeartbeat(data, now_tick);

    if (data[0] != CAN_PROTOCOL_VERSION)
    {
      if (g_can_demo_error == 0U)
      {
        printf("Device 2 ERROR: expected protocol %u, received %u\r\n",
               CAN_PROTOCOL_VERSION, data[0]);
      }
      g_can_demo_error = 1U;
      continue;
    }

    if (g_can_demo_started == 0U &&
        data[1] == CAN_DRIVE_STATE_DISABLED &&
        data[2] == CAN_DRIVE_FAULT_NONE &&
        (data[3] & CAN_STATUS_CONTROL_PLANE_ONLY) != 0U &&
        (data[3] & CAN_STATUS_OUTPUT_ENABLED) == 0U)
    {
      g_can_demo_started = 1U;
      g_can_demo_next_send_tick = now_tick + 500U;
      printf("Device 2 validated safe DISABLED heartbeat; demo begins in 500 ms\r\n");
    }

    if (g_can_demo_waiting_for_ack != 0U &&
        CAN_DemoHeartbeatMatchesStep(
            data, &g_can_demo_steps[g_can_demo_step_index]) != 0U)
    {
      const CAN_DemoStep_t *completed_step =
          &g_can_demo_steps[g_can_demo_step_index];
      printf("Device 2 confirmed %s from heartbeat\r\n",
             completed_step->name);
      g_can_demo_waiting_for_ack = 0U;
      g_can_demo_retry_count = 0U;
      g_can_demo_step_index++;

      if (g_can_demo_step_index >=
          (uint8_t)(sizeof(g_can_demo_steps) /
                    sizeof(g_can_demo_steps[0])))
      {
        g_can_demo_complete = 1U;
        printf("Device 2 CAN demo PASSED: all state and fault transitions confirmed; both motor outputs remain inhibited\r\n");
      }
      else
      {
        g_can_demo_next_send_tick =
            now_tick + completed_step->delay_after_ack_ms;
      }
    }
  }

  if (g_can_demo_error != 0U || g_can_demo_complete != 0U)
  {
    return;
  }

  if (g_can_demo_started == 0U)
  {
    if ((now_tick - g_can_demo_last_print_tick) >= 1000U)
    {
      printf("Device 2 waiting for a valid DISABLED heartbeat from device 1\r\n");
      g_can_demo_last_print_tick = now_tick;
    }
    return;
  }

  if (g_can_demo_seen_heartbeat != 0U &&
      (now_tick - g_can_demo_last_rx_tick) >= 1000U &&
      (now_tick - g_can_demo_last_print_tick) >= 1000U)
  {
    printf("Device 2 warning: no heartbeat received for 1 second\r\n");
    g_can_demo_last_print_tick = now_tick;
  }

  if (g_can_demo_waiting_for_ack != 0U)
  {
    if ((now_tick - g_can_demo_last_send_tick) >= 500U)
    {
      if (g_can_demo_retry_count >= 3U)
      {
        printf("Device 2 ERROR: no matching acknowledgement for %s after 3 attempts\r\n",
               g_can_demo_steps[g_can_demo_step_index].name);
        g_can_demo_error = 1U;
      }
      else
      {
        g_can_demo_waiting_for_ack = 0U;
        CAN_DemoSendCurrentStep(now_tick);
      }
    }
  }
  else if ((int32_t)(now_tick - g_can_demo_next_send_tick) >= 0)
  {
    CAN_DemoSendCurrentStep(now_tick);
  }
}
#endif

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

#if CAN_CONTROL_ENABLE
  CAN_ProtocolInit();
#if UART_OUTPUT_ENABLE
  printf("CAN control-plane revision %u ready at 1 Mbit/s; node=%u\r\n",
         CAN_PROTOCOL_VERSION, CAN_NODE_ID);
  printf("CAN IDs: E-stop=0x%03X, heartbeat=0x%03X, state command=0x%03X\r\n",
         CAN_GLOBAL_ESTOP_ID, CAN_HEARTBEAT_ID, CAN_STATE_COMMAND_ID);
  printf("Motor outputs remain inhibited in this first CAN increment\r\n");
#endif
 #elif CAN_DEMO_DEVICE2_ENABLE
  CAN_DemoControllerInit();
#if UART_OUTPUT_ENABLE
  printf("Firmware role: device 2 CAN sender/test controller\r\n");
  printf("Device 2 PWM and gate-driver outputs are inhibited\r\n");
#endif
#else
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
#if WILL_SVPWM_EXPERIMENT_ENABLE
  printf("Selected test: Will W_SVPWM implementation\r\n");
  printf("Probe target: %d electrical Hz, %d ms, modulation=%ld/1000\r\n",
         WILL_SVPWM_PROBE_ELECTRICAL_HZ,
         WILL_SVPWM_PROBE_DURATION_MS,
         (long)(WILL_SVPWM_PROBE_MODULATION_STRENGTH * 1000.0f));
  printf("Startup: existing velocity ramp copied to the monotonic-counter scheduler; constant modulation\r\n");
  printf("Counter backlog statistics print after PWM shutdown\r\n");
  printf("Dead time=%u ticks (approximately %lu ns before DRV-added timing)\r\n",
         (unsigned int)MOTOR_TIM1_DEADTIME_TICKS,
         (unsigned long)(((1000000000ULL * MOTOR_TIM1_DEADTIME_TICKS) +
                          (Get_TIM1_ClockHz() / 2U)) /
                         Get_TIM1_ClockHz()));
  printf("The DRV nFAULT pin is monitored, but this experimental path does not use the software phase-current cutoff\r\n");
#elif MOTOR_FIELD_STEP_DEMO_ENABLE
  printf("Selected test: stepped static SVPWM field orientation\r\n");
  printf("Keep the shaft unloaded; each point is a two-second static hold\r\n");
#elif MOTOR_FOC_DEMO_ENABLE
  printf("The shaft must be unloaded and free to rotate during automatic alignment\r\n");
#if FOC_POSITION_DEMO_ENABLE
#if FOC_VELOCITY_HEAT_TEST_ENABLE
  printf("Selected test: FOC velocity-only motor thermal run\r\n");
  printf("Profile: ramp to %ld motor rpm (%ld mRPM output), hold for approximately %lu seconds, then ramp to zero\r\n",
         (long)FOC_HEAT_TEST_TARGET_MOTOR_RPM,
         (long)(FOC_HEAT_TEST_TARGET_OUTPUT_RPM * 1000.0f),
         (unsigned long)(FOC_HEAT_TEST_HOLD_MS / 1000U));
  printf("Timing: deceleration at %lu ms, active profile=%lu ms, settle allowance=%lu ms; overspeed threshold=%ld motor rpm\r\n",
         (unsigned long)FOC_HEAT_TEST_DECEL_START_MS,
         (unsigned long)FOC_HEAT_TEST_DURATION_MS,
         (unsigned long)FOC_HEAT_TEST_SETTLE_TIMEOUT_MS,
         (long)FOC_HEAT_TEST_OVERSPEED_MOTOR_RPM);
  printf("After alignment there is a %lu-second PWM-off safety countdown; use the heat gun without approaching the rotating shaft\r\n",
         (unsigned long)FOC_HEAT_TEST_ARMING_PAUSE_SEC);
  printf("STATUS_2 turns on for the 60-second target-speed measurement window and turns off before deceleration\r\n");
#elif FOC_FORCE_SCALE_TEST_ENABLE
  printf("Selected test: contact-gated CCW force-scale loading\r\n");
  printf("Profile: approach toward %ld mdeg at no more than %ld mRPM output, detect rigid contact, then ramp to %ld mA Iq and hold for %lu ms\r\n",
         (long)(FOC_FORCE_TEST_CCW_SIGN *
                FOC_FORCE_TEST_CONTACT_OUTPUT_DEG * 1000.0f),
         (long)(FOC_FORCE_TEST_APPROACH_MAX_OUTPUT_RPM * 1000.0f),
         (long)(FOC_FORCE_TEST_TARGET_IQ_A * 1000.0f),
         (unsigned long)FOC_FORCE_TEST_TORQUE_HOLD_MS);
  printf("Configured CCW direction sign=%ld/1000 when facing the motor front; PWM-off arming pause=%lu seconds\r\n",
         (long)(FOC_FORCE_TEST_CCW_SIGN * 1000.0f),
         (unsigned long)FOC_FORCE_TEST_ARMING_PAUSE_SEC);
#elif FOC_PRE_POSITION_IMPEDANCE_ENABLE
  printf("Selected tests: fixed-position impedance -> position PID -> velocity PI -> Kt-based torque control\r\n");
  printf("Impedance: target=%ld mdeg, stiffness=%ld mNm/output-deg for %lu seconds; then the existing position sequence begins\r\n",
         (long)(FOC_IMPEDANCE_TARGET_OUTPUT_DEG * 1000.0f),
         (long)(FOC_IMPEDANCE_STIFFNESS_NM_PER_OUTPUT_DEG * 1000.0f),
         (unsigned long)(FOC_COMPOSITE_IMPEDANCE_DURATION_MS / 1000U));
  printf("After alignment there is a %lu-second PWM-off arming pause before impedance control\r\n",
         (unsigned long)FOC_IMPEDANCE_ARMING_PAUSE_SEC);
#elif FOC_COMPOSITE_DEMO_ENABLE
  printf("Selected five-stage demo: fixed-position impedance -> FOC position -> FOC velocity -> open-loop SVPWM velocity -> open-loop six-step velocity\r\n");
  printf("Durations: impedance=%lu s, position allowance=%lu s, FOC velocity=%lu s, SVPWM=%lu s plus alignment, six-step=%lu s\r\n",
         (unsigned long)(FOC_COMPOSITE_IMPEDANCE_DURATION_MS / 1000U),
         (unsigned long)(FOC_POSITION_TEST_DURATION_MS / 1000U),
         (unsigned long)(FOC_VELOCITY_DEMO_DURATION_MS / 1000U),
         (unsigned long)(MOTOR_SVPWM_RUN_TIME_MS / 1000U),
         (unsigned long)(MOTOR_SIX_STEP_RUN_TIME_MS / 1000U));
  printf("After alignment there is a %lu-second PWM-off arming pause before the impedance stage\r\n",
         (unsigned long)FOC_IMPEDANCE_ARMING_PAUSE_SEC);
#elif FOC_IMPEDANCE_DEMO_ENABLE
  printf("Selected test: fixed-angle single-stiffness impedance control only\r\n");
  printf("FOC output target=%ld mdeg; stiffness=%ld mNm/output-deg for %lu seconds, followed by a %lu-second smooth release\r\n",
         (long)(FOC_IMPEDANCE_TARGET_OUTPUT_DEG * 1000.0f),
         (long)(FOC_IMPEDANCE_STIFFNESS_NM_PER_OUTPUT_DEG * 1000.0f),
         (unsigned long)(FOC_IMPEDANCE_DEMO_DURATION_MS / 1000U),
         (unsigned long)(FOC_IMPEDANCE_RELEASE_RAMP_MS / 1000U));
  printf("FOC impedance has no integral: displacement is proportional to applied torque; Iq clamp=%ld mA, motor overspeed=%ld rpm\r\n",
         (long)(FOC_IMPEDANCE_IQ_LIMIT_A * 1000.0f),
         (long)FOC_IMPEDANCE_OVERSPEED_MOTOR_RPM);
  printf("After alignment there is a %lu-second PWM-off arming pause; keep clear and apply force with a gauge/fixture, not by hand\r\n",
         (unsigned long)FOC_IMPEDANCE_ARMING_PAUSE_SEC);
#elif FOC_TORQUE_ONLY_DEMO_ENABLE
  printf("Selected test: Kt-based torque control only; position and velocity demos are skipped\r\n");
  printf("After alignment there is a %lu-second PWM-off pause to secure/load the shaft for the +%ld/0/-%ld/0 mNm motor-torque sequence\r\n",
         (unsigned long)FOC_TORQUE_LOAD_PAUSE_SEC,
         (long)(FOC_TORQUE_TARGET_MOTOR_NM * 1000.0f),
         (long)(FOC_TORQUE_TARGET_MOTOR_NM * 1000.0f));
  printf("FOC torque target=%ld mA Iq, Id=0 mA, ideal 11:1 output=%ld mNm before losses; overspeed=%ld motor rpm\r\n",
         (long)((FOC_TORQUE_TARGET_MOTOR_NM /
                 FOC_MOTOR_ESTIMATED_KT_NM_PER_A) * 1000.0f),
         (long)(FOC_TORQUE_TARGET_MOTOR_NM *
                FOC_MOTOR_TO_OUTPUT_GEAR_RATIO * 1000.0f),
         (long)FOC_TORQUE_OVERSPEED_MOTOR_RPM);
#else
  printf("Selected tests: geared position PID, ramped velocity PI, then Kt-based torque control\r\n");
  printf("FOC AS5048A=motor-side, gearbox=%ld/1000 motor rev/output rev, direction=%ld\r\n",
         (long)(FOC_MOTOR_TO_OUTPUT_GEAR_RATIO * 1000.0f),
         (long)FOC_OUTPUT_DIRECTION_SIGN);
  printf("FOC output-frame position P=%ld/1000 A/output-deg, I=%ld/1000 A/(output-deg*s), D=%ld/1000 A/output-rpm\r\n",
         (long)(FOC_POSITION_KP_A_PER_OUTPUT_DEG * 1000.0f),
         (long)(FOC_POSITION_KI_A_PER_OUTPUT_DEG_S * 1000.0f),
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
  printf("FOC velocity stage: 0 -> %ld mRPM output (%ld rpm motor) -> 0 over %lu ms; accel/decel=%ld/%ld mRPM/s\r\n",
         (long)(FOC_VELOCITY_TARGET_OUTPUT_RPM * 1000.0f),
         (long)(FOC_VELOCITY_TARGET_OUTPUT_RPM *
                FOC_MOTOR_TO_OUTPUT_GEAR_RATIO),
         (unsigned long)FOC_VELOCITY_DEMO_DURATION_MS,
         (long)(FOC_VELOCITY_ACCEL_OUTPUT_RPM_S * 1000.0f),
         (long)(FOC_VELOCITY_DECEL_OUTPUT_RPM_S * 1000.0f));
  printf("FOC torque stage: motor torque 0 -> +%ld -> 0 -> -%ld -> 0 mNm over %lu ms (%ld mA peak Iq); load fixture required, overspeed=%ld motor rpm\r\n",
         (long)(FOC_TORQUE_TARGET_MOTOR_NM * 1000.0f),
         (long)(FOC_TORQUE_TARGET_MOTOR_NM * 1000.0f),
         (unsigned long)FOC_TORQUE_DEMO_DURATION_MS,
         (long)((FOC_TORQUE_TARGET_MOTOR_NM /
                 FOC_MOTOR_ESTIMATED_KT_NM_PER_A) * 1000.0f),
         (long)FOC_TORQUE_OVERSPEED_MOTOR_RPM);
#endif
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
  printf("Continuous open-loop SVPWM probe demo, approximately 500 ns dead time\r\n");
  printf("Startup: 1 s fixed-angle alignment, then 0.5 eHz toward 500 eHz at 2 eHz/s\r\n");
  printf("Modulation: 0.020 to 0.040 over 4 s, then to 0.100 by %lu s\r\n",
         (unsigned long)(MOTOR_SVPWM_RUN_TIME_MS / 1000U));
  printf("Probe mode runs until reset or a detected fault; current-log dump is disabled\r\n");
#if MOTOR_PHASE_CURRENT_LIMIT_ENABLE
  printf("Synchronized software phase-current cutoff: %.1f A\r\n",
         MOTOR_PHASE_CURRENT_LIMIT_A);
#else
  printf("WARNING: software phase-current cutoff is disabled\r\n");
#endif
  printf("DRV8353S SPI used only for verified CSA calibration; full configuration unchanged\r\n");
#endif
#endif
#if WILL_SVPWM_EXPERIMENT_ENABLE
  Will_SVPWM_Implementation(WILL_SVPWM_PROBE_ELECTRICAL_HZ,
                            WILL_SVPWM_PROBE_DURATION_MS);
  printf("W_SVPWM probe window complete; PWM remains inhibited\r\n");
#elif MOTOR_FIELD_STEP_DEMO_ENABLE
  Motor_FieldOrientationDemo(MOTOR_DEADTIME_500NS_TICKS);
  printf("Field-orientation demonstration finished; PWM remains inhibited\r\n");
#elif MOTOR_FOC_DEMO_ENABLE
#if FOC_COMPOSITE_DEMO_ENABLE
  Motor_FOC_Demo(MOTOR_DEADTIME_500NS_TICKS);
  if (g_foc_fault == 0U)
  {
    printf("Stages 1-3 complete; PWM OFF for %lu ms before open-loop SVPWM\r\n",
           (unsigned long)MOTOR_DEMO_HANDOFF_PAUSE_MS);
    HAL_Delay(MOTOR_DEMO_HANDOFF_PAUSE_MS);
    if (BLDC_OpenLoop_SVPWM_RampLoop(
            MOTOR_DEADTIME_500NS_TICKS, false))
    {
      printf("PWM OFF for %lu ms before six-step alignment\r\n",
             (unsigned long)MOTOR_SIX_STEP_HANDOFF_PAUSE_MS);
      HAL_Delay(MOTOR_SIX_STEP_HANDOFF_PAUSE_MS);
      if (BLDC_SixStep_FiniteDemo())
      {
        printf("All five control demonstrations completed; PWM remains inhibited\r\n");
      }
      else
      {
        printf("Composite demo aborted during six-step commutation; PWM remains inhibited\r\n");
      }
    }
    else
    {
      printf("Composite demo aborted during open-loop SVPWM; six-step stage skipped\r\n");
    }
  }
  else
  {
    printf("Composite demo aborted by FOC fault=%u; open-loop stages skipped and PWM remains inhibited\r\n",
           g_foc_fault);
  }
#elif FOC_VELOCITY_HEAT_TEST_ENABLE
  Motor_FOC_Demo(MOTOR_DEADTIME_500NS_TICKS);
  printf("FOC velocity thermal test finished; PWM remains inhibited\r\n");
#else
  Motor_FOC_Demo(MOTOR_DEADTIME_500NS_TICKS);
  printf("FOC bring-up demonstration finished; PWM remains inhibited\r\n");
#endif
#else
  if (!BLDC_OpenLoop_SVPWM_RampLoop(MOTOR_DEADTIME_500NS_TICKS, false))
  {
    printf("SVPWM probe demo stopped by a protection or driver fault; PWM remains inhibited\r\n");
  }
#endif
#endif /* MOTOR_ENCODER_TEST_ENABLE */
#endif /* firmware role */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if CAN_CONTROL_ENABLE
    CAN_ProtocolPoll(g_foc_fault);
#elif CAN_DEMO_DEVICE2_ENABLE
    CAN_DemoControllerPoll();
#endif
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
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 14;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 6;
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
