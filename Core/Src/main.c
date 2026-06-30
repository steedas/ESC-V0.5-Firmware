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

#define DRV_CTRL_CLR_FLT         (1U << 0)
#define DRV_CTRL_BRAKE           (1U << 1)
#define DRV_CTRL_COAST           (1U << 2)
#define DRV_CTRL_PWM_MODE_MASK   (3U << 5)
#define DRV_CTRL_PWM_MODE_6X     (0U << 5)

#define ADC_VREF                 3.3f
#define CURRENT_GAIN             (0.0015f * 20.0f)

#define MOTOR_POLE_PAIRS         12U
#define MOTOR_DUTY_TICKS         200U
#define DESIRED_MOTOR_DUTY       0.3f
#define MOTOR_START_ELECTRICAL_HZ 5.0f
#define MOTOR_HIGH_ELECTRICAL_HZ 500.0f
#define MOTOR_LOW_ELECTRICAL_HZ  10.0f
#define MOTOR_RAMP_EHZ_PER_SEC   15.0f
#define MOTOR_DEBUG_PRINT_MS     10000000U
#define MOTOR_SVPWM_UPDATE_US    250U
#define MOTOR_SVPWM_MAX_INDEX    0.57735026919f
#define MOTOR_OPEN_LOOP_MODULATION 0.08f
#define MOTOR_TIM1_DEADTIME_TICKS 0U

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
static uint8_t prev_high = 0;
static uint8_t prev_low = 0;
volatile float g_target_electrical_hz = MOTOR_START_ELECTRICAL_HZ;
volatile float g_actual_electrical_hz = MOTOR_START_ELECTRICAL_HZ;
volatile uint16_t g_pwm_duty_ticks = MOTOR_DUTY_TICKS;
volatile uint16_t g_drv_fault_status1 = 0U;
volatile uint16_t g_drv_vgs_status2 = 0U;
volatile uint16_t g_drv_driver_control = 0U;

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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

uint16_t DRV8353_ReadSPI(uint8_t reg)
{
  uint16_t rx = 0;
  uint16_t tx = 0x8000U | ((reg & 0x0FU) << 11);
  HAL_StatusTypeDef status;

  if (SPI1_WaitNotBusy(2U) == 0U)
  {
    return 0x07FFU;
  }

  HAL_GPIO_WritePin(DRV_SCS_N_GPIO_Port, DRV_SCS_N_Pin, GPIO_PIN_RESET);
  delay_us(1);
  status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tx, (uint8_t *)&rx, 1, HAL_MAX_DELAY);
  delay_us(1);
  HAL_GPIO_WritePin(DRV_SCS_N_GPIO_Port, DRV_SCS_N_Pin, GPIO_PIN_SET);

  if (status != HAL_OK)
  {
    Debug_Status1_Set(GPIO_PIN_SET);
  }

  delay_us(1);

  return (rx & 0x07FFU);
}

void DRV8353_WriteSPI(uint8_t reg, uint16_t data)
{
  uint16_t rx = 0;
  uint16_t tx = ((reg & 0x0FU) << 11) | (data & 0x07FFU);
  HAL_StatusTypeDef status;

  if (SPI1_WaitNotBusy(2U) == 0U)
  {
    return;
  }

  HAL_GPIO_WritePin(DRV_SCS_N_GPIO_Port, DRV_SCS_N_Pin, GPIO_PIN_RESET);
  delay_us(1);
  status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tx, (uint8_t *)&rx, 1, HAL_MAX_DELAY);
  delay_us(1);
  HAL_GPIO_WritePin(DRV_SCS_N_GPIO_Port, DRV_SCS_N_Pin, GPIO_PIN_SET);

  if (status != HAL_OK)
  {
    Debug_Status1_Set(GPIO_PIN_SET);
  }

  delay_us(1);
}

uint8_t DRV8353_ReadFaults(void)
{
  g_drv_fault_status1 = DRV8353_ReadSPI(FAULT_STATUS1_REG_ADDR);
  g_drv_vgs_status2 = DRV8353_ReadSPI(VGS_STATUS2_REG_ADDR);

  return ((g_drv_fault_status1 != 0U) || (g_drv_vgs_status2 != 0U)) ? 1U : 0U;
}

void DRV8353_UpdateFaultLED(void)
{
  if (DRV8353_ReadFaults() != 0U)
  {
    Debug_Status1_Set(GPIO_PIN_SET);
  }
  else
  {
    Debug_Status1_Set(GPIO_PIN_RESET);
  }
}

void DRV8353_ConfigureSixPWM(void)
{
  uint16_t driver_control;

  driver_control = DRV8353_ReadSPI(DRIVER_CONTROL_REG_ADDR);
  driver_control |= DRV_CTRL_CLR_FLT;
  DRV8353_WriteSPI(DRIVER_CONTROL_REG_ADDR, driver_control);
  HAL_Delay(1);

  driver_control &= (uint16_t)~(DRV_CTRL_PWM_MODE_MASK |
                                DRV_CTRL_COAST |
                                DRV_CTRL_BRAKE |
                                DRV_CTRL_CLR_FLT);
  driver_control |= DRV_CTRL_PWM_MODE_6X;
  DRV8353_WriteSPI(DRIVER_CONTROL_REG_ADDR, driver_control);
  HAL_Delay(1);

  g_drv_driver_control = DRV8353_ReadSPI(DRIVER_CONTROL_REG_ADDR);
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
  //maintains state for the disconnected phase 
  if (prev_high != high && prev_low != high)
  {
    Phase_Disconnected(high);
  }

  if (prev_high != low && prev_low != low)
  {
    Phase_Disconnected(low);
  }

  if (prev_high != floating && prev_low != floating)
  {
    Phase_Disconnected(floating);
  }

  Phase_High(high, duty);
  Phase_Low(low);
  Phase_Disconnected(floating);

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

float adc_to_current(uint16_t adc_val, float offset)
{
  float voltage = (adc_val / 4095.0f) * ADC_VREF;
  return (voltage - offset) / CURRENT_GAIN;
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
  static uint16_t divider = 0;

  divider++;
  if (divider >= 1000U)
  {
    divider = 0;
    HAL_GPIO_TogglePin(STATUS_2_GPIO_Port, STATUS_2_Pin);
  }
}

static void TIM1_Force_PWM_Outputs_Enabled(void)
{
  TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE |
                 TIM_CCER_CC2E | TIM_CCER_CC2NE |
                 TIM_CCER_CC3E | TIM_CCER_CC3NE);
  TIM1->BDTR |= TIM_BDTR_MOE;
  TIM1->CR1 |= TIM_CR1_CEN;
}

void TIM1_Set_EdgeAligned_For_OpenLoopPWM(void)
{
  __HAL_TIM_DISABLE(&htim1);
  TIM1->CR1 &= ~(TIM_CR1_CMS | TIM_CR1_DIR);
  TIM1->BDTR &= ~TIM_BDTR_DTG;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  TIM1->EGR = TIM_EGR_UG;
  __HAL_TIM_ENABLE(&htim1);
  TIM1_Force_PWM_Outputs_Enabled();
}

void TIM1_Set_CenterAligned_For_SVPWM(void)
{
  Set_Phase_PWM_Ticks(__HAL_TIM_GET_AUTORELOAD(&htim1) / 2U,
                      __HAL_TIM_GET_AUTORELOAD(&htim1) / 2U,
                      __HAL_TIM_GET_AUTORELOAD(&htim1) / 2U);

  __HAL_TIM_DISABLE(&htim1);
  TIM1->CR1 = (TIM1->CR1 & ~(TIM_CR1_CMS | TIM_CR1_DIR)) | TIM_COUNTERMODE_CENTERALIGNED1;
  TIM1->BDTR = (TIM1->BDTR & ~TIM_BDTR_DTG) | MOTOR_TIM1_DEADTIME_TICKS;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  TIM1->EGR = TIM_EGR_UG;
  __HAL_TIM_ENABLE(&htim1);
  TIM1_Force_PWM_Outputs_Enabled();
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

static void Set_OpenLoop_SVPWM(float electrical_angle_rad, float modulation_index)
{
  float sin_theta = sinf(electrical_angle_rad);
  float cos_theta = cosf(electrical_angle_rad);
  float vd = 0.0f;
  float vq;
  float v_alpha;
  float v_beta;
  float va;
  float vb;
  float vc;
  float v_max;
  float v_min;
  float v_offset;
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);

  modulation_index = ClampFloat(modulation_index, 0.0f, MOTOR_SVPWM_MAX_INDEX);
  vq = modulation_index;

  /* Vd = 0, Vq = modulation_index. This makes a rotating voltage vector without encoder feedback. */
  Inverse_Park_Transform(vd, vq, sin_theta, cos_theta, &v_alpha, &v_beta);

  va = v_alpha;
  vb = -0.5f * v_alpha + (0.5f * SQRT3_F) * v_beta;
  vc = -0.5f * v_alpha - (0.5f * SQRT3_F) * v_beta;

  v_max = fmaxf(va, fmaxf(vb, vc));
  v_min = fminf(va, fminf(vb, vc));
  v_offset = -0.5f * (v_max + v_min);

  va = ClampFloat(0.5f + va + v_offset, 0.0f, 1.0f);
  vb = ClampFloat(0.5f + vb + v_offset, 0.0f, 1.0f);
  vc = ClampFloat(0.5f + vc + v_offset, 0.0f, 1.0f);

  Set_Phase_PWM_Ticks((uint32_t)(va * (float)arr),
                      (uint32_t)(vb * (float)arr),
                      (uint32_t)(vc * (float)arr));
  Debug_Status2_ToggleSlow();
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

void Print_Frequency_Info(void)
{
  printf("SYSCLK: %lu Hz\r\n", HAL_RCC_GetSysClockFreq());
  printf("HCLK:   %lu Hz\r\n", HAL_RCC_GetHCLKFreq());
  printf("PCLK1:  %lu Hz\r\n", HAL_RCC_GetPCLK1Freq());
  printf("PCLK2:  %lu Hz\r\n", HAL_RCC_GetPCLK2Freq());
  printf("TIM1:   %lu Hz\r\n", Get_TIM1_ClockHz());
  printf("PWM:    %lu Hz\r\n", Get_PWM_FrequencyHz());
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
    CommutateStep(step++, g_pwm_duty_ticks);
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

void BLDC_OpenLoop_SVPWM_RampLoop(void)
{
  float electrical_angle_rad = 0.0f;
  float electrical_hz = MOTOR_START_ELECTRICAL_HZ;
  uint32_t last_update_cycles = DWT->CYCCNT;
  uint32_t last_tick = HAL_GetTick();
  uint32_t start_tick = last_tick;
  uint32_t last_print_tick = last_tick;
  uint32_t last_fault_poll_tick = last_tick;
  const float update_period_sec = (float)MOTOR_SVPWM_UPDATE_US / 1000000.0f;
  const uint32_t update_period_cycles = (SystemCoreClock / 1000000U) * MOTOR_SVPWM_UPDATE_US;

  g_target_electrical_hz = MOTOR_HIGH_ELECTRICAL_HZ;
  g_actual_electrical_hz = electrical_hz;
  TIM1_Set_CenterAligned_For_SVPWM();
  Set_OpenLoop_SVPWM(electrical_angle_rad, 0.0f);
  DRV8353_UpdateFaultLED();

  while (1)
  {
    uint32_t now_cycles = DWT->CYCCNT;

    if ((now_cycles - last_update_cycles) >= update_period_cycles)
    {
      uint32_t now_tick = HAL_GetTick();
      float ramp_dt_sec = (now_tick - last_tick) / 1000.0f;

      last_update_cycles += update_period_cycles;
      last_tick = now_tick;

      if ((now_tick - start_tick) < 3000U)
      {
        g_target_electrical_hz = MOTOR_START_ELECTRICAL_HZ;
      }
      else
      {
        g_target_electrical_hz = MOTOR_HIGH_ELECTRICAL_HZ;
      }

      electrical_hz = RampToward(electrical_hz,
                                 g_target_electrical_hz,
                                 MOTOR_RAMP_EHZ_PER_SEC,
                                 ramp_dt_sec);

      g_actual_electrical_hz = electrical_hz;
      electrical_angle_rad = WrapRadians(electrical_angle_rad +
                                         (TWO_PI_F * electrical_hz * update_period_sec));

      Set_OpenLoop_SVPWM(electrical_angle_rad, MOTOR_OPEN_LOOP_MODULATION);

      if ((now_tick - last_fault_poll_tick) >= 100U)
      {
        last_fault_poll_tick = now_tick;
        DRV8353_UpdateFaultLED();
      }

      if ((now_tick - last_print_tick) >= MOTOR_DEBUG_PRINT_MS)
      {
        last_print_tick = now_tick;
        printf("open-loop svpwm: target %.2f eHz, actual %.2f eHz, rpm %.1f, pwm %lu Hz, mod %.3f, arr %lu\r\n",
               g_target_electrical_hz,
               g_actual_electrical_hz,
               Get_MechanicalRPM(),
               Get_PWM_FrequencyHz(),
               MOTOR_OPEN_LOOP_MODULATION,
               __HAL_TIM_GET_AUTORELOAD(&htim1));
      }
    }
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

void Start_PWM(void)
{
  Debug_Status1_Set(GPIO_PIN_RESET);
  Debug_Status2_Set(GPIO_PIN_RESET);

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

  TIM1_Force_PWM_Outputs_Enabled();
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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();
  HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  DRV8353_ConfigureSixPWM();
  DRV8353_UpdateFaultLED();

  Start_PWM();
  Print_Frequency_Info();
  BLDC_OpenLoop_SVPWM_RampLoop();

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

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
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
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
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
