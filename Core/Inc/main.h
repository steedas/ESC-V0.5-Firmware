/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct
{
  float ia;
  float ib;
  float ic;
  float ib_sensed;
  float ib_sensed_corrected;
  float ic_sensed;
  float ia_filtered;
  float ib_filtered;
  float ic_filtered;
  float current_sum;
  uint16_t raw_a;
  uint16_t raw_b;
  uint16_t raw_c;
  uint16_t offset_a;
  uint16_t offset_b;
  uint16_t offset_c;
  float peak_phase_current;
  uint32_t sample_count;
  uint8_t valid_a;
  uint8_t valid_b;
  uint8_t valid_c;
  uint8_t saturated_a;
  uint8_t saturated_b;
  uint8_t saturated_c;
} PhaseCurrents_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

PhaseCurrents_t CurrentSense_GetPhaseCurrents(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SOA_Pin GPIO_PIN_0
#define SOA_GPIO_Port GPIOC
#define SOB_Pin GPIO_PIN_1
#define SOB_GPIO_Port GPIOC
#define SOC_Pin GPIO_PIN_2
#define SOC_GPIO_Port GPIOC
#define DRV_FAULT_Pin GPIO_PIN_0
#define DRV_FAULT_GPIO_Port GPIOA
#define DRV_ENABLE_Pin GPIO_PIN_1
#define DRV_ENABLE_GPIO_Port GPIOA
#define DRV_SCS_N_Pin GPIO_PIN_4
#define DRV_SCS_N_GPIO_Port GPIOA
#define DRV_SCLK_Pin GPIO_PIN_5
#define DRV_SCLK_GPIO_Port GPIOA
#define DRV_MISO_Pin GPIO_PIN_6
#define DRV_MISO_GPIO_Port GPIOA
#define DRV_MOSI_Pin GPIO_PIN_7
#define DRV_MOSI_GPIO_Port GPIOA
#define STATUS_1_Pin GPIO_PIN_0
#define STATUS_1_GPIO_Port GPIOB
#define STATUS_2_Pin GPIO_PIN_1
#define STATUS_2_GPIO_Port GPIOB
#define DRV_INLA_Pin GPIO_PIN_13
#define DRV_INLA_GPIO_Port GPIOB
#define DRV_INLB_Pin GPIO_PIN_14
#define DRV_INLB_GPIO_Port GPIOB
#define DRV_INLC_Pin GPIO_PIN_15
#define DRV_INLC_GPIO_Port GPIOB
#define ENC_I2C_SCL_Pin GPIO_PIN_8
#define ENC_I2C_SCL_GPIO_Port GPIOC
#define ENC_I2C_SDA_Pin GPIO_PIN_9
#define ENC_I2C_SDA_GPIO_Port GPIOC
#define DRV_INHA_Pin GPIO_PIN_8
#define DRV_INHA_GPIO_Port GPIOA
#define DRV_INHB_Pin GPIO_PIN_9
#define DRV_INHB_GPIO_Port GPIOA
#define DRV_INHC_Pin GPIO_PIN_10
#define DRV_INHC_GPIO_Port GPIOA
#define ENC_SPI_CLK_Pin GPIO_PIN_10
#define ENC_SPI_CLK_GPIO_Port GPIOC
#define ENC_SPI_MISO_Pin GPIO_PIN_11
#define ENC_SPI_MISO_GPIO_Port GPIOC
#define ENC_SPI_MOSI_Pin GPIO_PIN_12
#define ENC_SPI_MOSI_GPIO_Port GPIOC
#define USB_VSENSE_Pin GPIO_PIN_4
#define USB_VSENSE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
