/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/*___________________________________Settings___________________________________*/
#define RS_EJC_HEIGHT 170						//RS will be activated if below that value, cm
#define TEAM_NAME "Sporadic-V (SHARAGA_FOREVER!)"  //Team name
#define SD_FILENAME "gg.wp"						//microSD file name, BIN content
#define SD_FILENAME_WQ "gg.wq"				//microSD memory dump file name, BIN content
#define DATA_PERIOD 85								//Data update period, ms
#define DATA_PERIOD_LND 85						//Data update period after landing, ms
#define PRESS_BUFFER_LEN 15						//Buffer for landing detection
#define	PRESS_LAND_DELTA 10						//Set land flag if buffered pressure difference < [this val], Pa
/*_____________________________Settings_(Danger_Zone)____________________________*/
#define FRAME_SIZE 56			//frame for LoRa & WQ, bytes
#define START_TH 10      //Set start flag if altitude > [this val], cm
#define EJECT_TH 240			//Set EJC flag if adc_val < [this_val], 8bit
#define RS_ANGLE 30			//RS Servo angle,<=120 deg
#define RS_DELAY 3000			//Time while RS stays active (TEST ONLY), ms

/*___________________________________Bit Macro___________________________________*/
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1 << (bit)))
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWR2_EN_Pin GPIO_PIN_0
#define PWR2_EN_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOA
#define LED_ERR_Pin GPIO_PIN_1
#define LED_ERR_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOA
#define WQ_HOLD_Pin GPIO_PIN_4
#define WQ_HOLD_GPIO_Port GPIOA
#define WQ_NSS_Pin GPIO_PIN_4
#define WQ_NSS_GPIO_Port GPIOC
#define LORA_NSS_Pin GPIO_PIN_5
#define LORA_NSS_GPIO_Port GPIOC
#define MS_NSS_Pin GPIO_PIN_0
#define MS_NSS_GPIO_Port GPIOB
#define LIS_NSS_Pin GPIO_PIN_1
#define LIS_NSS_GPIO_Port GPIOB
#define LSM_NSS_Pin GPIO_PIN_2
#define LSM_NSS_GPIO_Port GPIOB
#define SDIO_CD_Pin GPIO_PIN_10
#define SDIO_CD_GPIO_Port GPIOB
#define JMP_PROG_Pin GPIO_PIN_12
#define JMP_PROG_GPIO_Port GPIOB
#define ADC_NSS_Pin GPIO_PIN_9
#define ADC_NSS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
