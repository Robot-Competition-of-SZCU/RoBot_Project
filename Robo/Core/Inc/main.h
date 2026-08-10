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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Digial_OUT3_Pin GPIO_PIN_2
#define Digial_OUT3_GPIO_Port GPIOE
#define Digial_OUT4_Pin GPIO_PIN_3
#define Digial_OUT4_GPIO_Port GPIOE
#define Slave2_Pin GPIO_PIN_5
#define Slave2_GPIO_Port GPIOE
#define Slave1_Pin GPIO_PIN_6
#define Slave1_GPIO_Port GPIOE
#define RUN_LED_Pin GPIO_PIN_13
#define RUN_LED_GPIO_Port GPIOC
#define Digital_IN9_Pin GPIO_PIN_0
#define Digital_IN9_GPIO_Port GPIOF
#define Digital_IN10_Pin GPIO_PIN_1
#define Digital_IN10_GPIO_Port GPIOF
#define Digital_IN11_Pin GPIO_PIN_2
#define Digital_IN11_GPIO_Port GPIOF
#define Digital_IN12_Pin GPIO_PIN_3
#define Digital_IN12_GPIO_Port GPIOF
#define Digital_IN13_Pin GPIO_PIN_4
#define Digital_IN13_GPIO_Port GPIOF
#define Digital_IN14_Pin GPIO_PIN_5
#define Digital_IN14_GPIO_Port GPIOF
#define Digital_IN15_Pin GPIO_PIN_6
#define Digital_IN15_GPIO_Port GPIOF
#define Digital_IN16_Pin GPIO_PIN_7
#define Digital_IN16_GPIO_Port GPIOF
#define Rocker_UP_Pin GPIO_PIN_11
#define Rocker_UP_GPIO_Port GPIOF
#define Rocker_Right_Pin GPIO_PIN_12
#define Rocker_Right_GPIO_Port GPIOF
#define Rocker_Down_Pin GPIO_PIN_13
#define Rocker_Down_GPIO_Port GPIOF
#define Rocker_Enter_Pin GPIO_PIN_14
#define Rocker_Enter_GPIO_Port GPIOF
#define Rocker_Left_Pin GPIO_PIN_15
#define Rocker_Left_GPIO_Port GPIOF
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOG
#define KEY2_Pin GPIO_PIN_1
#define KEY2_GPIO_Port GPIOG
#define KEY3_Pin GPIO_PIN_7
#define KEY3_GPIO_Port GPIOE
#define Digital_IN1_Pin GPIO_PIN_8
#define Digital_IN1_GPIO_Port GPIOE
#define Digital_IN2_Pin GPIO_PIN_9
#define Digital_IN2_GPIO_Port GPIOE
#define Digital_IN3_Pin GPIO_PIN_10
#define Digital_IN3_GPIO_Port GPIOE
#define Digital_IN4_Pin GPIO_PIN_11
#define Digital_IN4_GPIO_Port GPIOE
#define Digital_IN5_Pin GPIO_PIN_12
#define Digital_IN5_GPIO_Port GPIOE
#define Digital_IN6_Pin GPIO_PIN_13
#define Digital_IN6_GPIO_Port GPIOE
#define Digital_IN7_Pin GPIO_PIN_14
#define Digital_IN7_GPIO_Port GPIOE
#define Digital_IN8_Pin GPIO_PIN_15
#define Digital_IN8_GPIO_Port GPIOE
#define KEY4_Pin GPIO_PIN_12
#define KEY4_GPIO_Port GPIOB
#define OLED_RES_Pin GPIO_PIN_10
#define OLED_RES_GPIO_Port GPIOD
#define OLED_DC_Pin GPIO_PIN_11
#define OLED_DC_GPIO_Port GPIOD
#define Encoder3_A_Pin GPIO_PIN_12
#define Encoder3_A_GPIO_Port GPIOD
#define Encoder3_B_Pin GPIO_PIN_13
#define Encoder3_B_GPIO_Port GPIOD
#define Encoder4_A_Pin GPIO_PIN_14
#define Encoder4_A_GPIO_Port GPIOD
#define Encoder4_B_Pin GPIO_PIN_15
#define Encoder4_B_GPIO_Port GPIOD
#define SPI_CS1_Pin GPIO_PIN_2
#define SPI_CS1_GPIO_Port GPIOG
#define SPI_CS2_Pin GPIO_PIN_3
#define SPI_CS2_GPIO_Port GPIOG
#define SPI_CS3_Pin GPIO_PIN_4
#define SPI_CS3_GPIO_Port GPIOG
#define M1_Control_Pin GPIO_PIN_5
#define M1_Control_GPIO_Port GPIOG
#define M2_Control_Pin GPIO_PIN_6
#define M2_Control_GPIO_Port GPIOG
#define M3_Control_Pin GPIO_PIN_7
#define M3_Control_GPIO_Port GPIOG
#define M4_Control_Pin GPIO_PIN_8
#define M4_Control_GPIO_Port GPIOG
#define Encoder1_A_Pin GPIO_PIN_6
#define Encoder1_A_GPIO_Port GPIOC
#define Encoder1_B_Pin GPIO_PIN_7
#define Encoder1_B_GPIO_Port GPIOC
#define Encoder2_A_Pin GPIO_PIN_8
#define Encoder2_A_GPIO_Port GPIOC
#define Encoder2_B_Pin GPIO_PIN_9
#define Encoder2_B_GPIO_Port GPIOC
#define M1_PWM_Pin GPIO_PIN_8
#define M1_PWM_GPIO_Port GPIOA
#define M2_PWM_Pin GPIO_PIN_9
#define M2_PWM_GPIO_Port GPIOA
#define M3_PWM_Pin GPIO_PIN_10
#define M3_PWM_GPIO_Port GPIOA
#define M4_PWM_Pin GPIO_PIN_11
#define M4_PWM_GPIO_Port GPIOA
#define Slave8_Pin GPIO_PIN_15
#define Slave8_GPIO_Port GPIOA
#define OLED_Select_Pin GPIO_PIN_0
#define OLED_Select_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOD
#define Digial_OUT5_Pin GPIO_PIN_10
#define Digial_OUT5_GPIO_Port GPIOG
#define Digial_OUT6_Pin GPIO_PIN_11
#define Digial_OUT6_GPIO_Port GPIOG
#define Digial_OUT7_Pin GPIO_PIN_12
#define Digial_OUT7_GPIO_Port GPIOG
#define Digial_OUT8_Pin GPIO_PIN_13
#define Digial_OUT8_GPIO_Port GPIOG
#define Slave7_Pin GPIO_PIN_3
#define Slave7_GPIO_Port GPIOB
#define Slave6_Pin GPIO_PIN_4
#define Slave6_GPIO_Port GPIOB
#define Slave5_Pin GPIO_PIN_5
#define Slave5_GPIO_Port GPIOB
#define Slave4_Pin GPIO_PIN_8
#define Slave4_GPIO_Port GPIOB
#define Slave3_Pin GPIO_PIN_9
#define Slave3_GPIO_Port GPIOB
#define Digial_OUT1_Pin GPIO_PIN_0
#define Digial_OUT1_GPIO_Port GPIOE
#define Digial_OUT2_Pin GPIO_PIN_1
#define Digial_OUT2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
