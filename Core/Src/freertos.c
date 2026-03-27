/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "sbus.h"
#include "servo.h"
#include "BMI088driver.h"
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h"
extern USBD_HandleTypeDef hUsbDeviceFS;

/* ====== 360度舵机物理零点补偿 (软件微调) ====== */
#define TRIM_M1  30
#define TRIM_M3  30
#define TRIM_M4  30
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_temp = 0.0f;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartLedTask(void const * argument);
void StartControlTask(void const * argument);
void StartIMUTask(void const * argument);
/* USER CODE END FunctionPrototypes */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ========== Task 1: LED 心跳 ========== */
void StartLedTask(void const * argument)
{
  /* USER CODE BEGIN StartLedTask */
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);  // 蓝灯闪烁 (系统心跳)
    osDelay(500);
  }
  /* USER CODE END StartLedTask */
}

/* ========== Task 2: 核心战车控制 ========== */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
  for(;;)
  {
    // 1. 信号状态灯
    if(SBUS_IsValid()) {
      HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);  // 绿灯常亮
      HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET);    // 红灯灭
    } else {
      HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET);    // 绿灯灭
      HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET);  // 红灯亮
    }

    SBUS_Process();

    // 2. 双模式遥控控制
    if (SBUS_IsValid())
    {
      int16_t channel1 = SBUS_GetChannel(1);
      int16_t channel2 = SBUS_GetChannel(2);
      int16_t channel3 = SBUS_GetChannel(3);
      int16_t channel9 = SBUS_GetChannel(9);
      int16_t channel5 = SBUS_GetChannel(5);

      if (channel9 < 1000)
      {
        // 模式1：同步驾驶模式 (左手上下方向，右手上下油门)
        int16_t drive_speed = 0;
        uint16_t steer_pulse = 1500;

        int16_t ch2_offset = channel2 - 1024;
        if (ch2_offset > -50 && ch2_offset < 50) {
          drive_speed = 0;
        } else if (ch2_offset >= 50) {
          drive_speed = (ch2_offset * 1000) / (1700 - 1024);
        } else {
          drive_speed = (ch2_offset * 1000) / (1024 - 300);
        }

        if (drive_speed > 1000) drive_speed = 1000;
        if (drive_speed < -1000) drive_speed = -1000;

        int16_t servo1_speed = (drive_speed * 50) / 63;

        Servo_Set360(1, servo1_speed + TRIM_M1);
        Servo_Set360(3, drive_speed + TRIM_M3);
        Servo_Set360(4, drive_speed + TRIM_M4);

        #define STEER_MIN_PULSE  500
        #define STEER_MAX_PULSE  2500

        int16_t ch3_offset = channel3 - 1024;
        if (ch3_offset > -30 && ch3_offset < 30) {
          steer_pulse = 1500;
        } else {
          steer_pulse = 1500 + (ch3_offset * 1000) / (1700 - 1024);
        }

        if (steer_pulse > STEER_MAX_PULSE) steer_pulse = STEER_MAX_PULSE;
        if (steer_pulse < STEER_MIN_PULSE) steer_pulse = STEER_MIN_PULSE;

        Servo_Set180(2, steer_pulse);
      }
      else
      {
        // 模式2：纯右手调试模式
        int16_t servo1_speed = 0;
        int16_t servo3_speed = 0;
        int16_t servo4_speed = 0;

        int16_t ch2_offset = channel2 - 1024;
        if (ch2_offset > -50 && ch2_offset < 50) servo1_speed = 0;
        else if (ch2_offset >= 50) servo1_speed = (ch2_offset * 1000) / (1700 - 1024);
        else servo1_speed = (ch2_offset * 1000) / (1024 - 300);

        int16_t ch1_offset = channel1 - 1024;
        int16_t ch1_speed = 0;
        if (ch1_offset > -50 && ch1_offset < 50) ch1_speed = 0;
        else if (ch1_offset >= 50) ch1_speed = (ch1_offset * 1000) / (1700 - 1024);
        else ch1_speed = (ch1_offset * 1000) / (1024 - 300);

        if (channel5 < 1000) {
          servo3_speed = ch1_speed;
        } else {
          servo4_speed = ch1_speed;
        }

        if (servo1_speed > 1000) servo1_speed = 1000; if (servo1_speed < -1000) servo1_speed = -1000;
        if (servo3_speed > 1000) servo3_speed = 1000; if (servo3_speed < -1000) servo3_speed = -1000;
        if (servo4_speed > 1000) servo4_speed = 1000; if (servo4_speed < -1000) servo4_speed = -1000;

        Servo_Set360(1, servo1_speed + TRIM_M1);
        Servo_Set360(3, servo3_speed + TRIM_M3);
        Servo_Set360(4, servo4_speed + TRIM_M4);

        Servo_Set180(2, 1500);
      }
    }
    else
    {
      // 失控保护
      Servo_Set360(1, TRIM_M1);
      Servo_Set360(3, TRIM_M3);
      Servo_Set360(4, TRIM_M4);
      Servo_Set180(2, 1500);
    }

    osDelay(10);
  }
  /* USER CODE END StartControlTask */
}

/* ========== Task 3: IMU 读取任务 ========== */
void StartIMUTask(void const * argument)
{
  // 1. 初始化大疆 IMU 核武器
  while (BMI088_init() != 0)
  {
      osDelay(50);
  }

  char usb_buf[128];

  for(;;)
  {
      // 2. 读取角速度和加速度
      BMI088_read(INS_gyro, INS_accel, &INS_temp);

      // 3. FireWater 协议格式
      sprintf(usb_buf, "%d,%d,%d,%d,%d,%d\n",
              (int)(INS_gyro[0] * 1000), (int)(INS_gyro[1] * 1000), (int)(INS_gyro[2] * 1000),
              (int)(INS_accel[0] * 1000), (int)(INS_accel[1] * 1000), (int)(INS_accel[2] * 1000));

      // 4. 终极防爆安全发送锁
      if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
      {
          if (hUsbDeviceFS.pClassData != NULL)
          {
              USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
              if (hcdc->TxState == 0)
              {
                  extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
                  CDC_Transmit_FS((uint8_t*)usb_buf, strlen(usb_buf));
              }
          }
      }

      osDelay(2);
  }
}

/* USER CODE END Application */
