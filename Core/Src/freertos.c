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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "hwt101_hal.h"
#include "oled.h"
#include "bujin.h"
#include "odometer.h"
#include "tof200f.h"
#include "navigation.h"
#include "voice.h"
#include "overroll.h"
#include "pca9685.h"
#include "app.h"
#include "UpperCP.h"
#include "vofa.h"
#include "tiancan.h"
#include "arms.h"
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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal4,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal5,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal6,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal7,
};
/* Definitions for myTask06 */
osThreadId_t myTask06Handle;
const osThreadAttr_t myTask06_attributes = {
  .name = "myTask06",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask07 */
osThreadId_t myTask07Handle;
const osThreadAttr_t myTask07_attributes = {
  .name = "myTask07",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for usart2TX */
osMutexId_t usart2TXHandle;
const osMutexAttr_t usart2TX_attributes = {
  .name = "usart2TX"
};
/* Definitions for duojiI2c */
osMutexId_t duojiI2cHandle;
const osMutexAttr_t duojiI2c_attributes = {
  .name = "duojiI2c"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);
void StartTask07(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationTickHook(void);

/* USER CODE BEGIN 3 */
/* USER CODE END 3 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of usart2TX */
  usart2TXHandle = osMutexNew(&usart2TX_attributes);

  /* creation of duojiI2c */
  duojiI2cHandle = osMutexNew(&duojiI2c_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(StartTask04, NULL, &myTask04_attributes);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(StartTask05, NULL, &myTask05_attributes);

  /* creation of myTask06 */
  myTask06Handle = osThreadNew(StartTask06, NULL, &myTask06_attributes);

  /* creation of myTask07 */
  myTask07Handle = osThreadNew(StartTask07, NULL, &myTask07_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  *         优先级: osPriorityNormal (正常优先级)
  *         功能: 系统初始化与基础移动控制任务
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
      osDelay(100);
     Navigation_Stop();
     now_pos=0;
    Move_Pos(8.0f);
  /* Infinite loop */
  for(;;){
      
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
*        优先级: osPriorityBelowNormal4 (低于正常优先级 4)
*        功能: 天蚕/天灿传感器及传感器数据处理任务
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  uint32_t oled_last_refresh = 0U;

  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
         osDelay(500);
  for(;;)
  {

    // float vofa_values[5];

    if ((OLED_IsReady() != 0U) && ((HAL_GetTick() - oled_last_refresh) >= 500U))
    {
      oled_last_refresh = HAL_GetTick();
      OLED_ShowString(40, 0, "        ", 16);
      OLED_ShowFloat(40, 0, g_robot_pos.yaw, 6, 16);
      OLED_ShowString(40, 2, "        ", 16);
      OLED_ShowFloat(40, 2, TofData / 10.0f, 6, 16);
      OLED_ShowString(24, 4, "        ", 16);
      OLED_ShowFloat(24, 4, g_robot_pos.x, 6, 16);
      OLED_ShowString(24, 6, "        ", 16);
      OLED_ShowFloat(24, 6, g_robot_pos.y, 6, 16);
    }

    // vofa_values[0] = g_robot_pos.x / 10.0f;
    // vofa_values[1] = g_robot_pos.y / 10.0f;
    // vofa_values[2] = g_robot_pos.yaw;
    // vofa_values[3] = TofData / 10.0f;
    // vofa_values[4] = (float)navigation_state;
    // Vofa_SendFirewater(vofa_values, 5U);

    /* 打印 5U 夹爪舵机当前角度到 VOFA (FireWater 协议) */
    // {
    //     float angle_5u = PCA9685_Get180Angle(6U);
    //     Vofa_SendFirewater(&angle_5u, 1U);
    // }

    Tiancan_Process();
    osDelay(100);
 
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
*        优先级: osPriorityBelowNormal5 (低于正常优先级 5)
*        功能: 航向角获取与里程计位置姿态解算更新任务
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
       osDelay(500);
  /* Infinite loop */
  for(;;)
  {
     g_robot_pos.yaw = Get_zeroYaw();
     Odometer_Update();
     vTaskDelay(pdMS_TO_TICKS(10));
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the myTask04 thread.
*        优先级: osPriorityBelowNormal6 (低于正常优先级 6)
*        功能: 主应用 App 模式与应用逻辑调度任务
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
       osDelay(500);
       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  
  /* Infinite loop */
  App_Init();
  /* 调度器已运行：在任务上下文下发送一次底盘停止帧。 */

    // PCA9685_Set180Angle(1U,-80.0f);

  for(;;)
  {
        
        /* App chain: read current app mode and execute one scheduling step. */
        App_RunCurrentMode();
      
    osDelay(100);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the myTask05 thread.
*        优先级: osPriorityBelowNormal7 (低于正常优先级 7)
*        功能: 上位机串口通信接收处理任务
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  // PCA9685_Set180Angle(1U, 0.0f);
       osDelay(500);

  /* Infinite loop */
  for(;;)
  {
    UpperCP_RX();
     vTaskDelay(pdMS_TO_TICKS(10));
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the myTask06 thread.
*        优先级: osPriorityNormal (正常优先级)
*        功能: 底盘导航控制与路径跟踪计算任务 (10ms Tick)
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
  for(;;)
  {  Navigation_TaskTick();
     
    osDelay(10);
  }
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief Function implementing the myTask07 thread.
*        优先级: osPriorityNormal1 (高于正常优先级 1)
*        功能: 高优先级后台保留任务 / 空闲延时任务
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartTask07 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  FreeRTOS Tick Hook — 双层 LED 心跳
  *         - g_system_error == 0: 500Hz 翻转（示波器可见，人眼常亮）
  *         - g_system_error != 0: 2Hz 翻转（人眼可见闪烁 ≈ 每 250ms 切换一次）
  * @note   必须在 tick ISR 中快速返回，禁止调用阻塞 API
  */
void vApplicationTickHook(void)
{
    static uint32_t tick_count = 0;
    tick_count++;

    if (g_system_error != 0U)
    {
        /* 异常模式：每 250ms 翻转 → 2Hz 人眼可见闪烁 */
        if ((tick_count % 250U) == 0U)
        {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
        }
    }
}

/* USER CODE END Application */

