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
#include "action_scheduler.h"
#include "key.h"
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
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask06 */
osThreadId_t myTask06Handle;
const osThreadAttr_t myTask06_attributes = {
  .name = "myTask06",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
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
  /* 仅复位软件状态；此时调度器和外设任务均尚未开始运行。 */
  App_Init();
  ActionScheduler_Init();
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
      osDelay(1000);

      /* 使能四轮驱动电机，snF=true 等待同步触发 */
      Emm_V5_En_Control(1, true, true);
      Emm_V5_En_Control(2, true, true);
      Emm_V5_En_Control(3, true, true);
      Emm_V5_En_Control(4, true, true);
      Emm_V5_En_Control(5, true, true);
      Emm_V5_Synchronous_motion(0);
      Navigation_Stop();
//              Move_Pos(25.0f);
             vTaskDelay(pdMS_TO_TICKS(2000U));
//        s_app_running = true;
//        s_stop_requested = false;
//        App_SetMode(APP_MODE_ROUTE_A);
   
  /* 初始化完成，任务自杀 */
  vTaskDelete(NULL);
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
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  static uint32_t oled_last_refresh = 0;
         osDelay(500);
  for(;;)
  {
    if ((OLED_IsReady() != 0U) && ((HAL_GetTick() - oled_last_refresh) >= 500U))
    {
      oled_last_refresh = HAL_GetTick();
      oled_print(0, 0, 16, "X=%.2f", g_robot_pos.x);
      oled_print(0, 2, 16, "y=%.2f", g_robot_pos.y);
      oled_print(0, 4, 16, "yaw=%.2f", g_robot_pos.yaw);
      // {
      //   float vofa_data[4] = {g_robot_pos.x, g_robot_pos.y, g_robot_pos.yaw, (*anglepid.target) * 180.0f / 3.1415926f};
      //   Vofa_SendFirewater(vofa_data, 4);
      // }
    }
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
     vTaskDelay(pdMS_TO_TICKS(50));
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the myTask04 thread.
*        优先级: osPriorityBelowNormal6 (低于正常优先级 6)
*        功能: 保留任务（应用逻辑已迁移到 StartTask07）
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* 保留任务，启动后自删除 */
  osDelay(500);
  vTaskDelete(NULL);
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
       osDelay(200);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
//      Chassis_SetSpeed(0.0f,0.2f);
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
  { 
      Navigation_TaskTick();
      vTaskDelay(pdMS_TO_TICKS(10));
  }
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief Function implementing the myTask07 thread.
*        优先级: osPriorityNormal1 (高于正常优先级 1)
*        功能: 路线、抓取与云台非阻塞状态机（20ms Tick）
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(15);

  /* 获取当前的系统节拍作为初始唤醒时间 */
  xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
    /* 两个状态机均为单步推进；15ms 是严格的动作时间基准，不得在其中阻塞。 */
    Key_Scan();
    App_RunCurrentMode();
    ActionScheduler_Tick();
    
    /* 使用绝对延时替代相对延时，消除任务执行耗时和抢占带来的抖动 */
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
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

