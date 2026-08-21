#include "key.h"
#include "app.h"

/**
 * @file    key.c
 * @brief   按键及 EXTI 外部中断处理模块
 * 
 * @note    引脚及中断映射关系：
 *          - Key1: key1_Pin (PB9), EXTI9_5_IRQn，下降沿触发
 *          - Key2: key2_Pin (PE0), EXTI0_IRQn，  下降沿触发
 *          - Key3: key3_Pin (PC2), EXTI2_IRQn，  下降沿触发
 *          任何按键按下触发 EXTI 中断时，均会首先翻转 user_led 灯 (PB2)。
 */

/**
 * @brief  重写 HAL 库 GPIO 外部中断回调函数 (GPIO EXTI Callback)
 * @param  GPIO_Pin 触发中断的 GPIO 引脚编号
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* 检查是否为按键 1、按键 2 或按键 3 的中断触发 */
    if (GPIO_Pin == key1_Pin || GPIO_Pin == key2_Pin || GPIO_Pin == key3_Pin)
    {
        /* 先翻转一次 user_led 灯 (PB2) */
        HAL_GPIO_TogglePin(user_led_GPIO_Port, user_led_Pin);

        /* 针对不同按键的后续逻辑分支 */
        if (GPIO_Pin == key1_Pin)
        {
            /* Key1 按下处理 */
        }
        else if (GPIO_Pin == key2_Pin)
        {
            /* Key2 按下处理 */
              
        }
        else if (GPIO_Pin == key3_Pin)
        {
            /* Key3 按下处理 */
                s_app_running = true;
                s_stop_requested = false;
                App_SetMode(APP_MODE_ROUTE_A);
                //   App_SetMode(APP_MODE_TEST);
        }
    }
}

