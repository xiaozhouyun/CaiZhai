#include "key.h"
#include "navigation.h"

/**
 * @file    key.c
 * @brief   按键轮询扫描模块（非中断方式，软件消抖）
 *
 * @note    引脚映射：
 *          - led1 (PB9), 内部上拉，按下为低电平
 *          - led3 (PE0), 内部上拉，按下为低电平
 *          - led2 (PC2), 内部上拉，按下为低电平
 *          任何按键按下时，均会首先翻转 user_led 灯 (PB2)。
 *
 *          消抖策略：连续两次扫描读到相同电平才确认状态变化。
 *          扫描周期 15ms → 消抖时间约 30ms。
 */

#define KEY_DEBOUNCE_CNT  2U   /**< 消抖确认次数：2 * 扫描周期 = 稳定窗口 */

/* 每个按键的消抖状态 */
static struct {
    uint8_t last;        /**< 上一次读到的电平 */
    uint8_t stable_cnt;  /**< 连续读到相同电平的次数 */
    uint8_t triggered;   /**< 本次按下是否已触发动作（防重复触发）*/
} g_key[3];

/**
 * @brief  按键轮询扫描函数
 * @note   需由 FreeRTOS 任务周期性调用（建议周期 10~20ms）。
 *         下降沿检测：引脚从高（未按下）→ 低（按下）且消抖窗口内保持稳定。
 */
void Key_Scan(void)
{
    /* 读取三个按键的当前电平（上拉 → 未按时为高，按下为低）*/
    const uint8_t raw[3] = {
        (uint8_t)HAL_GPIO_ReadPin(led1_GPIO_Port, led1_Pin), /* led1: PB9 */
        (uint8_t)HAL_GPIO_ReadPin(led3_GPIO_Port, led3_Pin), /* led3: PE0 */
        (uint8_t)HAL_GPIO_ReadPin(led2_GPIO_Port, led2_Pin)  /* led2: PC2 */
    };

    /* 对三个按键做统一的消抖 + 下降沿检测 */
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (raw[i] == g_key[i].last)
        {
            /* 电平未变，累加消抖计数 */
            if (g_key[i].stable_cnt < KEY_DEBOUNCE_CNT)
            {
                g_key[i].stable_cnt++;
            }

            /* 消抖窗口满足，且为低电平（按下），且本轮未触发过 → 有效按下 */
            if (g_key[i].stable_cnt >= KEY_DEBOUNCE_CNT
                && raw[i] == 0U
                && g_key[i].triggered == 0U)
            {
                g_key[i].triggered = 1U;

                /* 任意按键按下都翻转 LED */
                HAL_GPIO_TogglePin(user_led_GPIO_Port, user_led_Pin);

                switch (i)
                {
                case 0U: /* led1: 启动原地旋转 */
                    Chassis_SetSpeed(0.0f, 0.2f);
                    break;
                case 1U: /* led3: 预留 */
                    break;
                case 2U: /* led2: 预留 */
                    break;
                default:
                    break;
                }
            }
        }
        else
        {
            /* 电平跳变 → 复位消抖计数，等待重新稳定 */
            g_key[i].stable_cnt = 0U;
            g_key[i].last = raw[i];
        }

        /* 按键释放（回到高电平且稳定）→ 复位触发标记，允许下次按下再次响应 */
        if (g_key[i].stable_cnt >= KEY_DEBOUNCE_CNT && raw[i] == 1U)
        {
            g_key[i].triggered = 0U;
        }
    }
}


