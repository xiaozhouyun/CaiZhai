#include "UpperCP.h"
#include "cmsis_os.h"
#include "vofa.h"
#include "app.h"
#include "arms.h"
#include "navigation.h"
#include "pca9685.h"
#include "usart.h"
#include "voice.h"
#include "tof200f.h"
#include "bujin.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define UPPERCP_RX_BUF_LEN 128U
#define UPPERCP_CMD_MOVE_GBK "\xD2\xC6\xB6\xAF"

static volatile uint8_t uppercp_rx_buf[UPPERCP_RX_BUF_LEN];
static volatile uint16_t uppercp_rx_head;
static volatile uint16_t uppercp_rx_tail;
static volatile uint32_t uppercp_rx_count;
static volatile uint8_t uppercp_last_byte;
static char uppercp_cmd_buf[UPPERCP_RX_BUF_LEN];
static char uppercp_last_cmd[UPPERCP_RX_BUF_LEN];
static uint16_t uppercp_cmd_len;

static void Serial5_Printf(const char *fmt, ...)
{
    char tx_buf[96];
    va_list args;
    int len;

    va_start(args, fmt);
    len = vsnprintf(tx_buf, sizeof(tx_buf), fmt, args);
    va_end(args);

    if (len <= 0) {
        return;
    }

    if ((uint32_t)len >= sizeof(tx_buf)) {
        len = (int)sizeof(tx_buf) - 1;
    }

    (void)HAL_UART_Transmit(&huart5, (uint8_t *)tx_buf, (uint16_t)len, 100U);
}

void UpperCP_UartRxByte(uint8_t data)
{
    uint16_t next_head = (uint16_t)((uppercp_rx_head + 1U) % UPPERCP_RX_BUF_LEN);

    uppercp_last_byte = data;
    uppercp_rx_count++;

    if (next_head == uppercp_rx_tail) {
        return;
    }

    uppercp_rx_buf[uppercp_rx_head] = data;
    uppercp_rx_head = next_head;
}

void UpperCP_SendTask(const char *task)
{
    static const char line_end[] = "\r\n";

    if (task == NULL) {
        return;
    }

    if ((strcmp(task, "send") != 0) &&
        (strcmp(task, "pour") != 0) &&
        (strcmp(task, "scan") != 0)) {
        return;
    }

    (void)HAL_UART_Transmit(&huart5, (uint8_t *)task, (uint16_t)strlen(task), 100U);
    (void)HAL_UART_Transmit(&huart5, (uint8_t *)line_end, (uint16_t)(sizeof(line_end) - 1U), 100U);
}

const char *UpperCP_GetLastCommand(void)
{
    return uppercp_last_cmd;
}

uint32_t UpperCP_GetRxCount(void)
{
    return uppercp_rx_count;
}

uint8_t UpperCP_GetLastByte(void)
{
    return uppercp_last_byte;
}

static char *ret = NULL;
uint8_t PosFlag = 1;
float angle_dif1 = 0.0f;         /**< 旋转角度微调步进增量全局变量 */
uint8_t upordownFlag = 0;        /**< 上下抓取目标状态标志位 (0：抓地上，1：抓树上) */
uint8_t CameraFlag = 0;
static uint8_t s_retry_count = 0;  /**< 云台极限重试计数器 */

/* uint8_t fruits[8] = {3,5,7,1,6,10,12,9}; */
uint8_t fruits[8] = {4,3,1,10,8,9,2,11};
/* uint8_t fruits[8] = {12,2,9,4,5,11,1,7}; */

uint8_t fruits_count = 0;

void UpperCP_RX(void)
{
    uint8_t command_ready = 0U;

    while (uppercp_rx_tail != uppercp_rx_head) {
        uint8_t ch = uppercp_rx_buf[uppercp_rx_tail];
        uppercp_rx_tail = (uint16_t)((uppercp_rx_tail + 1U) % UPPERCP_RX_BUF_LEN);

        if (ch == ';' || ch == '\r' || ch == '\n') {
            if (uppercp_cmd_len == 0U) {
                continue;
            }
            uppercp_cmd_buf[uppercp_cmd_len] = '\0';
            uppercp_cmd_len = 0U;
            command_ready = 1U;
            break;
        }

        if (uppercp_cmd_len < (UPPERCP_RX_BUF_LEN - 1U)) {
            uppercp_cmd_buf[uppercp_cmd_len++] = (char)ch;
        } else {
            uppercp_cmd_len = 0U;
        }
    }

    if (command_ready == 0U) {
        return;
    }

    /* 将上位机 UpperCP 接收到的原始命令数据通过 VOFA+ 打印输出 */
    Vofa_PrintUpperCPData(uppercp_cmd_buf);

    ret = strtok(uppercp_cmd_buf, ":");
    if (ret != NULL) {
        strncpy(uppercp_last_cmd, ret, sizeof(uppercp_last_cmd) - 1U);
        uppercp_last_cmd[sizeof(uppercp_last_cmd) - 1U] = '\0';

        /* 按首字符分发，只调用匹配的函数 */
        switch (ret[0]) {
            case 'a': Arm_func();   break;  /* arm:X */
            case 'c': cmd_func();   break;  /* cmd:X */
            case 'v': voice_func(); break;  /* voice:X */
            case 'Q': ErWeiMa_func(); break; /* QR:X */
            case 'm': Move_func();  break;  /* move:X */
            default:
                if ((uint8_t)ret[0] == 0xD2) { Move_func(); }  /* 移动(GBK) */
                break;
        }
        ret = NULL;
    }
}

void cmd_func(void)
{
    if (memcmp(ret, "cmd", 3) == 0) {
        float temp_num = 0.0f;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%f", &temp_num);
            Serial5_Printf("num = %f\r\n", temp_num);
        }

    }
}

void speed_func(void)
{
    if (memcmp(ret, "speed", 5) == 0) {
        int temp_num = 0;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%d", &temp_num);
        }

        speed.tar = (float)temp_num;
        speed.diff = (speed.tar - speed.real) / 8.0f;

    }
}

void angle_func(void)
{
    if (memcmp(ret, "angle", 5) == 0) {
        int temp_num = 0;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%d", &temp_num);
        }

        angle_speed.tar = (float)temp_num;
        angle_speed.diff = (angle_speed.tar - angle_speed.real) / 8.0f;

    }
}

void face_func(void)
{
    if (memcmp(ret, "face", 4) == 0) {
        float temp_num = 0.0f;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%f", &temp_num);
        }

        TarAngle = (int)temp_num;

    }
}

void voice_func(void)
{
    if (memcmp(ret, "voice", 5) == 0) {
        int temp_num = 0;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%d", &temp_num);
        }

        Voice_Num(temp_num);

    }
}

void Arm_func(void)
{
    if (memcmp(ret, "arm", 3) == 0) {
        int temp_num = 0;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%d", &temp_num);
        }
        /* 云台角度只读一次，temp_num 1/2 共用 */
        float gimbal_angle = PCA9685_Get180Angle(7U);
        
        if(temp_num == 1)       //目标偏右：整车向前移动 1cm (10.0mm)
        {
            // if (gimbal_angle > 0.0f)   // 云台在右侧：目标偏右 = 车前进
            // {
            //     Emm_V5_Chassis_Pos_Control(0, 50, 20, 10.0f);
            // }
            // else                        // 云台在左侧：方向反转，目标偏右 = 车后退
            // {
            //     Emm_V5_Chassis_Pos_Control(1, 20, 50, 10.0f);
            // }
             PCA9685_Set180Angle(7U,s_pca9685_180_angles[7]+1);
            osDelay(pdMS_TO_TICKS(500U));
        }else
        if(temp_num == 2)       //目标偏左：整车向后移动 1cm (10.0mm)
        {
            // if (gimbal_angle > 0.0f)   // 云台在右侧：目标偏左 = 车后退
            // {
            //     Emm_V5_Chassis_Pos_Control(1, 20, 50, 10.0f);
            // }
            // else                        // 云台在左侧：方向反转，目标偏左 = 车前进
            // {
            //     Emm_V5_Chassis_Pos_Control(0, 50, 20, 10.0f);
            // }
            PCA9685_Set180Angle(7U,s_pca9685_180_angles[7]-1);
            osDelay(pdMS_TO_TICKS(500U));
        }else
		if(temp_num == 3)		//目标偏上
		{
			Move_up(1);
            osDelay(pdMS_TO_TICKS(500U));
		}else
		if(temp_num == 4)		//目标偏下
		{
			Move_down(1);
            osDelay(pdMS_TO_TICKS(500U));
		}else
		if(temp_num == 0)		//对准目标抓取
		{
			s_retry_count = 0;
			if(upordownFlag == 0)	//抓地上
			{
				get_dis();
				vTaskDelay(pdMS_TO_TICKS(100U));
				/* extend_cm 内部会拆成 100 步平滑执行。 */
				float dis_diff_temp = TofData / 10.0f - 1.0f;
                ZhuaZi_open();		//爪子张开
                vTaskDelay(pdMS_TO_TICKS(800U));
				extend_cm(dis_diff_temp);//机械臂前移
	//			Serial5_Printf("Dis_diff=%.2f",dis_diff_temp);
             	vTaskDelay(pdMS_TO_TICKS(100U));
				ZhuaZi_close();		//爪子夹住
				vTaskDelay(pdMS_TO_TICKS(800U));
				Arm_put();			//放置果子
				App_NotifyGrabDone();//释放任务四 继续下一个点
			}
			if(upordownFlag == 1)	//抓树上
			{
				App_NotifyGrabDone();
			}
		} else if (temp_num == 5) //视觉系统判断当前水果不值得抓，机械臂复位并跳过
//视觉系统判断当前这个水果不值得抓（比如误识别、已被采摘、角度太偏无法抓取），就发 arm:5 指令让机械臂复位跳过，
		{
			s_retry_count = 0;
			if(upordownFlag == 0)
			{   Move_Pos(5.0f);
                vTaskDelay(pdMS_TO_TICKS(500U));
                Arm_ExtendZero();//伸缩归零
                vTaskDelay(pdMS_TO_TICKS(1000U));
				Arm_SetRotateAngle(0.0f);
                	vTaskDelay(pdMS_TO_TICKS(200U));
			App_NotifyGrabDone();
			}
			if(upordownFlag == 1)
			{
				// Arm_SetRotateAngle(0.0f);
//				vTaskDelay(500);
			App_NotifyGrabDone();
			}
		} else if (temp_num == 6)	//移除坏果
		{	
			if(upordownFlag == 0)	//抓地上
			{
				get_dis();
				vTaskDelay(pdMS_TO_TICKS(500U));
				/* extend_cm 内部会拆成 100 步平滑执行。 */
				float dis_diff_temp = TofData / 10.0f - 2.0f;
				extend_cm(dis_diff_temp);//机械臂前移
				ZhuaZi_close();		//爪子夹住
				vTaskDelay(pdMS_TO_TICKS(800U));
				// PosFlag == 0U ? Arm_SetRotateAngle(Rotate_Angle_Real + 30.0f) :
				//                  Arm_SetRotateAngle(Rotate_Angle_Real - 30.0f);
				vTaskDelay(pdMS_TO_TICKS(800U));
				ZhuaZi_open();
				vTaskDelay(pdMS_TO_TICKS(800U));
				Arm_put();
				vTaskDelay(pdMS_TO_TICKS(500U));
				App_NotifyGrabDone();
			}
			if(upordownFlag == 1)	//抓树上
			{
				// Arm_SetRotateAngle(0.0f);
				App_NotifyGrabDone();
			}
		}
    }
}

void ErWeiMa_func(void)
{
    if (memcmp(ret, "QR", 2) == 0) {
        float temp_num = 0.0f;
        char *p_num;
        uint8_t i = 0U;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%f", &temp_num);
            if (i < 8U) {
                fruits[i++] = (uint8_t)temp_num;
            }
        }

        fruits_count = i;
        CameraFlag = 1U;

    }
}

void Move_func(void)
{
    if ((memcmp(ret, UPPERCP_CMD_MOVE_GBK, 4) == 0) || (memcmp(ret, "move", 4) == 0)) {
        float temp_num = 0.0f;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%f", &temp_num);
            Serial5_Printf("move %.2f\r\n", temp_num);
        }

        TarPos = temp_num;
    }
}
