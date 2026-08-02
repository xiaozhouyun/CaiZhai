#include "UpperCP.h"
#include "app.h"
#include "navigation.h"
#include "usart.h"
#include "voice.h"

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
float Rotate_Angle_Real = 0.0f;  /**< 云台实际当前角度 (单位：度) */
uint8_t CameraFlag = 0;

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

    ret = strtok(uppercp_cmd_buf, ":");
    if (ret != NULL) {
        strncpy(uppercp_last_cmd, ret, sizeof(uppercp_last_cmd) - 1U);
        uppercp_last_cmd[sizeof(uppercp_last_cmd) - 1U] = '\0';
        cmd_func();
        Arm_func();//爪子对其
        voice_func();
        ErWeiMa_func();//二维码
        Move_func();//位移
        // angle_func();
        // face_func();
          // speed_func();
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

        *ret = 0;
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

        *ret = 0;
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

        *ret = 0;
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

        *ret = 0;
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

        *ret = 0;
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
        	if(temp_num == 1)		//目标偏右
		{
		    PCA9685_Set180Angle(3U,Rotate_Angle_Real+1);
//			Serial5_Printf("L_Angle_Tar=%f\r\n",Rotate_Angle_Real);
		}else
		if(temp_num == 2)		//目标偏左
		{
			PCA9685_Set180Angle(3U,Rotate_Angle_Real-1);
//			Serial5_Printf("L_Angle_Tar=%f\r\n",Rotate_Angle_Real);
		}else
		if(temp_num == 3)		//目标偏上
		{
			Move_up(1);
		}else
		if(temp_num == 4)		//目标偏下
		{
			Move_down(1);
		}
		if(temp_num == 0)		//对准目标抓取
		{
			if(StateFlag == 0)	//抓地上
			{
				get_dis();
				vTaskDelay(500);
				//计算长度
				float dis_diff_temp = (TofData/10.0-1)/100.0;
				
				uint8_t i=100;
				while(i--){
					extend_cm(dis_diff_temp);//机械臂前移
					vTaskDelay(15);
				}
	//			Serial5_Printf("Dis_diff=%.2f",dis_diff_temp);
				ZhuaZi_close();		//爪子夹住
				vTaskDelay(800);
				Arm_put();			//放置果子
				App_NotifyGrabDone();
			}
			if(StateFlag == 1)	//抓树上
			{
				App_NotifyGrabDone();
			}
		}else
		if(temp_num == 5)	
		{
			if(StateFlag == 0)
			{
				vTaskDelay(200);
				Move_Pos(22);
				vTaskDelay(2000);
				set_extend_cm(8);
				float angle_dif1=(0-Rotate_Angle_Real)/100;
				for(int i=0; i<100; i++){
					set_rotate_angle(Rotate_Angle_Real+angle_dif1);
					vTaskDelay(fabs(angle_dif1)*18);
				}
				set_rotate_angle(0);
//				vTaskDelay(500);
			App_NotifyGrabDone();
			}
			if(StateFlag == 1)
			{
				set_rotate_angle(0);
//				vTaskDelay(500);
			App_NotifyGrabDone();
			}
		}else
		if(temp_num == 6)	//移除坏果
		{	
			if(StateFlag == 0)	//抓地上
			{
				get_dis();
				vTaskDelay(500);
				//计算长度
				float dis_diff_temp = (TofData/10.0-2)/100.0;
				
				uint8_t i=100;
				while(i--){
					extend_cm(dis_diff_temp);//机械臂前移
					vTaskDelay(15);
				}
				ZhuaZi_close();		//爪子夹住
				vTaskDelay(800);
				ArmState==0 ?  set_rotate_angle(Rotate_Angle_Real+30):set_rotate_angle(Rotate_Angle_Real-30);
				vTaskDelay(800);
				ZhuaZi_open();
				vTaskDelay(800);
				Arm_put();
				vTaskDelay(500);
				App_NotifyGrabDone();
			}
			if(StateFlag == 1)	//抓树上
			{
				set_rotate_angle(0);
				App_NotifyGrabDone();
			}
		}
        *ret = 0;
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

        *ret = 0;
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
        *ret = 0;
    }
}
