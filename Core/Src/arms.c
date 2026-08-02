#include "arms.h"
#include "pca9685.h"
#include "bujin.h"
#include "UpperCP.h"

#define L0 1.05f
#define L1 7.2f	//6.2 越小越长
#define L2 8.4f	//7.0 越小越长
/* 当前升降位置，单位 cm；Move_up/Move_down/Move_Pos 会维护这个值。 */
float now_pos = 0.0f;

/**
  * @brief  升降机构上升指定距离
  * @param  Data_cm 上升距离，单位 cm
  * @note   5 是旧工程使用的升降步进电机地址；
  *         dir=0 表示上升方向；
  *         snF=true 表示先缓存运动命令，随后用同步命令触发。
  */
void Move_up(float Data_cm)
{
    Emm_V5_Pos_Control(5, 0, 200, 200, Data_cm * 10.0f, false, true);
    Emm_V5_Synchronous_motion(0);
    now_pos += Data_cm;
}

/**
  * @brief  升降机构下降指定距离
  * @param  Data_cm 下降距离，单位 cm
  * @note   dir=1 表示下降方向；运动完成后同步更新 now_pos。
  */
void Move_down(float Data_cm)
{
    Emm_V5_Pos_Control(5, 1, 200, 200, Data_cm * 10.0f, false, true);
    Emm_V5_Synchronous_motion(0);
    now_pos -= Data_cm;
}

/**
  * @brief  移动到目标高度位置
  * @param  Tar_pos 目标位置，单位 cm
  * @note   函数会用 Tar_pos - now_pos 算出相对移动距离：
  *         结果为正就上升，结果为负就下降。
  */
void Move_Pos(float Tar_pos)
{
    float move_pos = Tar_pos - now_pos;

    if (move_pos > 0.0f)
    {
        Move_up(move_pos);
    }
    else
    {
        Move_down(-move_pos);
    }

    now_pos = Tar_pos;
}

/**
  * @brief  机械臂伸缩控制函数
  * @param  dist_cm 伸缩距离，单位 cm
  * @note   用于控制机械臂水平方向的伸出与缩回。
  *         内部包含距离至舵机角度的转换预留及 [-80°, +40°] 角度安全限幅保护。
  */
void extend_cm(float dist_cm)
{
    float angle_deg;

    /* 1. 距离(cm)转舵机角度(deg)转换预留位（待公式确定后在此填充转换逻辑） */
    dist_cm = dist_cm/2.0;
	float theta1 = atanf(L0/dist_cm);
	float long1 = sqrt(dist_cm*dist_cm + L0*L0);
	float theta2 = acosf((L2*L2 + long1*long1 - L1*L1)/(2*L2*long1));
	float theta = (theta1 + theta2)*2;
	angle_deg = theta*180/3.1415926 ;

    /* 2. 角度限幅保护：严格限制在 [-80.0f, +40.0f] 范围内 */
    if (angle_deg > 40.0f)
    {
        angle_deg = 40.0f;
    }
    else if (angle_deg < -80.0f)
    {
        angle_deg = -80.0f;
    }

    /* 3. 平滑驱动舵机 (通道 1) 到目标角度 */
    PCA9685_Set180AngleSmooth(1U, angle_deg, 100U, 10U);
}

/**
  * @brief  爪子闭合/抓取控制函数
  * @note   用于驱动末端夹爪执行闭合动作以抓取目标对象
  */
void ZhuaZi_close(void)
{
    /* 爪子闭合/抓取控制逻辑实现预留 */
       PCA9685_Set180AngleSmooth(4U, 10, 100U, 10U);
}

/**
  * @brief  机械臂放置果子控制函数
  * @note   用于驱动机械臂与夹爪完成果子的松开与放置动作
  */
void Arm_put(void)
{
    /* 机械臂放置果子控制逻辑实现预留 */
    //缩回抬升后旋转
	Move_Pos(26);
	vTaskDelay(200);
	PCA9685_Set180AngleSmooth(3U, -80, 100U, 10U);//回中
	vTaskDelay(1000);
	PCA9685_Set180AngleSmooth(1U, 0, 100U, 10U);//回中
	//开爪
	ZhuaZi_open();
	vTaskDelay(800);
}

/**
  * @brief  爪子打开/松开控制函数
  * @note   用于驱动末端夹爪张开以释放目标对象
  */
void ZhuaZi_open(void)
{
    /* 爪子张开/释放控制逻辑实现预留 */
      PCA9685_Set180AngleSmooth(4U, -30, 100U, 10U);
}
