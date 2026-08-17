#include "app.h"
#include "navigation.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

typedef struct {
    float x;
    float y;
    float yaw;
} NavRequest_t;

static uint32_t s_tick;
static bool s_navigation_idle = true;
static NavRequest_t s_requests[32];
static uint8_t s_request_count;
static float s_gimbal_angles[16];
static uint8_t s_gimbal_count;
static uint8_t s_send_count;
static uint8_t s_scan_count;
static float s_last_linear_speed;

volatile position_t g_robot_pos = {-2600.0f, 500.0f, 0.0f};
volatile bool g_enable_auto_reverse;
volatile float TofData;
volatile uint32_t TofFrameSeq;
volatile float g_hwt101_yaw;
float now_pos = 27.0f;
uint8_t CameraFlag;
uint8_t fruits[8] = {4U, 3U, 1U, 10U, 8U, 9U, 2U, 11U};
uint8_t fruits_count;

static bool NearlyEqual(float actual, float expected)
{
    return fabsf(actual - expected) < 0.01f;
}

static void AssertRequest(uint8_t index, float x, float y, float yaw)
{
    assert(index < s_request_count);
    assert(NearlyEqual(s_requests[index].x, x));
    assert(NearlyEqual(s_requests[index].y, y));
    assert(NearlyEqual(s_requests[index].yaw, yaw));
}

static void ArriveAtLastRequest(void)
{
    assert(s_request_count > 0U);
    g_robot_pos.x = s_requests[s_request_count - 1U].x;
    g_robot_pos.y = s_requests[s_request_count - 1U].y;
    g_robot_pos.yaw = s_requests[s_request_count - 1U].yaw * 180.0f / PI;
    s_navigation_idle = true;
}

static void AdvanceTime(void)
{
    s_tick += 5000U;
}

static void CompleteCurrentWorkPoint(void)
{
    ArriveAtLastRequest();
    App_RunCurrentMode();
    AdvanceTime();
    App_RunCurrentMode();
    AdvanceTime();
    App_RunCurrentMode();
    AdvanceTime();
    App_RunCurrentMode();
    assert(s_send_count > 0U);
    App_NotifyGrabDone();
    AdvanceTime();
    App_RunCurrentMode();
}

uint32_t HAL_GetTick(void)
{
    return s_tick;
}

int8_t Navigation_Request(float x, float y, float yaw, float move_heading_bias_rad)
{
    (void)move_heading_bias_rad;
    assert(s_navigation_idle);
    assert(s_request_count < (uint8_t)(sizeof(s_requests) / sizeof(s_requests[0])));
    s_requests[s_request_count++] = (NavRequest_t){x, y, yaw};
    s_navigation_idle = false;
    return 0;
}

bool Navigation_IsIdle(void)
{
    return s_navigation_idle;
}

void Navigation_Stop(void)
{
    s_navigation_idle = true;
}

void Navigation_Reset(float x, float y, float yaw_zero_deg)
{
    g_robot_pos.x = x;
    g_robot_pos.y = y;
    g_robot_pos.yaw = 0.0f;
    g_hwt101_yaw = yaw_zero_deg;
    s_navigation_idle = true;
}

void Navigation_SetY(float y)
{
    g_robot_pos.y = y;
}

void Navigation_SetX(float x)
{
    g_robot_pos.x = x;
}

void Chassis_SetSpeed(float linear_vel_mm_s, float angular_vel_rad_s)
{
    s_last_linear_speed = linear_vel_mm_s;
    (void)angular_vel_rad_s;
}

void Move_Pos(float pos)
{
    now_pos = pos;
}

void ZhuaZi_open(void) {}
void ZhuaZi_close(void) {}

void Tiancan_RxByte(uint8_t data)
{
    (void)data;
}

void Emm_GetTxStatus(void *status)
{
    (void)status;
}

void Voice_Num(int value)
{
    (void)value;
}

void PCA9685_Set270Angle(float angle)
{
    (void)angle;
}

float PCA9685_Get270Angle(void)
{
    return 35.0f;
}

int32_t PCA9685_Set180Angle(uint8_t channel, float angle)
{
    (void)channel;
    (void)angle;
    return 0;
}

float PCA9685_Get180Angle(uint8_t channel)
{
    (void)channel;
    return 0.0f;
}

void UpperCP_ResetQrResult(void)
{
    CameraFlag = 0U;
    fruits_count = 0U;
}

uint32_t UpperCP_GetRxCount(void)
{
    return 0U;
}

void vTaskDelay(uint32_t ticks)
{
    s_tick += ticks;
}

void UpperCP_SendTask(const char *task)
{
    if (strcmp(task, "send") == 0) {
        s_send_count++;
    } else if (strcmp(task, "scan") == 0) {
        s_scan_count++;
    }
}

void ActionScheduler_Cancel(void)
{
}

bool ActionScheduler_IsGimbalBusy(void)
{
    return false;
}

void ActionScheduler_StartGimbalMove(float target_angle, uint32_t duration_ms)
{
    (void)duration_ms;
    assert(s_gimbal_count < (uint8_t)(sizeof(s_gimbal_angles) / sizeof(s_gimbal_angles[0])));
    s_gimbal_angles[s_gimbal_count++] = target_angle;
}

int main(void)
{
    App_Init();
    s_app_running = true;
    App_SetMode(APP_MODE_ROUTE_B);

    App_RunCurrentMode();
    AssertRequest(0U, -2600.0f, 0.0f, PI / 2.0f);
    ArriveAtLastRequest();
    App_RunCurrentMode();
    AssertRequest(1U, -950.0f, 0.0f, PI / 2.0f);
    ArriveAtLastRequest();
    App_RunCurrentMode();

    /* 到达 B 区入口时保持 +90°，然后单独转到 0°。 */
    assert(g_app_mode == APP_MODE_CALIBRATE_B);
    App_RunCurrentMode();
    AssertRequest(2U, -950.0f, 0.0f, 0.0f);
    ArriveAtLastRequest();
    App_RunCurrentMode();
    App_RunCurrentMode();

    /* 200mm 校准需要新帧，距离过大时先倒车，连续两帧达标后才置 Y=0。 */
    TofData = 260.0f;
    TofFrameSeq++;
    App_RunCurrentMode();
    assert(NearlyEqual(s_last_linear_speed, -50.0f));
    g_robot_pos.y = -25.0f;
    TofData = 200.0f;
    TofFrameSeq++;
    App_RunCurrentMode();
    TofFrameSeq++;
    App_RunCurrentMode();
    assert(NearlyEqual(g_robot_pos.y, 0.0f));

    /* Y 标定后转到 -90°，再执行 1250mm 的 X 轴校准。 */
    App_RunCurrentMode();
    AssertRequest(3U, -950.0f, 0.0f, -PI / 2.0f);
    ArriveAtLastRequest();
    App_RunCurrentMode();
    App_RunCurrentMode();
    g_robot_pos.x = -920.0f;
    TofData = 1250.0f;
    TofFrameSeq++;
    App_RunCurrentMode();
    TofFrameSeq++;
    App_RunCurrentMode();
    assert(NearlyEqual(g_robot_pos.x, -950.0f));

    /* 双轴校准完成后才继续原路线第 3 点和 B 区作业路线。 */
    AssertRequest(4U, -950.0f, 2300.0f, PI);
    ArriveAtLastRequest();
    App_RunCurrentMode();
    App_RunCurrentMode();
    AssertRequest(5U, -950.0f, 2150.0f, PI);
    assert(g_app_mode == APP_MODE_SCAN_B);
    assert(App_IsRunning());
    assert(s_scan_count == 0U);

    puts("app_route_b_test: PASS");
    return 0;
}
