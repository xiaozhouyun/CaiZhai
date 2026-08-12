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

volatile position_t g_robot_pos = {-2600.0f, 500.0f, 0.0f};
volatile bool g_enable_auto_reverse;
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

int8_t Navigation_Request(float x, float y, float yaw)
{
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

void Move_Pos(float pos)
{
    now_pos = pos;
}

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
    static const float expected_y[8] = {
        2150.0f, 1950.0f, 1700.0f, 1500.0f,
        1200.0f, 1000.0f, 700.0f, 500.0f
    };
    static const float expected_gimbal[8] = {
        90.0f, -90.0f, 90.0f, -90.0f,
        90.0f, -90.0f, 90.0f, -90.0f
    };
    uint8_t i;

    App_Init();
    s_app_running = true;
    App_SetMode(APP_MODE_ROUTE_B);

    App_RunCurrentMode();
    AssertRequest(0U, -2600.0f, 10.0f, PI / 2.0f);
    ArriveAtLastRequest();
    App_RunCurrentMode();
    AssertRequest(1U, -1500.0f, 10.0f, 0.0f);
    ArriveAtLastRequest();
    App_RunCurrentMode();
    AssertRequest(2U, -1500.0f, 2350.0f, PI);
    ArriveAtLastRequest();
    App_RunCurrentMode();
    App_RunCurrentMode();

    assert(s_scan_count == 0U);
    assert(s_request_count == 4U);

    for (i = 0U; i < 8U; i++) {
        uint8_t request_index = (uint8_t)(3U + i);
        uint8_t gimbal_before = s_gimbal_count;
        uint8_t send_before = s_send_count;

        AssertRequest(request_index, -1500.0f, expected_y[i], PI);
        CompleteCurrentWorkPoint();
        assert(s_gimbal_count == (uint8_t)(gimbal_before + 1U));
        assert(NearlyEqual(s_gimbal_angles[gimbal_before], expected_gimbal[i]));
        assert(s_send_count == (uint8_t)(send_before + 1U));
    }

    assert(s_request_count == 11U);
    App_RunCurrentMode();
    AssertRequest(11U, -1500.0f, 10.0f, PI / 2.0f);
    ArriveAtLastRequest();
    App_RunCurrentMode();
    AssertRequest(12U, 0.0f, 0.0f, PI / 2.0f);
    ArriveAtLastRequest();
    App_RunCurrentMode();
    assert(g_app_mode == APP_MODE_IDLE);
    assert(!App_IsRunning());

    puts("app_route_b_test: PASS");
    return 0;
}
