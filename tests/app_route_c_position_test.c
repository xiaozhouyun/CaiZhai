#include "app.h"
#include "navigation.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

static uint32_t s_tick;
static bool s_navigation_idle = true;
static char s_tasks[16][8];
static uint8_t s_task_count;

volatile position_t g_robot_pos = {-2600.0f, 10.0f, 0.0f};
volatile bool g_enable_auto_reverse;
float now_pos = 27.0f;
uint8_t CameraFlag;
uint8_t fruits[8];
uint8_t fruits_count;

uint32_t HAL_GetTick(void) { return s_tick; }

int8_t Navigation_Request(float x, float y, float yaw)
{
    g_robot_pos.x = x;
    g_robot_pos.y = y;
    g_robot_pos.yaw = yaw * 180.0f / PI;
    s_navigation_idle = false;
    return 0;
}

bool Navigation_IsIdle(void) { return s_navigation_idle; }
void Navigation_Stop(void) { s_navigation_idle = true; }
void Move_Pos(float pos) { now_pos = pos; }
void Tiancan_RxByte(uint8_t data) { (void)data; }
void Emm_GetTxStatus(void *status) { (void)status; }
void Voice_Num(int value) { (void)value; }
void PCA9685_Set270Angle(float angle) { (void)angle; }
float PCA9685_Get180Angle(uint8_t channel) { (void)channel; return 0.0f; }
void UpperCP_ResetQrResult(void) { CameraFlag = 0U; fruits_count = 0U; }
void ActionScheduler_Cancel(void) {}
bool ActionScheduler_IsGimbalBusy(void) { return false; }
void ActionScheduler_StartGimbalMove(float target_angle, uint32_t duration_ms)
{
    (void)target_angle;
    (void)duration_ms;
}

void UpperCP_SendTask(const char *task)
{
    assert(s_task_count < (uint8_t)(sizeof(s_tasks) / sizeof(s_tasks[0])));
    assert(strlen(task) < sizeof(s_tasks[0]));
    strcpy(s_tasks[s_task_count++], task);
}

static void RunRoute(const uint8_t positions[8])
{
    uint32_t guard;

    s_tick = 0U;
    s_navigation_idle = true;
    s_task_count = 0U;
    now_pos = 27.0f;
    App_Init();
    s_app_running = true;
    assert(App_RouteC_PlanAndRun(positions, APP_MODE_IDLE) == 0);

    for (guard = 0U; guard < 500U && App_IsRunning(); ++guard) {
        uint8_t task_count_before = s_task_count;

        if (!s_navigation_idle) {
            s_navigation_idle = true;
        }
        s_tick += 5000U;
        App_RunCurrentMode();
        if (s_task_count > task_count_before) {
            App_NotifyGrabDone();
        }
    }

    assert(guard < 500U);
    assert(s_task_count == 8U);
}

static uint8_t ParsePosition(const char *task)
{
    unsigned int position = 0U;
    assert(sscanf(task, "send:%u", &position) == 1);
    assert(position >= 1U && position <= 12U);
    return (uint8_t)position;
}

int main(void)
{
    static const uint8_t paired_positions[8] = {1U, 5U, 2U, 6U, 3U, 7U, 4U, 8U};
    static const char *expected_tasks[8] = {
        "send:4", "send:8", "send:3", "send:7",
        "send:2", "send:6", "send:1", "send:5"
    };
    static const uint8_t spread_positions[8] = {1U, 2U, 3U, 4U, 9U, 10U, 11U, 12U};
    bool seen[13] = {false};
    uint8_t i;

    RunRoute(paired_positions);
    for (i = 0U; i < 8U; ++i) {
        assert(strcmp(s_tasks[i], expected_tasks[i]) == 0);
    }

    RunRoute(spread_positions);
    for (i = 0U; i < 8U; ++i) {
        uint8_t position = ParsePosition(s_tasks[i]);
        assert(!seen[position]);
        seen[position] = true;
    }
    for (i = 0U; i < 8U; ++i) {
        assert(seen[spread_positions[i]]);
    }

    puts("app_route_c_position_test: PASS");
    return 0;
}
