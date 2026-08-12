#ifndef TEST_NAVIGATION_H
#define TEST_NAVIGATION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float x;
    float y;
    float yaw;
} position_t;

extern volatile position_t g_robot_pos;
extern volatile bool g_enable_auto_reverse;

int8_t Navigation_Request(float target_x_mm, float target_y_mm, float target_yaw_rad);
bool Navigation_IsIdle(void);
void Navigation_Stop(void);

#endif
