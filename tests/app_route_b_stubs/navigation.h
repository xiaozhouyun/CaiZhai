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

int8_t Navigation_Request(float target_x_mm, float target_y_mm, float target_yaw_rad,
                          float move_heading_bias_rad);
bool Navigation_IsIdle(void);
void Navigation_Stop(void);
void Navigation_Reset(float start_x_mm, float start_y_mm, float yaw_zero_deg);
void Navigation_SetY(float y_mm);
void Navigation_SetX(float x_mm);
void Chassis_SetSpeed(float linear_vel_mm_s, float angular_vel_rad_s);

#define NAV_START_CENTER_X_MM 0.0f
#define NAV_START_CENTER_Y_MM 0.0f

#endif
