#ifndef TEST_NAVIGATION_H
#define TEST_NAVIGATION_H

struct move {
    float tar;
    float real;
    float diff;
};

extern struct move speed;
extern struct move angle_speed;
extern int TarAngle;
extern float TarPos;

#endif
