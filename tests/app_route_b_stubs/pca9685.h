#ifndef TEST_PCA9685_H
#define TEST_PCA9685_H

#include <stdint.h>

void PCA9685_Set270Angle(float angle);
float PCA9685_Get180Angle(uint8_t channel);

#endif
