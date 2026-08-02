#include "vofa.h"
#include "usart.h"
#include <string.h>

#define VOFA_MAX_FLOATS 12U

void Vofa_SendFloat(const float *values, uint8_t count)
{
    static const uint8_t frame_tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
    uint8_t buffer[VOFA_MAX_FLOATS * sizeof(float) + sizeof(frame_tail)];
    uint16_t length;

    if (values == NULL || count == 0U || count > VOFA_MAX_FLOATS) {
        return;
    }

    length = (uint16_t)count * sizeof(float);
    memcpy(buffer, values, length);
    memcpy(&buffer[length], frame_tail, sizeof(frame_tail));
    (void)HAL_UART_Transmit(&huart6, buffer, length + sizeof(frame_tail), 100U);
}
