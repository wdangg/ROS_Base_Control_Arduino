#include "motor_action.h"

void enable_motor(uint8_t en_pin, uint8_t value)
{
    if (value)
    {
        digitalWrite(en_pin, LOW);
    }
    else
    {
        digitalWrite(en_pin, HIGH);
    }
}

void dir_motor(uint8_t en_pin, bool dir)
{
    if (dir)
    {
        digitalWrite(en_pin, L_CW);
    }
    else
    {
        digitalWrite(en_pin, L_CCW);
    }
} /* DIR_LEFT_MOTOR */

