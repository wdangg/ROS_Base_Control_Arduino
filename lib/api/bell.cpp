#include "bell.h"

void bell_ring(uint8_t bell_pin, uint8_t bell_pending, uint8_t allow_ring, uint32_t pre_bell)
{
    if (bell_pending == true)
    {
        if (allow_ring == true)
        {
            if (((millis() / 1000) - pre_bell) <= 3)
            {
                digitalWrite(bell_pin, HIGH);
            }
            else
            {
                digitalWrite(bell_pin, LOW);
                bell_pending = false;
            }
        }
    }
}



