#ifndef __BELL_H__
#define __BELL_H__

#include "Arduino.h"

void bell_ring(uint8_t bell_pin, uint8_t bell_pending, uint8_t allow_ring, uint32_t pre_bell);

#endif /* __BELL_H__ */