#ifndef __MOTOR_ACTION_H__
#define __MOTOR_ACTION_H__

#include "Arduino.h"

#define L_CW                                    HIGH 
#define L_CCW                                   LOW 
#define R_CW                                    LOW 
#define R_CCW                                   HIGH

void enable_motor(uint8_t en_pin, uint8_t value);
void dir_motor(uint8_t en_pin, bool dir);

#endif /* __MOTOR_ACTION_H__ */