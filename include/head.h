#ifndef __HEAD_H__
#define __HEAD_H__

#include <Arduino.h>
#include "PS2X_lib.h"
#include "board_action.h"
#include "motor_action.h"
#include "bell.h"

#define PRESSURES   							false
#define RUMBLE      							false

#define PS2_DAT        							11    
#define PS2_CMD        							10
#define PS2_SEL        							8
#define PS2_CLK        							9

#define CATHODE_LED                             A3
#define ANODE_LED                               A1

#define BELL_PIN                                A0

#define sgn(x)                                  ((x) < 0 ? -1 : 1)

#define ENA_PIN_LEFT                            2
#define DIR_PIN_LEFT                            3
#define PUL_PIN_LEFT                            4
#define ENA_PIN_RIGHT                           5
#define DIR_PIN_RIGHT                           6
#define PUL_PIN_RIGHT                           7

#define LOCK_TIMER_TICK                         1   // 2 * 0.5 us
#define TIMER_TICK                              (100 * LOCK_TIMER_TICK)   // us

#define PPR                                     1500
#define RADIUS                                  0.0475
#define LENGTH                                  0.23
#define PI                                      3.14

#define V_MAX                                   0.75    // m/s
#define V_MIN                                   0.01    // m/s

#define L_CW                                    HIGH 
#define L_CCW                                   LOW 
#define R_CW                                    LOW 
#define R_CCW                                   HIGH

volatile int char_temp = 0;    // for incoming serial data
int8_t vel_idx = 0;
volatile float twist_v = 0.0;
volatile float twist_w = 0.0;
char* c_token;
static int8_t is_processing = 0;
static int8_t buf_temp[20];

volatile int16_t pulse_max_right = 0;
volatile int16_t pulse_max_left = 0;
volatile int16_t pulse_counter_right = 0;
volatile int16_t pulse_counter_left = 0;
volatile int8_t pulse_on_right = 0;
volatile int8_t pulse_on_left = 0;

PS2X ps2x;
int8_t error = 0;
int8_t type = 0;
int8_t vibrate = 0;

void enable_left_motor(bool ena);
void enable_right_motor(bool ena);
void dir_left_motor(bool dir);
void dir_right_motor(bool dir);
void control_step(float tl, float tr);
bool get_vel();
void set_timer();
void control_motor(float v, float w);


#endif /* __HEAD_H__ */