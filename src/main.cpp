#include "head.h"

#define BASE_LINNEAR_X (0.25)
#define BASE_ANGULAR_Z (0.8)

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

volatile uint32_t system_timeout = 0;
volatile bool manual_mode = false;
volatile bool reset_board = false;
volatile uint32_t last_cmd_receive = 0;

uint8_t bell_pending = false;
uint32_t pre_bell = 0;
uint8_t allow_ring = true;

void ps2_control();

void enable_left_motor(bool ena);
void enable_right_motor(bool ena);
void dir_left_motor(bool dir);
void dir_right_motor(bool dir);
void control_step(float tl, float tr);
bool get_vel();
void set_timer();
void control_motor(float v, float w);

void setup()
{
    Serial.begin(115200);
    Serial1.begin(115200);

    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, PRESSURES, RUMBLE);

    set_timer();

    pinMode(ENA_PIN_LEFT, OUTPUT);
    pinMode(DIR_PIN_LEFT, OUTPUT);
    pinMode(PUL_PIN_LEFT, OUTPUT);
    pinMode(ENA_PIN_RIGHT, OUTPUT);
    pinMode(DIR_PIN_RIGHT, OUTPUT);
    pinMode(PUL_PIN_RIGHT, OUTPUT);
    pinMode(PUL_PIN_RIGHT, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(BELL_PIN, OUTPUT);
    pinMode(CATHODE_LED, OUTPUT);
    pinMode(ANODE_LED, OUTPUT);

    digitalWrite(CATHODE_LED, LOW);
    digitalWrite(ANODE_LED, LOW);
    digitalWrite(BELL_PIN, LOW);

    enable_motor(ENA_PIN_LEFT, false);
    enable_motor(ENA_PIN_RIGHT, false);

    bell_pending = false;
    manual_mode = false;
    pre_bell = 0;
    allow_ring = true;

} /* SETUP */

void loop()
{
    if (Serial.available() > 0)
    {
        bool rs = get_vel();
        if (rs)
        {
            control_motor(twist_v, twist_w);
        }
    }

    bell_ring(BELL_PIN, bell_pending, allow_ring, pre_bell);

} /*END_LOOP*/

void control_motor(float v, float w)
{
    w *= (-1);
    float vl = (2 * v + w * LENGTH) / 2;
    float vr = (2 * v - w * LENGTH) / 2;

    if (abs(vl) > V_MAX)
    {
        vl = sgn(vl) * V_MAX;
    }
    if (abs(vr) > V_MAX)
    {
        vr = sgn(vr) * V_MAX;
    }

    if (abs(vl) < V_MIN)
    {
        enable_motor(ENA_PIN_LEFT, false);
    }
    else
    {
        enable_motor(ENA_PIN_LEFT, true);
    }
    if (abs(vr) < V_MIN)
    {
        enable_motor(ENA_PIN_RIGHT, false);
    }
    else
    {
        enable_motor(ENA_PIN_RIGHT, true);
    }
    if ((abs(vl) < V_MIN) && (abs(vr) < V_MIN))
    {
        return;
    }

    // Time of a half of pulse
    float Thl = (PI * RADIUS * 1000000) / (vl * PPR); // in us
    float Thr = (PI * RADIUS * 1000000) / (vr * PPR); // in us

    control_step(Thl / TIMER_TICK, Thr / TIMER_TICK);
} /* CONTROL_MOTOR */


// void dir_left_motor(bool dir)
// {
//     if (dir)
//     {
//         digitalWrite(DIR_PIN_LEFT, L_CW);
//     }
//     else
//     {
//         digitalWrite(DIR_PIN_LEFT, L_CCW);
//     }
// } /* DIR_LEFT_MOTOR */

// void dir_right_motor(bool dir)
// {
//     if (dir)
//     {
//         digitalWrite(DIR_PIN_RIGHT, R_CW);
//     }
//     else
//     {
//         digitalWrite(DIR_PIN_RIGHT, R_CCW);
//     }
// } /* DIR_RIGHT_MOTOR */

void control_step(float tl, float tr)
{
    if (tl > 0)
    {
        dir_motor(ENA_PIN_LEFT ,false);
        pulse_max_left = (int)tl;
    }
    else
    {
        dir_motor(ENA_PIN_RIGHT ,true);
        pulse_max_left = (int)(-tl);
    }
    if (tr > 0)
    {
        dir_motor(ENA_PIN_RIGHT ,false);
        pulse_max_right = (int)tr;
    }
    else
    {
        dir_motor(ENA_PIN_RIGHT ,true);
        pulse_max_right = (int)(-tr);
    }
} /* CONTROL_STEP */

void set_timer()
{
    cli(); /* Disable Global Interrupts */

    /* Reset Timer/Counter1 */
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 = 0;

    /* Setup Timer/Counter1 */
    // prescale = 8 and CTC mode 4
    TCCR1B |= (1 << WGM12) | (1 << CS11);
    // initialize OCR1A
    OCR1A = TIMER_TICK;
    TIMSK1 = (1 << OCIE1A); // Output Compare Interrupt Enable Timer/Counter1 channel A
    sei();                  /* Enable Global Interrupts */
} /* SET_TIMER */

ISR(TIMER1_COMPA_vect)
{
    if (pulse_counter_left > pulse_max_left)
    {
        pulse_counter_left = 0;
        if (pulse_on_left)
        {
            pulse_on_left = 0;
        }
        else
        {
            pulse_on_left = 1;
        }
    }
    if (pulse_counter_right > pulse_max_right)
    {
        pulse_counter_right = 0;
        if (pulse_on_right)
        {
            pulse_on_right = 0;
        }
        else
        {
            pulse_on_right = 1;
        }
    }
    if (pulse_on_left)
    {
        digitalWrite(PUL_PIN_LEFT, HIGH);
        digitalWrite(LED_BUILTIN, HIGH);
    }
    else
    {
        digitalWrite(PUL_PIN_LEFT, LOW);
        digitalWrite(LED_BUILTIN, LOW);
    }
    if (pulse_on_right)
    {
        digitalWrite(PUL_PIN_RIGHT, HIGH);
    }
    else
    {
        digitalWrite(PUL_PIN_RIGHT, LOW);
    }
    pulse_counter_left++;
    pulse_counter_right++;
} /* ISR_TIMER1_COMPA_vect */

bool get_vel()
{
    char_temp = Serial.read();

    if (char_temp == ']')
    {
        is_processing = 0;
        //
        c_token = strtok((char *)buf_temp, ",");
        if (c_token != NULL)
        {
            twist_v = atof(c_token);
            c_token = strtok(NULL, ",");
            if (c_token != NULL)
            {
                twist_w = atof(c_token);
            }
        }

        /*
        if ((twist_v == 0) && (twist_w == 0))
        {
            bell_pending = true;
        }
        else
        {
            bell_pending = false;
            pre_bell = (millis() / 1000);
        }
        */

        // TODO: Cal encoder

        return 1;
    }
    if (is_processing)
    {
        buf_temp[vel_idx++] = char_temp;
    }
    if (char_temp == '[')
    {
        vel_idx = 0;
        is_processing = 1;
    }
    return 0;
} /* GET_VEL */

void ps2_control()
{
    digitalWrite(ANODE_LED, HIGH);
    if (ps2x.ButtonPressed(PSB_TRIANGLE))
    {
        control_motor(BASE_LINNEAR_X, 0);
    }
    else if (ps2x.ButtonPressed(PSB_CIRCLE))
    {
        control_motor(0, -BASE_ANGULAR_Z);
    }
    else if (ps2x.ButtonPressed(PSB_SQUARE))
    {
        control_motor(0, BASE_ANGULAR_Z);
    }
    else if (ps2x.ButtonPressed(PSB_CROSS))
    {
        control_motor(-BASE_LINNEAR_X, 0);
    }
    else if (ps2x.ButtonReleased(PSB_TRIANGLE) || ps2x.ButtonReleased(PSB_CIRCLE) || ps2x.ButtonReleased(PSB_SQUARE) || ps2x.ButtonReleased(PSB_CROSS))
    {
        control_motor(0, 0);
    }
    else if (ps2x.ButtonPressed(PSB_PAD_UP))
    {
        digitalWrite(BELL_PIN, HIGH);
    }
    else if (ps2x.ButtonPressed(PSAB_PAD_DOWN))
    {
        digitalWrite(BELL_PIN, LOW);
    }
} /* PS2_CONTROL */
