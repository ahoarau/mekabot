
#include "setup.h"
#include "current.h"

static int current_meas;
static int current_desired;
static int i_a, i_b;

#define MEAS_A &i_a
#define MEAS_B &i_b
static const int * current_sensor[8] = {MEAS_A, MEAS_A, MEAS_B, MEAS_B,
                                         MEAS_B, MEAS_A, MEAS_A, MEAS_A};
static const int current_signs[8] = {0,1,1,1,-1,1,-1,0};


int current_control()
{
    static long ki_sum = 0;
    int error = current_desired-current_meas;
    int command;
    const int kp = ec_cmd.command[0].k_p;
    const int kp_shift = ec_cmd.command[0].k_p_shift;
    const int ki = ec_cmd.command[0].k_i;
    const int ki_shift = ec_cmd.command[0].k_i_shift;
    const long ki_limit = (long) ec_cmd.command[0].k_i_limit<<6;

//    current_meas = current_reading;

    ki_sum += ki*error;
    ki_sum = CLAMP(ki_sum,-ki_limit,ki_limit);
    command = (__builtin_mulss(-kp,error)>>(6+kp_shift)) - (ki_sum>>(6+ki_shift));

    return (command);
}

int get_current_ma()
{
    //return (current_meas);
    return adc2ma(current_meas);
}

int get_max_current_ma()
{
    // return the max of the two current sensors
    int i = MAX(ABS(i_a),ABS(i_b));

    return adc2ma(i);
}

void set_current_command_ma(int current_desired_ma)
{
    // sets current_desired in shifted adc ticks
    current_desired = ma2adc(current_desired_ma);

}

void set_current_ab(int current_a, int current_b)
{
    int hall_state = get_hall_state();
    i_a = current_a;
    i_b = current_b;

    current_meas = *current_sensor[hall_state]*current_signs[hall_state];
}

int adc2ma(int adc)
{
    return (__builtin_mulsu(adc,CURRENT_MA_MULT)>>CURRENT_MA_SHIFT);
}

int ma2adc(int ma)
{
    return (__builtin_mulsu(ma,CURRENT_ADC_MULT)>>CURRENT_ADC_SHIFT);
}

