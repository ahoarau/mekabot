

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

enum dsp_state {
    DSP_OFF,
    DSP_STARTUP,
    DSP_PWM,
    DSP_CURRENT,
    DSP_BRAKE,
    DSP_ERROR
};



void step_state();
enum dsp_state get_dsp_state();
int limit_check(int item, int compare, int* count, int count_max );

#endif
