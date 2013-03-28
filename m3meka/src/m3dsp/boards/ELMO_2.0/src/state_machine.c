

#include "setup.h"



static enum dsp_state dsp = DSP_OFF;
static int16_t wd_cnt = 0;
static int16_t last_status = 0xFFFF;
int max_current_ma = 13000;
int max_current_error_count_max = 10;
int max_amp_temperature_cC = 8000;
int max_amp_temperature_count_max = 10;

void step_state()
{
    static int count = 0;
    static int max_current_error_count = 0;
    static int max_amp_temperature_count = 0;

    if (limit_check(ABS(get_max_current_ma()), max_current_ma,
            &max_current_error_count, max_current_error_count_max) )
        dsp = DSP_ERROR;

    if (limit_check(get_temperature_cC(ADC_AMP_TEMP), max_amp_temperature_cC,
            &max_amp_temperature_count, max_amp_temperature_count_max) )
        dsp = DSP_ERROR;
    

    if (count<1000) {
        count++;
        dsp = DSP_STARTUP;
    } else if (dsp != DSP_ERROR) { // latch in the error mode


        // ERROR CHECKING STUFF:
	wd_cnt++;
	if ((ec_cmd.command[0].config & M3ACT_CONFIG_EC_WD)  != last_status)		// if the status changes, everything is cool
	{
		wd_cnt = 0;
		last_status = ec_cmd.command[0].config & M3ACT_CONFIG_EC_WD;
	}
	else if (wd_cnt > 10)					// if the status doesn't change for some cycles, we're hosed
	{
		ec_cmd.command[0].mode = DSP_OFF;
	}

        switch (ec_cmd.command[0].mode) {
            case 0:
                dsp = DSP_OFF;
                break;
            case 1:
                dsp = DSP_PWM;
                break;
            case 2:
                dsp = DSP_OFF;
                break;
            case 3:
                dsp = DSP_CURRENT;
                break;
            case 4:
            default:
                dsp = DSP_BRAKE;
                break;
        }
    }

    switch (dsp) {
        case DSP_OFF:
            set_current_command_ma(0);
            set_pwm_desired(0);
            set_bldc_open();
            break;
        case DSP_STARTUP:
            set_current_command_ma(0);
            set_pwm_desired(0);
            set_bldc_open();
            set_adc_zeros();
            break;
        case DSP_PWM:
            set_current_command_ma(0);
            set_pwm_desired(ec_cmd.command[0].pwm_desired);
            set_bldc_commutation();
            break;
        case DSP_CURRENT:
            set_current_command_ma(ec_cmd.command[0].current_desired);
            set_pwm_desired(0);
            set_bldc_commutation();
            break;
        case DSP_BRAKE:
            set_current_command_ma(0);
            set_pwm_desired(0);
            set_bldc_brake();
            break;
        case DSP_ERROR:
        default:
            set_current_command_ma(0);
            set_pwm_desired(0);
            set_bldc_open();
            break;
    }
}

enum dsp_state get_dsp_state()
{
    return dsp;
}

int limit_check(int item, int compare, int* count, int count_max )
{
    if (item > compare) {
        *count++;
        *count = CLAMP(*count,0, count_max);
    } else
        *count = 0;

    return (*count >= count_max) ? 1 : 0;
}

