

#include "temperature_model.h"

static int temperature;

/* model is first order RC, tf -> T/P = R/(tau*s+1)
 P = i^2*r, tau = R*C
 step response T = P*R*(1-exp(-t/tau))
 non homogeneous model T = exp(-t/tau)*T + R*P*(1-exp(-t/tau)
 */

// tau = 4;  // s
// R = 72; // K/W or is it 3.5?

#define EAT 65372 // exp(-dt/tau) = exp(-.01/4) =
#define EAT_SHIFT 16
//#define R_1EAT  51829// R*(1-exp(-dt/tau)) = 72*(1-eat) -> 1.582
//#define R_1EAT_SHIFT 15
#define R_1EAT  40317// R*(1-exp(-dt/tau)) =  3.5-> .0769
#define R_1EAT_SHIFT 16//19

#include "setup.h"

void init_temperature_model()
{
    temperature = 0;
}

// call this at 100 Hz
int step_temperature_model(int current_ma)
{
    // returns temperature, Q6 format
    int tmp = __builtin_mulsu(current_ma,16777)>>16;
    int current_square = __builtin_mulss(tmp,tmp)>>16;  // in A^2
    int term1 = __builtin_muluu(temperature,EAT)>>EAT_SHIFT;
    int term2 = __builtin_muluu(R_1EAT,current_square)>>R_1EAT_SHIFT;
    temperature = term1 + term2;

    tmp_debug = current_square;

    return get_model_temperature_cK();
}

int get_model_temperature_cK()
{
    // returns temperature in centikelvin
    return temperature;
}
