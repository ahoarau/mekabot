 /*
Copyright (c) 2010, Meka Robotics
All rights reserved.
http://mekabot.com

Redistribution and use in source and binary forms, with or without
modification, are permitted. 


THIS SOFTWARE IS PROVIDED BY THE Copyright HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
Copyright OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifdef USE_ADC

#include "p33Fxxxx.h"
#include "setup.h"
#include "adc.h"


int adc_idx;
int adc_idx_fast;

static int16_t an_idx;
static int16_t an[ADC_NUM_CH];


unsigned int adc_raw[ADC_NUM_CH];
unsigned int volatile adc_buffer[ADC_NUM_CH][ADC_NUM_SMOOTH];
unsigned int volatile adc_buffer_fast[ADC_NUM_SMOOTH_FAST];
int irq_cnt; 
unsigned int wd_cnt = 0, last_status = 0, watchdog_expired;	//Watchdog

unsigned int get_avg_adc(int ch)
{
	long v;
	int i;
	v=0;
	for (i=0;i<ADC_NUM_SMOOTH;i++)
		v=v+adc_buffer[ch][i];
		
	return (unsigned int)(v>>ADC_SHIFT_SMOOTH);
}

unsigned int get_avg_adc_torque()
{
	int i;
	long v=0;
	for (i=0;i<ADC_NUM_SMOOTH_FAST;i++)
		v=v+adc_buffer_fast[i];
		
	return (unsigned int)(v>>ADC_SHIFT_SMOOTH_FAST);
}

void setup_adc(void) 
{
	adc_idx=0;
	adc_idx_fast=0;

        
        an[ADC_CURRENT_A] = 1;
        an[ADC_TEMP_BOARD] = 0;
	
        an[ADC_TEMP_AMB] = 6;        
        an[ADC_CURRENT_B] = 8;
        
	an_idx = 0;


	//Setup for current sensing
	// System clock divider TAD=(ADCS+1)*TCY==50ns (As fast as works...)
	// Auto Sample Time = 5*Tad, 4 conversions, (14+5)*4*TAD=68*50ns=3.8us

	AD1CON1bits.ADON = 0;			//Turn off ADC

	AD1CON3bits.SAMC = 5; 			// Change to 10?	//ToDo Original BMW = 10	
	AD1CON3bits.ADCS = 1;								//ToDo Original BMW = 40
	AD1CON2bits.VCFG = 0;           // Vref AVdd/AVss
	AD1CON3bits.ADRC = 0;			// ADC Clock is derived from Systems Clock
	AD1CON1bits.SSRC = 0b011;		// Manual StartOfConversion 0b000 //PWM: 0b011;
	AD1CON2bits.CHPS = 0;			// Only convert CH0	in 12-bit mode
	AD1CON1bits.ASAM = 1;			// Sampling begins immediately after conversion is done
	AD1CON1bits.AD12B = 1;			// 12-bit ADC operation
	AD1CON2bits.BUFM = 1;			// Use 2x8-word buffer for conversion sequences
	AD1CON1bits.SIMSAM = 0;			// No simultaneous sample for 1CH
	AD1CON2bits.CSCNA = 0;			// Enable channel scanning
	//AD1CON2bits.SMPI = 3;			// Select 4 conversions between interrupts
        AD1CON2bits.SMPI = 0;			// Select 4 conversions between interrupts
	//AD1CSSLbits.CSS0 = 1;
	AD1CSSLbits.CSS1 = 1;
	/*AD1CSSLbits.CSS6 = 1;
        AD1CSSLbits.CSS8 = 1;*/
	AD1PCFGL = 0xFFFF; 				// All digital by default
	AD1PCFGLbits.PCFG0 = 0;
	AD1PCFGLbits.PCFG1 = 0;
	AD1PCFGLbits.PCFG6 = 0;
        AD1PCFGLbits.PCFG8 = 0;
	AD1CON1bits.FORM = 0;			// Select results format Integer Output Format (0B 0000 dddd dddd dddd )
	
	AD1CON1bits.ADON = 1;			// Turn on ADC
	_AD1IF = 0;						// Enable interrupt
	_AD1IE = 1;
}


void __attribute__((__interrupt__, no_auto_psv)) _ADC1Interrupt(void)
{
	static unsigned int count = 0;
	
	_AD1IF = 0;		//Clear the flag

         if(an_idx == ADC_TEMP_BOARD)
         {
            adc_raw[ADC_TEMP_BOARD]=ADC1BUF0;
            adc_buffer[ADC_TEMP_BOARD][adc_idx] = adc_raw[ADC_TEMP_BOARD];
         } else if (an_idx == ADC_TEMP_AMB) {
            adc_raw[ADC_TEMP_AMB]=ADC1BUF0;
            adc_buffer[ADC_TEMP_AMB][adc_idx] = adc_raw[ADC_TEMP_AMB];
         } else if (an_idx == ADC_CURRENT_A) {
            adc_raw[ADC_CURRENT_A]=ADC1BUF0;
            adc_buffer[ADC_CURRENT_A][adc_idx] = adc_raw[ADC_CURRENT_A];
         } else if (an_idx == ADC_CURRENT_B) {
            adc_raw[ADC_CURRENT_B]=ADC1BUF0;
            adc_buffer[ADC_CURRENT_B][adc_idx] = adc_raw[ADC_CURRENT_B];
         }

//	adc_raw[0] = ADC1BUF0;
//	adc_raw[1] = ADC1BUF1;
//	adc_raw[2] = ADC1BUF2;
	
	
		

	
	//Timed actions - 2kHz
	//Originaly in timer3 ISR
	//======================
	
	count = INC_MOD(count,5);		
	//if(count == 0)
        if(0)
	{
		//Latch encoder timestamp on Rising edge.
		#ifdef USE_TIMESTAMP_DC			//Takes 0.05us to execute
		SetTimestampLatch;	
		ClrTimestampLatch;
		#endif
	
		#if defined USE_ENCODER_VERTX	//Takes 122us to execute
		//step_vertx();
		#endif
	
		#ifdef USE_CURRENT				//Takes 0.475us to execute
		step_current();
		#endif
	
		#ifdef USE_CONTROL				//Takes 2.3us to execute
		step_control();					//(will probably increase in PID mode...)
		#endif
		
		irq_cnt++;		
		
		#ifdef USE_WATCHDOG
		wd_cnt++;
		if ((ec_cmd.command[0].config & 0x4000) != last_status)		// if the WD bit changes, everything is cool
		{
			wd_cnt = 0;
			watchdog_expired = 0;
		}
		else if (wd_cnt > 500)					// if the status doesn't change in 250ms, problem
		{
			watchdog_expired = 1;
		}	
		last_status = (ec_cmd.command[0].config & 0x4000);
		#else
		watchdog_expired = 0;	//Always off
		#endif
		
		//Sum = 125us, 25% of this time slice
	}

        /////////////////////////////
        // ADC MANAGEMENT:
	an_idx++;
        if (an_idx >= ADC_NUM_CH)
        {
            an_idx = 0;
            adc_idx = INC_MOD(adc_idx,ADC_NUM_SMOOTH);
        }

        if (an_idx == ADC_TEMP_BOARD)
        {
            AD1CSSLbits.CSS0 = 1;
            AD1CSSLbits.CSS1 = 0;
            AD1CSSLbits.CSS6 = 0;
            AD1CSSLbits.CSS8 = 0;
        } else if (an_idx == ADC_CURRENT_A) {
            AD1CSSLbits.CSS0 = 0;
            AD1CSSLbits.CSS1 = 1;
            AD1CSSLbits.CSS6 = 0;
            AD1CSSLbits.CSS8 = 0;
        } else if (an_idx == ADC_TEMP_AMB) {
            AD1CSSLbits.CSS0 = 0;
            AD1CSSLbits.CSS1 = 0;
            AD1CSSLbits.CSS6 = 1;
            AD1CSSLbits.CSS8 = 0;
        } else if (an_idx == ADC_CURRENT_B) {
            AD1CSSLbits.CSS0 = 0;
            AD1CSSLbits.CSS1 = 0;
            AD1CSSLbits.CSS6 = 0;
            AD1CSSLbits.CSS8 = 1;
        }

	AD1CON1bits.ADON	= 0;							// turn off the adc
	AD1CHS0bits.CH0SA	= an[an_idx];					// select the analog input      
	AD1CON1bits.SSRC	= 0b011;						// set to trigger the sample from the pwm hardware
	AD1CON1bits.ADON	= 1;	// turn on the adc module

        ToggleHeartbeatLED();
}

#endif
