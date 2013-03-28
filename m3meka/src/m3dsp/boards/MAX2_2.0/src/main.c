/* 
M3 -- Meka Robotics Real-Time Control System
Copyright (c) 2010 Meka Robotics
Author: edsinger@mekabot.com (Aaron Edsinger)

M3 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

M3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with M3.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "setup.h"
#include "warning.h"

//---------------------------------------------------------------
//                  Timing Information
//---------------------------------------------------------------
/*
=> PWM Frequency: 40kHz
=> The PWM Special Event trigger is used to start an ADC conversion
=> ADC ISR: 1 interrupt each 4 PWM cycles (40kHz/4 = 10kHz)
=> The ADC ISR calls many functions: control loops, Vertex Encoder, etc.
	=> Most of them run at 2kHz (10kHz/5)
*/

//---------------------------------------------------------------
//                  CONFIG
//---------------------------------------------------------------
//New config set, refer to p33FJ32MC204.h for the details
//_FWDT(FWDTEN_ON & WINDIS_OFF & WDTPRE_PR128 & WDTPOST_PS32768);		// Turn off the Watch-Dog Timer.
_FWDT(FWDTEN_ON & WINDIS_OFF & WDTPRE_PR128 & WDTPOST_PS1);		// Turn off the Watch-Dog Timer.
_FPOR(FPWRT_PWR1 & ALTI2C_ON & HPOL_ON & LPOL_ON & PWMPIN_ON);			// Turn off the power-up timers, ALTI2C = 0: I2C mapped to ASDA1/ASCL1
_FOSCSEL(FNOSC_PRIPLL & IESO_ON);	// Auto switch to the EC+PLL clock
//Clock switching+monitor disabled, OSC2 is clock O/P, External clock, allow lock/unlock of peripheral-pin-select
_FOSC(FCKSM_CSDCMD & OSCIOFNC_OFF  & POSCMD_EC & IOL1WAY_OFF); 
_FGS(GCP_OFF & GSS_OFF & GWRP_OFF);            							// Disable Code Protection
_FICD(ICS_PGD2 & JTAGEN_OFF);

//---------------------------------------------------------------------------------
//                     MAIN
//---------------------------------------------------------------------------------

int main (void)
{
	int i=0;
        tmp_debug = 0;

	//Setup oscillator/ports/pins first
	setup_oscillator();
	setup_ports();
	setup_peripheral_pin_select();
	setup_interrupt_priorities();

        //init_temperature_model();

	//Blinking HB LED and Timestamp
	#ifdef USE_DIO
	setup_dio();
	#endif
	
	//ToDo Used?
	#ifdef USE_TIMER1 
	setup_timer1(); //do before setup_pwm
	#endif	//ToDo: there was a missing #endif, maybe that's why disabling Timer1 was breaking the code!
	
	//Motor control PWM
	#ifdef USE_PWM
	//setup_pwm();
	#endif
	
	//Brushless motor
	#ifdef USE_BLDC
	setup_bldc();
	#endif
		
	//Torque PID and Current PID
	#ifdef USE_CONTROL
	setup_control();
	#endif
	
	//Analog to Digital
	#ifdef USE_ADC        
	setup_adc();
	#endif
	
	//Torso external ADC
	#ifdef USE_ADC_SPI
	setup_adc_spi();
	#endif
	
	//Current measurment, control and limitation
	#ifdef USE_CURRENT
	setup_current();
	#endif
	
	//ToDo: Remvove?
	#ifdef USE_TIMER3
	setup_timer3();
	#endif
	
	//VerteX SPI Encoder
	#ifdef USE_ENCODER_VERTX
	setup_vertx();
	#endif
	
	//Torso motor brake
	#ifdef USE_BRAKE
	setup_brake();
	#endif

	//EtherCAT Communication
	#ifdef USE_ETHERCAT
	while (!eeprom_loaded())	//Wait until ESC is ready
		SetHeartbeatLED;
	setup_ethercat();
	ClrHeartbeatLED;
	#endif

	setup_interrupt_priorities();

        if (RCONbits.WDTO)
            SetHeartbeatLED;

        //RCONbits.SWDTEN = 1;//enable watchdog timer

        if (RCON & 0x03 == 0x03)
        {
            RCONbits.BOR = 0;
            RCONbits.POR = 0;
        }
        
        rcon_reg = RCON;


	while(1)
	{
		if (i++%10001==0)
		{
			//ToggleHeartbeatLED();
                        #if defined USE_ETHERCAT
                        step_ethercat();
                        #endif
		}
		
	}
}
