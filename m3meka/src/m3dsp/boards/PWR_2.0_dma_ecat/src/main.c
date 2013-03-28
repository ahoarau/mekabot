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

#include "warning.h"

//#include "p33FJ64MC204.h"
#include "setup.h"
#include "ethercat_hw.h"
//#include "p33FJ64MC204.h"

//---------------------------------------------------------------
//                  CONFIG
//---------------------------------------------------------------
//New config set

//Refer to p33FJ32MC204.h for the details
_FWDT(FWDTEN_ON & WINDIS_OFF & WDTPRE_PR128 & WDTPOST_PS32768);					/* Turn off the Watch-Dog Timer.  */
_FPOR(FPWRT_PWR1 & ALTI2C_ON & HPOL_ON & LPOL_ON & PWMPIN_ON);			// Turn off the power-up timers, ALTI2C = 0: I2C mapped to ASDA1/ASCL1
_FOSCSEL(FNOSC_PRIPLL & IESO_ON);	// Auto switch to the EC+PLL clock
//Clock switching+monitor disabled, OSC2 is clock O/P, External clock, allow lock/unlock of peripheral-pin-select
_FOSC(FCKSM_CSDCMD & OSCIOFNC_OFF  & POSCMD_EC & IOL1WAY_OFF); 
_FGS(GCP_OFF & GSS_OFF & GWRP_OFF);            							// Disable Code Protection
_FICD(ICS_PGD2 & JTAGEN_OFF);

// Set up configuration bits

//_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2)
//_CONFIG1(JTAGEN_OFF  );
//_CONFIG2( FCKSM_CSDCMD & OSCIOFNC_ON & POSCMOD_HS & FNOSC_PRI )
//_CONFIG2( );
//---------------------------------------------------------------------------------
//                     MAIN
//---------------------------------------------------------------------------------

int main (void)
{
	

	int dummy;
	//int matrix; //only for test of ledmdrv
	int i;
	//int temp_current_val; //temporary
	//int first_ec=1;
	//Setup oscillator/ports/pins first
	setup_oscillator();
	setup_ports();
	setup_peripheral_pin_select();
	setup_interrupt_priorities();

#ifdef USE_DIO
	setup_dio();
#endif

#ifdef USE_CONTROL
	setup_control();
#endif

#ifdef USE_ADC_SPI
	setup_adc_spi();
#endif

#ifdef USE_ADC
	setup_adc();
        initDma2();					// Initialise the DMA controller to buffer ADC data in conversion order
#endif
#ifdef USE_TIMER3
	setup_timer3();
#endif

#ifdef USE_UART
	setup_uart();
#endif


#ifdef USE_ETHERCAT
	while (!eeprom_loaded())		//Wait until ESC is ready
		//ToggleHeartbeatLED();				//Success
		SetHeartbeatLED;

	setup_ethercat();
	ClrHeartbeatLED;
#endif


#if defined PWR_0_2 || defined PWR_0_3 || defined PWR_0_4 || defined PWR_0_5
	ClrEnableMotor;
#endif

setup_interrupt_priorities();

dummy = U1RXREG;

	while(1){

		if (i++%20001==0)
		{
			ToggleHeartbeatLED();
#if defined USE_ETHERCAT
                    if (!IEC0bits.DMA1IE)
                    {
                        DISABLE_AL_EVENT_INT;
                        step_ethercat();
                        ENABLE_AL_EVENT_INT;

                    }
#endif

		}

	} 
}
