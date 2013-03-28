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


#include "p33fxxxx.h"
#include "setup.h"


//---------------------------------------------------------------
//                  CONFIG
//---------------------------------------------------------------
//New config set
//Refer to p33FJ32MC204.h for the details
_FWDT(FWDTEN_OFF & WINDIS_OFF & WDTPRE_PR128 & WDTPOST_PS32768);					/* Turn off the Watch-Dog Timer.  */
_FPOR(FPWRT_PWR1 & ALTI2C_ON & HPOL_ON & LPOL_ON & PWMPIN_ON);			// Turn off the power-up timers, ALTI2C = 0: I2C mapped to ASDA1/ASCL1
_FOSCSEL(FNOSC_PRIPLL & IESO_ON);	// Auto switch to the EC+PLL clock
//Clock switching+monitor disabled, OSC2 is clock O/P, External clock, allow lock/unlock of peripheral-pin-select
_FOSC(FCKSM_CSDCMD & OSCIOFNC_OFF  & POSCMD_EC & IOL1WAY_OFF); 
_FGS(GCP_OFF & GSS_OFF & GWRP_OFF);            							// Disable Code Protection
_FICD(ICS_PGD2 & JTAGEN_OFF);

//---------------------------------------------------------------------------------
//                     MAIN
//---------------------------------------------------------------------------------

extern ready_for_sending_flag; 	//This variable comes from m3serial.c (for FB_DEV board), and allows the data sending to be done in the main, but only after a reception of data from the slave
									//It helps symchronizing the communication, while freeing the UART interruption while sending data (since the sending process is done in the main

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
#ifdef USE_PWM
	setup_pwm();
#endif
#ifdef USE_CONTROL
	setup_control();
#endif

#ifdef USE_ADC_SPI
	setup_adc_spi();
#endif

#ifdef USE_ENCODER_MA3
	setup_ma3();
#endif
#ifdef USE_ENCODER_QEI
	setup_qei();
#endif
#ifdef USE_ADC
	setup_adc();
#endif
#ifdef USE_TIMER3
	setup_timer3();
#endif
#ifdef USE_BLDC
	setup_bldc();
#endif
#ifdef USE_ENCODER_VERTX
	setup_vertx();
#endif
#ifdef USE_DAC
	setup_dac();
#endif
#ifdef USE_TACTILE_PPS
	setup_pps();
#endif
#ifdef USE_LED_RGB
 setup_led_rgb();
#endif
#ifdef USE_LED_MDRV
 setup_led_mdrv();
#endif
#ifdef USE_BRAKE
 setup_brake();
#endif
#ifdef USE_AS5510
 setup_as5510();
#endif


#ifdef USE_UART
	setup_uart();
#ifdef M3_DEV
	sprintf(uart_str,"Starting M3EC %s %s\n\n\r",__DATE__, __TIME__);
	PUTS(uart_str);
#endif
#endif



//#ifdef USE_ETHERCAT
//	while (!eeprom_loaded())		//Wait until ESC is ready
//		//ToggleHeartbeatLED();				//Success
//		SetHeartbeatLED;
//	setup_ethercat();
//	ClrHeartbeatLED;
//#endif

#ifdef USE_ETHERCAT
//#ifdef M3_FB_DEV_0_0
//	setup_ethercat();
//#else
	while (!eeprom_loaded())		//Wait until ESC is ready
		//ToggleHeartbeatLED();				//Success
		SetHeartbeatLED;
	setup_ethercat();
	ClrHeartbeatLED;
//#endif
#endif


#if defined M3_PWR_0_2 || defined M3_PWR_0_3 || defined M3_PWR_0_4 || defined M3_PWR_0_5
	ClrEnableMotor;
#endif

setup_interrupt_priorities();

dummy = U1RXREG;

	while(1){

#ifdef M3_FB_DEV_0_0
if(ready_for_sending_flag==1)
	{
	serial_eth_refresh();
	serial_eth_send(); //Sending data to the ARMH slave
	//ms_delay(1);
	ready_for_sending_flag=0;
	}
#endif	


#if defined USE_LED_MDRV
	//step_led_mdrv();
	//ToggleHeartbeatLED();
	if (i++%20==0)
	{
		//ToggleHeartbeatLED();
		step_ethercat();
	}
#else
		if (i++%20001==0)
		{
			ToggleHeartbeatLED();
			
			//Temporary code
			//Testing current sensing on PWR_0_5 via UART
			//#ifdef M3_PWR_0_5
			//temp_current_val=get_adc_spi(0);
			//sprintf(uart_str,"ADC: %i n\r",temp_current_val);
			//PUTS(uart_str);
			//TXREG='x';
			//puts1(uart_str);
			//puts1("hello");
			//PORTBBBBBbits.RB10^=1;
			//#endif
		}
#if defined USE_ETHERCAT //&& ! defined M3_MAX2_BDC_ARMH
		step_ethercat();
#endif
#endif
		

#if defined USE_TACTILE_PPS && defined USE_TIMER3
#ifdef M3_GMB_G1R1
	if (pps_trigger)
	{
		pps_trigger=0;
		step_pps(0);
		step_pps(1);
	}
#endif
#endif
//#ifdef USE_AS5510
//	step_as5510();
//	ms_delay(1);
//#endif


	} 
}
