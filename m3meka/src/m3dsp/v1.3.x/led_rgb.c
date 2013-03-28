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
#ifdef USE_LED_RGB
#include "led_rgb.h"
#include "setup.h"

#if defined M3_LEDX2_S1R1 || defined M3_LEDX2XN_S2R1
int led_rgb_vals[LED_RGB_NUM_BRANCH][LED_RGB_BOARDS_PER_BRANCH][3]={0}; //RGB
unsigned long led_cmd;
void setup_led_rgb()
{
#if defined M3_LEDX2_S1R1 || defined M3_LEDX2XN_S2R1
   	LED_LAT_CLR;
	LED_CLK_CLR;
   	LED_EN_A=0;
   	LED_EN_B=0;
	//From Datasheet, good default command
	led_cmd= ((long)2 <<29) | ((long)100 <<20 ) | ((long)120<<10)|((long)100);
	//led_cmd=3354524799; //Set currents to 100%
#endif
}

//Write two branches in parallel
void write_led_array()
{
	int k,i;
	unsigned long val_a,val_b;
	for (k=0;k<LED_RGB_BOARDS_PER_BRANCH;k++)
	{
		val_a=0;
		val_b=0;
		val_a=((long)led_rgb_vals[0][k][2] <<20 ) | 
				((long)led_rgb_vals[0][k][0]<<10)|
				((long)led_rgb_vals[0][k][1]);


		val_b=((long)led_rgb_vals[1][k][2] <<20 ) | 
				((long)led_rgb_vals[1][k][0]<<10)|
				((long)led_rgb_vals[1][k][1]);

	 	for (i=0;i<32;i++)
			LED_AB_BIT_OUT((int)((val_a>>(31-i))&1),(int)((val_b>>(31-i))&1));
	}
	us_delay(15);
	LED_LAT_SET;
	us_delay(15);
	LED_LAT_CLR;
	for (k=0;k<LED_RGB_BOARDS_PER_BRANCH;k++)
		for (i=0;i<32;i++)
			LED_AB_BIT_OUT((int)((led_cmd>>(31-i))&1),(int)((led_cmd>>(31-i))&1));
	DELAY_100NS;
	LED_LAT_SET;
	DELAY_100NS;
	LED_LAT_CLR;
}

//See: http://docs.macetech.com/doku.php/megabrite
void step_led_rgb()
{
#ifdef M3_LEDX2XN_S2R1
	int i,id;
	for (i=0;i<2;i++)
	{
	  id=MAX(0,MIN(LED_RGB_BOARDS_PER_BRANCH-1,ec_cmd.branch[i].id));
	  led_rgb_vals[i][id][0]=MIN(LED_RGB_MAX_VAL,MAX(0,ec_cmd.branch[i].r));
	  led_rgb_vals[i][id][1]=MIN(LED_RGB_MAX_VAL,MAX(0,ec_cmd.branch[i].b));//Not sure why but need to switch B and G here...
	  led_rgb_vals[i][id][2]=MIN(LED_RGB_MAX_VAL,MAX(0,ec_cmd.branch[i].g));
	}
	LED_EN_A = ec_cmd.enable[0]; 
	LED_EN_B = ec_cmd.enable[1]; 
#endif
#ifdef M3_LEDX2_S1R1
	led_rgb_vals[0][0][0]=MIN(LED_RGB_MAX_VAL,MAX(0,ec_cmd.branch_a.board_a.r));
	led_rgb_vals[0][0][1]=MIN(LED_RGB_MAX_VAL,MAX(0,ec_cmd.branch_a.board_a.g));
	led_rgb_vals[0][0][2]=MIN(LED_RGB_MAX_VAL,MAX(0,ec_cmd.branch_a.board_a.b));
	led_rgb_vals[0][1][0]=MIN(LED_RGB_MAX_VAL,MAX(0,ec_cmd.branch_a.board_b.r));
	led_rgb_vals[0][1][1]=MIN(LED_RGB_MAX_VAL,MAX(0,ec_cmd.branch_a.board_b.g));
	led_rgb_vals[0][1][2]=MIN(LED_RGB_MAX_VAL,MAX(0,ec_cmd.branch_a.board_b.b));
	led_rgb_vals[1][0][0]=MIN(LED_RGB_MAX_VAL,MAX(0,ec_cmd.branch_b.board_a.r));
	led_rgb_vals[1][0][1]=MIN(LED_RGB_MAX_VAL,MAX(0,ec_cmd.branch_b.board_a.g));
	led_rgb_vals[1][0][2]=MIN(LED_RGB_MAX_VAL,MAX(0,ec_cmd.branch_b.board_a.b));
	led_rgb_vals[1][1][0]=MIN(LED_RGB_MAX_VAL,MAX(0,ec_cmd.branch_b.board_b.r));
	led_rgb_vals[1][1][1]=MIN(LED_RGB_MAX_VAL,MAX(0,ec_cmd.branch_b.board_b.g));
	led_rgb_vals[1][1][2]=MIN(LED_RGB_MAX_VAL,MAX(0,ec_cmd.branch_b.board_b.b));
	LED_EN_A = ec_cmd.enable_a; 
	LED_EN_B = ec_cmd.enable_b; 
#endif
	write_led_array();
}
#endif //defined M3_LEDX2_S1R1 || M3_LEDX2XN_S2R1

#endif