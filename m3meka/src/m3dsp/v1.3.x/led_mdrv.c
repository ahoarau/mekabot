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
#ifdef USE_LED_MDRV
#include "led_mdrv.h"
#include "setup.h"


// This is the matrix where the Dot Correction values will be stored
// Its size should be set as 16 and 3 (see ledmdrv.h)
// It is initialized at 63 for full potential birghtness on each color of each pixel,
// and should only be modified if some colors are brighter than others.
// Also, dot correction HAS TO BE DONE at least once after power-up for the driver to function.
// LED MCD: Red: 400, Green: 180, Blue: 100
unsigned char dc_matrix[LED_MDRV_NUM_COLUMNS][LED_MDRV_NUM_COLORS]=     
         {	{63, 35, 16}, {63, 35, 16},{63, 35, 16},{63, 35, 16},
			{63, 35, 16}, {63, 35, 16},{63, 35, 16},{63, 35, 16},
			{63, 35, 16}, {63, 35, 16},{63, 35, 16},{63, 35, 16},
			{63, 35, 16}, {63, 35, 16},{63, 35, 16},{63, 35, 16}};

// This is the matrix where the Gray Scale values will be stored
// Its size should be set as 8, 16 and 3 (see ledmdrv.h)
// It is initialized at 0 and should be updated everytime the picture changes 
/// first convert into integer !!!

unsigned char gs_matrix[2][LED_MDRV_NUM_ROWS][LED_MDRV_NUM_COLUMNS][LED_MDRV_NUM_COLORS]=
{{
	{ {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}},
	{ {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}},
	{ {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}},
	{ {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}},
	{ {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}},
	{ {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}},
	{ {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}},
	{ {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}},
},
{
	{ {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}},
	{ {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}},
	{ {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}},
	{ {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}},
	{ {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}},
	{ {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}},
	{ {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}},
	{ {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63},{63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}, {63, 63, 63}},
}
};
int gs_idx;

void dot_correction_mdrv()  //dot correction HAS TO BE DONE at least once after power-up for the driver to function.
{
	
	int i,j,k;	//i=0 -> blue LED
				//i=1 -> green LED
				//i=2 -> red LED

				//j=0 -> pixel number 1
				//j=15 -> pixel number 16

	int x=0;	//x will represent each bit of the dot correction

	LED_MDRV_XLAT=0;
	LED_MDRV_BLANK=1; //Blank should only be low during the specific lighting timing in the gray scale function
	LED_MDRV_SCLK=0;
	DELAY_25NS; //TH2 = 10ns min
	LED_MDRV_DCSEL=1;
	
	// The function below is shifting information from pixel 0 to 16, which will probably result in a mirrored image (or dot correction in this case).
	// Once the program works, this can be easily changed by modifying : 
	// x=0b00000001&(dc_matrix[j][i]>>(5-k)); by
	// x=0b00000001&(dc_matrix[(LED_MDRV_NUM_COLUMNS-1)-j][i]>>(5-k));


	for (i=0;i<LED_MDRV_NUM_COLORS;i++) 
	{
		for(j=0;j<LED_MDRV_NUM_COLUMNS;j++)
		{
			for(k=0;k<LED_MDRV_DC_NBITS;k++)
			{
				x=1&(dc_matrix[j][i]>>(5-k)); //the MSB of the 6 bit dot correction data has to be clocked first.
													   //This line should give x the value of each dot correction bit of the i,j pixel, from MSB(k=0) to LSB(k=5)
				LED_MDRV_BIT_OUT(x);
			}
		}
	}
	
	DELAY_25NS; //TH1 = 10ns min
	LED_MDRV_XLAT=1; //Latch the entire row of Dot Correction from the Dot Correction Shift register, to the Doit correction Data Latch
	DELAY_25NS; //TWH1 = 15ns min
	LED_MDRV_XLAT=0;

	DELAY_25NS;// TSU2 = 100ns min (starting when LED_MDRV_XLAT=1;)
	DELAY_25NS;// 
	DELAY_25NS;
	DELAY_25NS;
}




void overcurrent_safety_mdrv()	//Makes sure that each pixel (R+G+B) has a brightness below the 255 max. Otherwise, the pixel is set to full blue
{								//********************* This Function should be used every time the GS Matrix gets updated !! *************//////
	int row,j;

	for (row=0;row<LED_MDRV_NUM_ROWS;row++) 
	{
		for(j=0;j<LED_MDRV_NUM_COLUMNS;j++)
		{
			if((gs_matrix[gs_idx][row][j][0]+gs_matrix[gs_idx][row][j][1]+gs_matrix[gs_idx][row][j][2])>255)
			{
				gs_matrix[gs_idx][row][j][0]=250; //Full blue LED
				gs_matrix[gs_idx][row][j][1]=00;
				gs_matrix[gs_idx][row][j][2]=00;
			}
		}
	}
}



void setup_led_mdrv()
{
int i,j;
	gs_idx=0;
LED_MDRV_BLANK=1; // Prevents outputs from turning on before the proper GS and DC are written.
LED_MDRV_EN_DEC=1; // None of the rows are activated
LED_MDRV_A_DEC=0;
LED_MDRV_B_DEC=0;
LED_MDRV_C_DEC=0;

for (i=0;i<LED_MDRV_NUM_ROWS;i++)
{
	for(j=0;j<LED_MDRV_NUM_COLUMNS;j++)
	{
		gs_matrix[gs_idx][i][j][0]=0; 
		gs_matrix[gs_idx][i][j][1]=0;
		gs_matrix[gs_idx][i][j][2]=0;
	}
}
dot_correction_mdrv(); //dot correction HAS TO BE DONE at least once after power-up for the driver to function.
}

void select_row_mdrv(int row) //This function pre-selects the row desired. LED_MDRV_EN_DEC=0; needs to be done to enable to BCD decoder output.
{
	if(row==0)
	{
		LED_MDRV_A_DEC=0;
		LED_MDRV_B_DEC=0;
		LED_MDRV_C_DEC=0;
	}
	if(row==1)
	{
		LED_MDRV_A_DEC=1;
		LED_MDRV_B_DEC=0;
		LED_MDRV_C_DEC=0;
	}
	if(row==2)
	{
		LED_MDRV_A_DEC=0;
		LED_MDRV_B_DEC=1;
		LED_MDRV_C_DEC=0;
	}
	if(row==3)
	{
		LED_MDRV_A_DEC=1;
		LED_MDRV_B_DEC=1;
		LED_MDRV_C_DEC=0;
	}
	if(row==4)
	{
		LED_MDRV_A_DEC=0;
		LED_MDRV_B_DEC=0;
		LED_MDRV_C_DEC=1;
	}
	if(row==5)
	{
		LED_MDRV_A_DEC=1;
		LED_MDRV_B_DEC=0;
		LED_MDRV_C_DEC=1;
	}
	if(row==6)
	{
		LED_MDRV_A_DEC=0;
		LED_MDRV_B_DEC=1;
		LED_MDRV_C_DEC=1;
	}
	if(row==7)
	{
		LED_MDRV_A_DEC=1;
		LED_MDRV_B_DEC=1;
		LED_MDRV_C_DEC=1;
	}
}


//NOTE: flag each row when update, then redo grayscale for that row only. otherwise pwm values.
int load_led_mdrv()
{
	int i, row, col,r,g,b,gsn,res;
	gsn=(gs_idx+1)%2; //next buffer for filling
	res=0;
#ifdef USE_ETHERCAT
	for (i = 0; i < LED_MTX_BUF_SIZE; i++)
	{
		row=ec_cmd.array[i].idx>>LED_MDRV_NUM_COLUMNS_SHIFT;
		col=ec_cmd.array[i].idx-row*LED_MDRV_NUM_COLUMNS;
		if (row<LED_MDRV_NUM_ROWS && col<LED_MDRV_NUM_COLUMNS)
		{
			r=ec_cmd.array[i].r;
			g=ec_cmd.array[i].g;
			b=ec_cmd.array[i].b;
			if (r+g+b>255)//detect and signal over-current
			{
				r=0;
				g=0;
				b=255;
			}
			gs_matrix[gsn][row][col][0] = b;
			gs_matrix[gsn][row][col][1] = g;
			gs_matrix[gsn][row][col][2] = r;
		}
		if (row==LED_MDRV_NUM_ROWS-1 && col==LED_MDRV_NUM_COLUMNS-1) //finished filling buf
		{
			gs_idx=gsn;
			res=1;
		}
	}
	ec_stat.debug=ec_cmd.enable;
	return res;
#endif
}

void step_led_mdrv()
{
	//ToggleHeartbeatLED();
	
	int row,col;

	int i,j,k;	//i=0 -> blue LED
				//i=1 -> green LED
				//i=2 -> red LED

				//j=0 -> pixel number 1
				//j=15 -> pixel number 16

	int x=0;	//x will represent each bit of the dot correction
	int buffer=0; // buffer will hold the gray scale value converted from 8 bits to 12bits (which is a <<4, meaning multiplied by 16)

	int pwm; // this variable will be incremented from 0 to 4095 to generate the PWM signal on the GSCLK pin.

	LED_MDRV_XLAT=0;
	LED_MDRV_BLANK=1; //Blank should only be low during the specific lighting timing in the gray scale function
	LED_MDRV_SCLK=0;
	DELAY_25NS; //TH2 = 10ns min
	LED_MDRV_DCSEL=0;
	
	for (row=1;row<LED_MDRV_NUM_ROWS-1;row++)//
	{
		for (i=0;i<LED_MDRV_NUM_COLORS;i++) 
		{
			for(j=0;j<LED_MDRV_NUM_COLUMNS;j++)
			{
				buffer=gs_matrix[gs_idx][row][j][i];				// Store the "unsigned char" gray scale value into an integer variable.
				buffer=buffer<<4;							// multiply the GS value by 16 to get a 255*16=4080 range. The Gray Scale of the driver needs to be coded with 12bits.
				for(k=0;k<LED_MDRV_GS_NBITS;k++)
				{
					x=1&(buffer>>(11-k));						//the MSB of the 6 bit dot correction data has to be clocked first.
																//This line should give x the value of each dot correction bit of the i,j pixel, from MSB(k=0) to LSB(k=5)
					LED_MDRV_BIT_OUT(x);
				}
			}
		}
		
		DELAY_25NS; //TH1 = 10ns min
		LED_MDRV_XLAT=1; //Latch the entire row of Dot Correction from the Dot Correction Shift register, to the Dot correction Data Latch
		DELAY_25NS; //TWH1 = 15ns min
		LED_MDRV_XLAT=0;

		select_row_mdrv(row);	// Pre-select the correct row (the row will only be selected once LED_MDRV_EN_DEC=0;

#ifdef USE_ETHERCAT
		if (ec_cmd.enable)
#endif
		LED_MDRV_EN_DEC=0;	// Enable the selected row to source current.

		DELAY_25NS;// TSU2 = 100ns min (starting when LED_MDRV_XLAT=1;)
		DELAY_25NS;// 
		DELAY_25NS;

		// This last block is repeated 3 times to light up the LEDs for about 1.6ms for each row.
		// Since, the transfer of Gray Scale value takes about (150ns*192*3=86us),
		// We can estimate the whole process of a single row around 1.7ms.
		// Going through the 8 rows should then take around 13.6ms.
		// So setting the timer3 a little above 13.6ms, say 15ms, should give enough time for each step_led_mdrv() to finish, 
		// and we would have a refresh rate of 1/13.6ms = 73.5Hz.
		/**************************************************/

		LED_MDRV_BLANK=0; //allows the constant current sink to function.
		DELAY_25NS; //TSU1 = 15ns min
		for (pwm=0;pwm<4096;pwm++) // This takes about 0.4ms
		{
			LED_MDRV_BB_PWM; //This takes about 100ns
		}
		LED_MDRV_BLANK=1;
		DELAY_25NS; //TWH1 = 15ns min
/*
		LED_MDRV_BLANK=0; //allows the constant current sink to function.
		DELAY_25NS; //TSU1 = 15ns min
		for (pwm=0;pwm<4096;pwm++) // This takes about 0.4ms
		{
			LED_MDRV_BB_PWM; //This takes about 100ns
		}
		LED_MDRV_BLANK=1;
		DELAY_25NS; //TWH1 = 15ns min


		LED_MDRV_BLANK=0; //allows the constant current sink to function.
		DELAY_25NS; //TSU1 = 15ns min
		for (pwm=0;pwm<4096;pwm++) // This takes about 0.4ms
		{
			LED_MDRV_BB_PWM; //This takes about 100ns
		}
		LED_MDRV_BLANK=1;
		DELAY_25NS; //TWH1 = 15ns min


		/**************************************************/

		LED_MDRV_EN_DEC=1; // Disable the selected row until the next row is ready to display.
	}
	//ClrHeartbeatLED;//ToggleHeartbeatLED();
}

/*void mouth_lines(int co)
{
	int intensity;
	int mat;

	int row;

	int i,j,k;	//i=0 -> blue LED
				//i=1 -> green LED
				//i=2 -> red LED

				//j=0 -> pixel number 1
				//j=15 -> pixel number 16

	for (row=0;row<LED_MDRV_NUM_ROWS;row++) 
	{
		for (i=0;i<LED_MDRV_NUM_COLORS;i++) 
		{
			for(j=0;j<LED_MDRV_NUM_COLUMNS;j++)
			{
				for(k=0;k<LED_MDRV_GS_NBITS;k++)
				{
					gs_matrix[row][j][i]=0;					
				}
			}
		}

	}
	
	for(intensity=0;intensity<220;intensity=intensity+14)
	{
		for (row=2;row<5;row++) 
		{
			for(j=0;j<LED_MDRV_NUM_COLUMNS;j++)
			{
				for(k=0;k<LED_MDRV_GS_NBITS;k++)
				{
					gs_matrix[row][j][co]=intensity;					
				}
			}	
		}
	step_led_mdrv();
	}
	//for (mat=0;mat<10;mat++) step_led_mdrv();
	
	for(intensity=150;intensity>0;intensity=intensity-14)
	{
		for (row=2;row<5;row++) 
		{
			for(j=0;j<LED_MDRV_NUM_COLUMNS;j++)
			{
				for(k=0;k<LED_MDRV_GS_NBITS;k++)
				{
					gs_matrix[row][j][co]=intensity;					
				}
			}	
		}
	step_led_mdrv();
	}
	for (row=0;row<LED_MDRV_NUM_ROWS;row++) 
	{
		for (i=0;i<LED_MDRV_NUM_COLORS;i++) 
		{
			for(j=0;j<LED_MDRV_NUM_COLUMNS;j++)
			{
				for(k=0;k<LED_MDRV_GS_NBITS;k++)
				{
					gs_matrix[row][j][i]=0;					
				}
			}
		}

	}

	for (mat=0;mat<7;mat++) step_led_mdrv();
 
}
*/
#endif