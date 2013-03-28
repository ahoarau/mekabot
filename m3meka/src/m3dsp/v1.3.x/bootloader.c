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


#include "p33Fxxxx.h"
#include "bootloader.h"
#include "setup.h"
#include "ethercat.h"
#include "dio.h"

///////////////////////////////////////////////////////////////////////////

#define COMMAND_NACK			0x00
#define COMMAND_ACK				0x01
#define COMMAND_READ_PM			0x02
#define COMMAND_READ_PM_CON		0x03
#define COMMAND_WRITE_PM		0x04
#define COMMAND_WRITE_PM_CON	0x05
#define COMMAND_WRITE_CM		0x06
#define COMMAND_RESET			0x07
#define COMMAND_READ_ID			0x08
#define COMMAND_PING			0x09
#define COMLINK_ERR 			0x0A

#define PM_ROW_SIZE 64 * 8 
#define CM_ROW_SIZE 8
#define CONFIG_WORD_SIZE 1
#define PM_ROW_ERASE 		0x4042
#define PM_ROW_WRITE 		0x4001
#define CONFIG_WORD_WRITE	0X4000

///////////////////////////////////////////////////////////////////////////

typedef short          Word16;
typedef unsigned short UWord16;
typedef long           Word32;
typedef unsigned long  UWord32;

typedef union tuReg32
{
	UWord32 Val32;
	struct
	{
		UWord16 LW;
		UWord16 HW;
	} Word;
	char Val[4];
} uReg32;

struct _global_cursor {
	int globalPtr;
	int globalCtr;
	unsigned char flag;
	uReg32 SourceAddrPM;
};

extern UWord32 ReadLatch(UWord16, UWord16);
void WriteBuffer(char *, int, int);
void ReadPM(char *, ec_stat_t *, uReg32 *);
void WritePM(char *, ec_stat_t *,uReg32 *);
void memncpy(char *, char *, int , int);
void memacpy(char *, char *, int, int);
int ProcessReq(ec_cmd_t *);

///////////////////////////////////////////////////////////////////////////
int cmd_idx_last=-1;
struct _global_cursor global_cursor;
static uReg32 TempAddr;
static uReg32 TempData;

///////////////////////////////////////////////////////////////////////////
void step_bootloader()
{
	step_ethercat();
}

void setup_bootloader()
{
	setup_ethercat();
	//Only run bootloader if Station ID is not 0
	if (get_configured_station_alias()==0)
	{
		clear_configuration();
		ResetDevice();
	}
}

//return 1 if to exit bootloader
int bootloader_process_cmd(ec_cmd_t * ec_cmd)
{
	char Command = ec_cmd->cmd;
	ec_stat.flags=0;

	if(Command == 0x0) { //Reset
		//memset(&ec_stat, 0x0, sizeof( ec_stat_t));
		ec_stat.flags =COMMAND_ACK;
		ec_stat.cmd_idx_ack=ec_cmd->cmd_idx;
		ec_stat.nbytes=ec_cmd->nbytes;
		cmd_idx_last=-1;
		return 0;
	}
	
	if (cmd_idx_last==ec_cmd->cmd_idx)
	{
		ec_stat.flags =COMMAND_ACK;
		return 0;
	}

	switch(Command)
	{
		case COMMAND_PING:
		{
			ToggleHeartbeatLED();
			ec_stat.cmd_idx_ack=ec_cmd->cmd_idx;
			ec_stat.nbytes=ec_cmd->nbytes;
			break;
		}
		case COMMAND_READ_ID:
		{
			uReg32 SourceAddr;
			uReg32 Temp;
			SourceAddr.Val32 = 0xFF0000;
			Temp.Val32 = ReadLatch(SourceAddr.Word.HW, SourceAddr.Word.LW);
			memncpy(ec_stat.data, &(Temp.Val[0]), 0, 4);
			SourceAddr.Val32 = 0xFF0002;
			Temp.Val32 = ReadLatch(SourceAddr.Word.HW, SourceAddr.Word.LW);
			memacpy(ec_stat.data, &(Temp.Val[0]), 4, 4);
			ec_stat.nbytes=8;
			ec_stat.cmd_idx_ack=ec_cmd->cmd_idx;
			break;
		}

		case COMMAND_READ_PM:		
		{
			//Prepare for read but don't begin
			memset(&global_cursor, 0x0, sizeof(global_cursor));
			global_cursor.flag = COMMAND_READ_PM;
			memncpy(global_cursor.SourceAddrPM.Val, ec_cmd->data, 0, 3);
			global_cursor.SourceAddrPM.Val[3]=0;
			ReadPM(ec_stat.data, &ec_stat, &global_cursor.SourceAddrPM);
			ec_stat.cmd_idx_ack=ec_cmd->cmd_idx;
			break;
		}
		case COMMAND_READ_PM_CON: /*Send the continuation of the READ_PM Data */
		{	
			ec_stat.nbytes=0;
			ReadPM(ec_stat.data, &ec_stat, &global_cursor.SourceAddrPM);
			ec_stat.cmd_idx_ack=ec_cmd->cmd_idx;
			break;
		}

		case COMMAND_WRITE_PM:				
		{
			//uReg32 SourceAddr;
			memset(&global_cursor, 0x0, sizeof(global_cursor));
			global_cursor.flag = COMMAND_WRITE_PM;
			memncpy(global_cursor.SourceAddrPM.Val, ec_cmd->data, 0, 3);
			global_cursor.SourceAddrPM.Val[3]=0;
			//SourceAddr.Val[0] = 0;
			//SourceAddr.Val[1] = 0;
			//SourceAddr.Val[2] = 0;
			//SourceAddr.Val[4] = 0;
			Erase(global_cursor.SourceAddrPM.Word.HW,global_cursor.SourceAddrPM.Word.LW,PM_ROW_ERASE);
			ec_stat.cmd_idx_ack=ec_cmd->cmd_idx;
			ec_stat.nbytes=ec_cmd->nbytes;
 			break;
		}

		case COMMAND_WRITE_PM_CON: /*Write the continuation of the WRITE_PM Data */
		{	
			ec_stat.nbytes=0;
			WritePM(ec_cmd->data,&ec_stat, &global_cursor.SourceAddrPM);
			ec_stat.cmd_idx_ack=ec_cmd->cmd_idx;
			break;
		}
			
		case COMMAND_WRITE_CM:	
		{
			ec_stat.cmd_idx_ack=ec_cmd->cmd_idx;
			ec_stat.nbytes=ec_cmd->nbytes;
			/*
			uReg32 SourceAddr;
			int    Size;
			uReg32 Temp;for(Size = 1, SourceAddr.Val32 = 0xF80000; Size < ec_cmd->nbytes; //CM_ROW_SIZE*3; 
											Size +=3, SourceAddr.Val32 += 2)
			{
				if(ec_cmd->data[Size] == 0)
				{
					Temp.Val[0]=ec_cmd->data[Size+1];
					Temp.Val[1]=ec_cmd->data[Size+2];

					WriteLatch( SourceAddr.Word.HW,
									SourceAddr.Word.LW,
									Temp.Word.HW,
									Temp.Word.LW);
					WriteMem(CONFIG_WORD_WRITE);
				}
			}*/
			break;
		}

		case COMMAND_RESET:
		{
			ec_stat.cmd_idx_ack=ec_cmd->cmd_idx;
			ec_stat.nbytes=ec_cmd->nbytes;
			clear_configuration();
			ResetDevice();
			break;
		}
		default:
			ec_stat.flags = COMLINK_ERR;
			ec_stat.cmd_idx_ack=ec_cmd->cmd_idx;
			ec_stat.nbytes=ec_cmd->nbytes;
			break;
	}
	
	cmd_idx_last=ec_cmd->cmd_idx;
	return 0;
	//memset(ec_cmd, 0x0, sizeof( ec_cmd_t));
}


/******************************************************************************/
void ReadPM(char * ptrData,  ec_stat_t *ec_statp, uReg32 *SourceAddrp)
{
	int Size;
	uReg32 Temp;

	if(global_cursor.flag != COMMAND_READ_PM) {
		
		ec_statp->flags = COMLINK_ERR;
		ec_statp->flags = -COMLINK_ERR;
		return;
	}

	ec_statp->nbytes = 0;
	for(Size = global_cursor.globalPtr; Size < PM_ROW_SIZE; Size++)
	{
		if((ec_statp->nbytes + 3) > BLD_BUF_SIZE) break;
		Temp.Val32 = ReadLatch(SourceAddrp->Word.HW, SourceAddrp->Word.LW);
		ptrData[0] = Temp.Val[2];
		ptrData[1] = Temp.Val[1];
		ptrData[2] = Temp.Val[0];
		ptrData = ptrData + 3;
		global_cursor.globalPtr++;
		ec_statp->nbytes += 3;
		global_cursor.globalCtr += 3;
		SourceAddrp->Val32 = SourceAddrp->Val32 + 2;
	}
}

void clear_configuration()
{
	T2CONbits.T32 = 0; /* to increment every instruction cycle */
	IFS0bits.T3IF = 0; /* Clear the Timer3 Interrupt Flag */
	IEC0bits.T3IE = 0; /* Disable Timer3 Interrup Service Routine */

	PR3 = 0xFFFF;
	PR2 = 0xFFFF;
	T2CONbits.TON = 0;
}

/******************************************************************************/
/*
 * Copy the nth data into target
 */
void memncpy(char *target, char *src, int start, int len)
{
	int ctr = 0;
    for(; ctr < len; ctr++)
    	*(target++) =  *(src + (start + ctr));
}

/*
 * Copy the nth data and append into target
 */
void memacpy(char *target, char *src, int start, int len)
{
	int ctr = 0;
	for(; ctr < len; ctr++) {
		*(target + ctr + start) =  *(src + ctr);
	}
}

void WritePM(char * ptrData, ec_stat_t *ec_statp, uReg32 *SourceAddrPM)//char * ptrData, uReg32 *SourceAddrPM)
{
	uReg32 Temp;
	ec_statp->nbytes=0;

	if(global_cursor.flag != COMMAND_WRITE_PM) {
		ec_stat.data[0] = global_cursor.flag; 
		ec_stat.flags = COMLINK_ERR;
		return;
	}
	for(; global_cursor.globalPtr < PM_ROW_SIZE; global_cursor.globalPtr++)
	{
		if((ec_statp->nbytes + 3) > BLD_BUF_SIZE) break;  //BLD_BUF_SIZE is multiple of 3 so byte-aligned

		Temp.Val[0]=ptrData[ec_statp->nbytes+0];
		Temp.Val[1]=ptrData[ec_statp->nbytes+1];
		Temp.Val[2]=ptrData[ec_statp->nbytes+2];
		Temp.Val[3]=0;
		ec_statp->nbytes+=3;
		global_cursor.globalCtr += 3;
	    WriteLatch(SourceAddrPM->Word.HW, SourceAddrPM->Word.LW,Temp.Word.HW,Temp.Word.LW);

		/* Device ID errata workaround: Save data at any address that has LSB 0x18 */
		if((SourceAddrPM->Val32 & 0x0000001F) == 0x18)
		{
			TempAddr.Val32 = SourceAddrPM->Val32;
			TempData.Val32 = Temp.Val32;
		}

		if((global_cursor.globalPtr !=0) && (((global_cursor.globalPtr + 1) % 64) == 0))
		{
			/* Device ID errata workaround: Reload data at address with LSB of 0x18 */
	      WriteLatch(TempAddr.Word.HW, TempAddr.Word.LW,TempData.Word.HW,TempData.Word.LW);
		  WriteMem(PM_ROW_WRITE);
		}
		SourceAddrPM->Val32 = SourceAddrPM->Val32 + 2;
	}
}



