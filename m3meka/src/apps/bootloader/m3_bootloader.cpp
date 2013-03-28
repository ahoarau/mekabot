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

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <signal.h>
#include "malloc.h"
#include <rtai_shm.h>
#include <rtai_lxrt.h>
#include <assert.h>
#include <string.h>  /* String function definitions */
#include <stdlib.h>
#include <ctype.h>
#include <time.h>
#include <math.h>

#include <m3rt/bootloader/m3_bootloader.h>
#include <mem.h>
#include <m3rt/base/toolbox.h>

using namespace m3rt;

////////////////////////////////////////////////////////////////////////////////////
typedef struct
{
	char               * pName;
	unsigned short int   Id;
	unsigned short int   ProcessId;
	eFamily              Family;
} sDevice;

sDevice Device[] =
{
	{"dsPIC33FJ32MC204",  0xb52, 0, dsPIC33F},
	{"dsPIC33FJ32MC204",  0xF0B, 3, dsPIC33F},

	{NULL, 0, 0}
};

////////////////////////////////////////////////////////////////////////////////////
void	SysEcShmPrettyPrint(M3EcSystemShm * shm);
void	SlaveEcShmPrettyPrint(M3EcSlaveShm * shm);
void	M3BldPdoStatusPrettyPrint(M3BldPdoV1Status * d);
void	M3BldPdoCmdPrettyPrint(M3BldPdoV1Cmd * d);
void	LockShm(){}
void	ReleaseShm(){}
int		WriteCommBlock (M3EcSlaveShm * slave_shm, char command, char *pBuffer ,  int BytesToWrite);
bool	ReceiveData(M3EcSlaveShm * slave_shm,char command,char * pBuffer, int BytesToReceive);
bool	ReceiveStaleStatusData(M3EcSlaveShm * slave_shm,char * pBuffer, int BytesToReceive);
int		GetStaleStatusNumByte(M3EcSlaveShm * slave_shm);

////////////////////////////////////////////////////////////////////////////////////
bool M3Bootloader::Shutdown()
{
	if (shm!=NULL)
		rtai_free(nam2num(SHMNAM_M3MKMD), &shm);
	shm=NULL;
 	return true;
}

bool M3Bootloader::Startup(int id_slave)
{
	int cntr=0;
	if (shm==NULL)
	{
		for (int i=0;i<100;i++)
			ping_success[i]=0;
		shm = (M3EcSystemShm *)rtai_malloc (nam2num(SHMNAM_M3MKMD),1);
	}
	if (shm == NULL)
	{
		printf("Unable to find EC shared memory\n");
		return false;
	}

	int ns=shm->slaves_responding;
	//if (verbose)
	//	printf("Found %d M3 EtherCAT slaves\n",ns);
	if (ns==0 || id_slave>=ns)
	{
		printf("Invalid slave count %d : %d\n", id_slave,ns);
		return false;
	}
	slave_shm = &(shm->slave[id_slave]);
	M3BldPdoV1Cmd * cmd =(M3BldPdoV1Cmd *) slave_shm->cmd;
	cmd->cmd_idx=0;
	
	//Init Bld
	WriteCommBlock(slave_shm,0,NULL,0);
	
	//Make sure all is ready
	if (!Ping(id_slave))
		return false;
	
	return true;
}



bool M3Bootloader::ReadConfigId(int id_slave)
{
	unsigned short int  DeviceId = 0;
	unsigned short int  ProcessId = 0;
	if (shm==NULL || !ping_success[id_slave])
		return false;
	
	
	if (verbose) printf("\nReading Target Device ID %d\n",id_slave);
	
	slave_shm = &(shm->slave[id_slave]);

	
	if (!WriteCommBlock(slave_shm,COMMAND_READ_ID, NULL, 0)<0)
	{
		printf("Fail on ReadID WriteCommBlock\n");
		return false;
	}
	if (!ReceiveData(slave_shm,COMMAND_READ_ID,Buffer,8))
	{
		printf("Fail on ReadID ReceiveData\n");
		return false;
	}
	DeviceId  = ((Buffer[1] << 8)&0xFF00) | (Buffer[0]&0x00FF);
	ProcessId = (Buffer[5] >> 4) & 0x0F;
	
	int Count         = 0;
	bool bDeviceFound = false;

	while(bDeviceFound != true)
	{
		if(Device[Count].pName == NULL)
		{
			break;
		}

		if((Device[Count].Id == DeviceId) && (Device[Count].ProcessId == ProcessId))
		{
			bDeviceFound = true;
			break;
		}

		Count++;
	}

	printf( "Slave: %d: DeviceId = 0x%04x, ProcessId = %d\n", id_slave, DeviceId, ProcessId);
	if(bDeviceFound == true)
		printf("Slave: %d: Slave: %d: ..   Found %s (ID: 0x%04x)\n", id_slave,Device[Count].pName, DeviceId);
	else
	{
		printf("Slave: %d: ReadConfigId: Device not found...\n",id_slave);
		return false;
	}
	return true;
}


bool M3Bootloader::Ping(int id_slave)
{
	if (shm==NULL)
		return false;
	slave_shm = &(shm->slave[id_slave]);
	if(WriteCommBlock(slave_shm,COMMAND_PING,NULL,0)>=0)
	{
		if (verbose) printf("Slave: %d: Ping reply from bootloader!!!\n",id_slave);
		ping_success[id_slave]=1;
		return true;
	}
	if (verbose) printf("Slave: %d: Ping failed from bootloader...\n",id_slave);
	return false;
}


////////////////////////////////////////////////////////////////////////////////////
bool M3Bootloader::WriteProgramMemory(int id_slave, char * filename)
{
	if (shm==NULL|| !ping_success[id_slave])
		return false;
	slave_shm = &(shm->slave[id_slave]);
	
	FILE * pFile = fopen(filename, "r");
	int  ExtAddr = 0;

	if(pFile == NULL)
	{
		printf("Could not find HEX file\n");
		return false;
	}

	// Initialize Memory 
	mem_cMemRow ** ppMemory = (mem_cMemRow **)malloc(sizeof(mem_cMemRow *) * PM_SIZE + sizeof(mem_cMemRow *) * EE_SIZE + sizeof(mem_cMemRow *) * CM_SIZE);

	for(int Row = 0; Row < PM_SIZE; Row++)
	{
		ppMemory[Row] = new mem_cMemRow(mem_cMemRow::Program, 0x000000, Row, Family);
	}

	for(int Row = 0; Row < EE_SIZE; Row++)
	{
		ppMemory[Row + PM_SIZE] = new mem_cMemRow(mem_cMemRow::EEProm, 0x7FF000, Row, Family);
	}

	for(int Row = 0; Row < CM_SIZE; Row++)
	{
		ppMemory[Row + PM_SIZE + EE_SIZE] = new mem_cMemRow(mem_cMemRow::Configuration, 0xF80000, Row, Family);
	}

	if (verbose) printf("\nSlave: %d Reading HexFile: %s",id_slave,filename);
	int ccnt=0;
	while(fgets(Buffer, sizeof(Buffer), pFile) != NULL)
	{
		int ByteCount;
		int Address;
		int RecordType;

		sscanf(Buffer+1, "%2x%4x%2x", &ByteCount, &Address, &RecordType);
		//printf("Hex file %d: ByteCount %d Address %d RecordType %d\n",ccnt++,ByteCount,Address,RecordType); 
		if(RecordType == 0)
		{
			Address = (Address + ExtAddr) / 2;

			for(int CharCount = 0; CharCount < ByteCount*2; CharCount += 4, Address++)
			{
				bool bInserted = false;

				for(int Row = 0; Row < (PM_SIZE + EE_SIZE + CM_SIZE); Row++)
				{
					if((bInserted = ppMemory[Row]->InsertData(Address, Buffer + 9 + CharCount)) == true)
					{
						break;
					}
				}

				if(bInserted != true)
				{
					printf("Slave %d: Bad Hex file: 0x%xAddress out of range\n", id_slave,Address);
					assert(0);
				}
			}
		}
		else if(RecordType == 1)
		{
		}
		else if(RecordType == 4)
		{
			sscanf(Buffer+9, "%4x", &ExtAddr);
			ExtAddr = ExtAddr << 16;
		}
		else
		{
			assert(!"Unknown hex record type\n");
		}
	}
	
	// Preserve first two locations for bootloader 
	char Data[32];
	ReadProgramMemory(id_slave, "0x000000");
	sprintf(Data, "%02x%02x%02x00%02x%02x%02x00",   Buffer[2] & 0xFF,
							Buffer[1] & 0xFF,
							Buffer[0] & 0xFF,
							Buffer[5] & 0xFF,
							Buffer[4] & 0xFF,
							Buffer[3] & 0xFF);
	
	//printf("0: %d\n",Data);
	ppMemory[0]->InsertData(0x000000, Data);
	//printf("1: %d\n",Data + 4);
	ppMemory[0]->InsertData(0x000001, Data + 4);
	//printf("2: %d\n",Data + 8);
	ppMemory[0]->InsertData(0x000002, Data + 8);
	//printf("3: %d\n",Data + 12);
	ppMemory[0]->InsertData(0x000003, Data + 12);

	for(int Row = 0; Row < (PM_SIZE + EE_SIZE + CM_SIZE); Row++)
	{
		ppMemory[Row]->FormatData();
	}

	if (verbose) printf("\nSlave %d: Programming Device... ",id_slave);
	for(int Row = 0; Row < (PM_SIZE + EE_SIZE + CM_SIZE); Row++)
		ppMemory[Row]->PrintData();
	for(int Row = 0; Row < (PM_SIZE + EE_SIZE + CM_SIZE); Row++)
		ppMemory[Row]->SendData(slave_shm);
	return true;
}

////////////////////////////////////////////////////////////////////////////////////
bool M3Bootloader::ReadProgramMemory(int id_slave, char * address)
{
	int          Count;
	unsigned int ReadAddress;
	int          RowSize;

	if (shm==NULL || !ping_success[id_slave])
		return false;
	slave_shm = &(shm->slave[id_slave]);
	
	assert(address[0] == '0' && address[1] =='x');
	assert(isxdigit(address[2]));
	assert(isxdigit(address[3]));
	assert(isxdigit(address[4]));
	assert(isxdigit(address[5]));
	assert(isxdigit(address[6]));
	assert(isxdigit(address[7]));

	RowSize = PM33F_ROW_SIZE;

	sscanf(address, "%x", &ReadAddress);
	//printf("Reading address %d : %s",ReadAddress, address);
	ReadAddress = ReadAddress - ReadAddress % (RowSize * 2);
	//printf("  : Actual read starting at %d\n",ReadAddress);
	Buffer[0] = ReadAddress & 0xFF;
	Buffer[1] = (ReadAddress >> 8) & 0xFF;
	Buffer[2] = (ReadAddress >> 16) & 0xFF;

	for (Count = 0; Count < 3; Count++) {

		if (verbose) printf("Address [%d] = %d\n", Count, Buffer[Count]);
		}

	if (WriteCommBlock(slave_shm,COMMAND_READ_PM, Buffer, 3)<0)
	{
		printf("ReadPM failed on WriteCommBlock\n");
		return false;
	}
	
	if (!ReceiveData(slave_shm,COMMAND_READ_PM_CON, Buffer, RowSize * 3))
	{
		printf("ReadPM failed on ReceiveData\n");
		return false;
	}

	for(Count = 0; Count < RowSize * 3;)
	{
		if (verbose) printf("0x%06x: ", ReadAddress);

		if (verbose) printf("%02x",Buffer[Count++] & 0xFF);
		if (verbose) printf("%02x",Buffer[Count++] & 0xFF);
		if (verbose) printf("%02x ",Buffer[Count++] & 0xFF);

		if (verbose) printf("%02x",Buffer[Count++] & 0xFF);
		if (verbose) printf("%02x",Buffer[Count++] & 0xFF);
		if (verbose) printf("%02x ",Buffer[Count++] & 0xFF);

		if (verbose) printf("%02x",Buffer[Count++] & 0xFF);
		if (verbose) printf("%02x",Buffer[Count++] & 0xFF);
		if (verbose) printf("%02x ",Buffer[Count++] & 0xFF);

		if (verbose) printf("%02x",Buffer[Count++] & 0xFF);
		if (verbose) printf("%02x",Buffer[Count++] & 0xFF);
		if (verbose) printf("%02x\n",Buffer[Count++] & 0xFF);

		ReadAddress = ReadAddress + 8;
	}
	if (verbose) printf("Slave %d: Memory read done!\n",id_slave);
	return true;
}
////////////////////////////////////////////////////////////////////////////////////
//Read data from EC shared-memory
//Assumes that up to BLD_BUF_SIZE already available from previous command
//Loops until BytesToReceive bytes are recieved, issuing read-commands to the dsPic as needed
bool ReceiveData(M3EcSlaveShm * slave_shm,char command,char * pBuffer, int BytesToReceive)
{
	int n_left=BytesToReceive;
	int nr=0;
	int nc=0;
	//Should be one stale buffer ready
	ReceiveStaleStatusData(slave_shm,pBuffer,GetStaleStatusNumByte(slave_shm));
	n_left-=GetStaleStatusNumByte(slave_shm);
	nr+=GetStaleStatusNumByte(slave_shm);
	while(n_left)
	{
		if (WriteCommBlock(slave_shm,command,NULL,0)<0)
		{
			printf("WriteCommBlock failed in ReceiveData\n");
			return false;
		}
		ReceiveStaleStatusData(slave_shm,pBuffer+nr,GetStaleStatusNumByte(slave_shm));
		n_left-=GetStaleStatusNumByte(slave_shm);
		nr+=GetStaleStatusNumByte(slave_shm);
		//printf("ReceiveData: total %d n_left: %d last rcv: %d\n",BytesToReceive,n_left,GetStaleStatusNumByte(slave_shm));
		if (n_left<0)
		{
			printf("Wrong number of reply bytes: Over by %d and Got %d\n",n_left,GetStaleStatusNumByte(slave_shm));
			return false;
		}
	}
	return true;
}
////////////////////////////////////////////////////////////////////////////////////
bool ReceiveStaleStatusData(M3EcSlaveShm * slave_shm,char * pBuffer, int BytesToReceive)
{
	M3BldPdoV1Status * status= (M3BldPdoV1Status *) slave_shm->status;
	if (BytesToReceive>BLD_BUF_SIZE)
	{
		printf("ReceiveStaleStatusData buffer size too large\n");
		return false;
	}
	if (status->flags==COMLINK_ERR)
	{
		printf("COMLINK_ERR: %d \n",status->flags);
		return false;
	}
	memcpy(pBuffer,status->data,BytesToReceive);
	return true;
}
////////////////////////////////////////////////////////////////////////////////////
int GetStaleStatusNumByte(M3EcSlaveShm * slave_shm)
{
	M3BldPdoV1Status * status= (M3BldPdoV1Status *) slave_shm->status;
	return status->nbytes;
}
////////////////////////////////////////////////////////////////////////////////////
//Return num bytes written as ack by dsp
int WriteCommBlock(M3EcSlaveShm * slave_shm,char command, char *pBuffer , int BytesToWrite)
{
	int buf_idx=0; 
	int n_left=BytesToWrite;
	int n_reply=0;
	M3BldPdoV1Cmd * cmd= (M3BldPdoV1Cmd *) slave_shm->cmd;
	M3BldPdoV1Status * status= (M3BldPdoV1Status *) slave_shm->status;

	//printf("Comm block Writing %d bytes\n",n_left);
	while (1)
	{
		int nb=(n_left>BLD_BUF_SIZE)?BLD_BUF_SIZE:n_left;
		LockShm();
		cmd->nbytes=nb;
		cmd->cmd=command;
		cmd->cmd_idx++;
		//printf("WriteCommBlock send cycle %d, n_left %d\n",cmd->cmd_idx,n_left);
		if (nb) 
			memcpy(cmd->data,pBuffer+buf_idx,nb);
		ReleaseShm();
		n_left-=nb;
		buf_idx+=nb;
		int cnt=0;
		while (status->cmd_idx_ack!=cmd->cmd_idx)
		{
			//if (cnt>2)
			//	printf("Cnt %d: Waiting on Cmd IDX ack %d. Have %d\n",cnt,cmd->cmd_idx,status->cmd_idx_ack);
			if (++cnt%100==0)
			{
				printf("WriteCommBlock failed waiting for ACK\n");
				M3BldPdoStatusPrettyPrint(status);
				return -1;
			}
			usleep(10000);
		}
		n_reply+=status->nbytes;
		if (!n_left) //allow single send of 0 bytes
			break;
	}
	//printf("WriteCommBlock successful send of %d bytes\n",BytesToWrite);
	return n_reply;
}

////////////////////////////////////////////////////////////////////////////////////
void SysEcShmPrettyPrint(M3EcSystemShm * shm)
{
	int i;
	printf("----- SysEcShm -----\n");
	printf("slaves_responding : %d\n",shm->slaves_responding );
	printf("slaves_active : %d\n",shm->slaves_active );
	printf("slaves_dropped : %d\n",shm->slaves_dropped );
	printf("link_up : %d\n",shm->link_up );
	printf("watchdog : %d\n",shm->watchdog );
	for (i=0;i<shm->slaves_responding;i++)
		if (shm->slave[i].active)
			SlaveEcShmPrettyPrint(&(shm->slave[i]));
}
	
////////////////////////////////////////////////////////////////////////////////////
void SlaveEcShmPrettyPrint(M3EcSlaveShm * slave_shm)
{
	printf("\n\n----------------- Slave: %d -----------------\n",slave_shm->network_id);
	printf("active : %d\n",slave_shm->active);
	printf("network_id : %d\n",slave_shm->network_id);
	printf("serial_number : %d\n",slave_shm->serial_number);
	printf("product_code : %d\n",slave_shm->product_code);
	printf("online : %d\n",slave_shm->online);
	printf("operational : %d\n",slave_shm->operational);
	printf("al_state : %d\n",slave_shm->al_state);
	M3BldPdoStatusPrettyPrint((M3BldPdoV1Status *) slave_shm->status);
	M3BldPdoCmdPrettyPrint((M3BldPdoV1Cmd *) slave_shm->cmd);
}

////////////////////////////////////////////////////////////////////////////////////
void M3BldPdoStatusPrettyPrint(M3BldPdoV1Status * d)
{
	printf("----- Status -----\n");
	printf("nbytes: %d\n",(int) d->nbytes);
	printf("flags: %d\n",(int) d->flags);
	printf("cmd_idx_ack: %d\n",(int) d->cmd_idx_ack);
}
////////////////////////////////////////////////////////////////////////////////////
void M3BldPdoCmdPrettyPrint(M3BldPdoV1Cmd * d)
{
	printf("----- Command -----\n");
	printf("nbytes: %d\n",(int) d->nbytes);
	printf("cmd_idx: %d\n",(int) d->cmd_idx);
	printf("cmd: %d\n",(int) d->cmd);
	printf("config: %d\n",(int) d->config);
}
////////////////////////////////////////////////////////////////////////////////////

