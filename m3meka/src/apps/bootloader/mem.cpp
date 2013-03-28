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

#include "mem.h"
#include "malloc.h"
#include "stdio.h"
#include "string.h"


extern int   WriteCommBlock (M3EcSlaveShm * slave_shm, char command, char *pBuffer ,  int BytesToWrite);
extern void   ReceiveData(M3EcSlaveShm * slave_shm,char command,char * pBuffer, int BytesToReceive);

char	spi_buffer[PM33F_ROW_SIZE*2];
int		spi_buffer_ctr;

/******************************************************************************/
mem_cMemRow::mem_cMemRow(eType Type, unsigned int StartAddr, int RowNumber, eFamily Family)
{
	int Size;



	m_RowNumber = RowNumber;
	m_eFamily    = Family;
	m_eType     = Type;
	m_bEmpty    = true;	

	spi_buffer_ctr = 0;

	if(m_eType == Program)
	{
		if(m_eFamily == dsPIC30F)
		{
			m_RowSize = PM30F_ROW_SIZE;
		}
		else
		{
			m_RowSize = PM33F_ROW_SIZE;
		}
	}
	else
	{
		m_RowSize = EE30F_ROW_SIZE;
	}

	if(m_eType == Program)
	{
		Size = m_RowSize * 3;
		m_Address = StartAddr + RowNumber * m_RowSize * 2;
	}
	if(m_eType == EEProm)
	{
		Size = m_RowSize * 2;
		m_Address = StartAddr + RowNumber * m_RowSize * 2;
	}
	if(m_eType == Configuration)
	{
		Size = 3;
		m_Address = StartAddr + RowNumber * 2;
	}

	m_pBuffer   = (char *)malloc(Size);

	memset(m_Data, 0xFFFF, sizeof(unsigned short)*PM33F_ROW_SIZE*2);	
}
/******************************************************************************/
bool mem_cMemRow::InsertData(unsigned int Address, char * pData)
{
	if(Address < m_Address)
	{
		return false;
	}

	if((m_eType == Program) && (Address >= (m_Address + m_RowSize * 2)))
	{
		return false;
	}

	if((m_eType == EEProm) && (Address >= (m_Address + m_RowSize * 2)))
	{
		return false;
	}

	if((m_eType == Configuration) && (Address >= (m_Address + 2)))
	{
		return false;
	}

	m_bEmpty    = false;

	sscanf(pData, "%4hx", &(m_Data[Address - m_Address]));
	
	return true;
}
/******************************************************************************/
void mem_cMemRow::FormatData(void)
{
	if(m_bEmpty == true)
	{
		return;
	}

	if(m_eType == Program)
	{
		for(int Count = 0; Count < m_RowSize; Count += 1)
		{
			m_pBuffer[0 + Count * 3] = (m_Data[Count * 2]     >> 8) & 0xFF;
			m_pBuffer[1 + Count * 3] = (m_Data[Count * 2])          & 0xFF;
			m_pBuffer[2 + Count * 3] = (m_Data[Count * 2 + 1] >> 8) & 0xFF;
		}
	}
	else if(m_eType == Configuration)
	{
		m_pBuffer[0] = (m_Data[0]  >> 8) & 0xFF;
		m_pBuffer[1] = (m_Data[0])       & 0xFF;
		m_pBuffer[2] = (m_Data[1]  >> 8) & 0xFF;
	}
	else
	{
		for(int Count = 0; Count < m_RowSize; Count++)
		{
			m_pBuffer[0 + Count * 2] = (m_Data[Count * 2] >> 8) & 0xFF;
			m_pBuffer[1 + Count * 2] = (m_Data[Count * 2])      & 0xFF;
		}
	}
}
void mem_cMemRow::PrintData()
{
	int addr=m_Address;
	if (!m_bEmpty)
	{
		if(m_eType == Program)
			printf("Program Memory: Row: %d Address: 0x%06x Empty: %d \n",m_RowNumber,m_Address,m_bEmpty);
		if(m_eType == Configuration)
			printf("Config Memory: Row: %d Address: 0x%06x Empty: %d \n",m_RowNumber,m_Address,m_bEmpty);
		if(m_eType == EEProm)
			printf("Eeprom Memory: Row: %d Address: 0x%06x Empty: %d \n",m_RowNumber,m_Address,m_bEmpty);
#if 1
		if(m_eType == Program)// && m_Address!=0)
		{
			for(int Count = 0; Count < m_RowSize * 3;)
			{
				printf("0x%06x: ", addr);
		
				printf("%02x",m_pBuffer[Count++] & 0xFF);
				printf("%02x",m_pBuffer[Count++] & 0xFF);
				printf("%02x ",m_pBuffer[Count++] & 0xFF);
		
				printf("%02x",m_pBuffer[Count++] & 0xFF);
				printf("%02x",m_pBuffer[Count++] & 0xFF);
				printf("%02x ",m_pBuffer[Count++] & 0xFF);
		
				printf("%02x",m_pBuffer[Count++] & 0xFF);
				printf("%02x",m_pBuffer[Count++] & 0xFF);
				printf("%02x ",m_pBuffer[Count++] & 0xFF);
		
				printf("%02x",m_pBuffer[Count++] & 0xFF);
				printf("%02x",m_pBuffer[Count++] & 0xFF);
				printf("%02x\n",m_pBuffer[Count++] & 0xFF);
		
				addr = addr + 8;
			}
		}
#endif
	}
}
/******************************************************************************/
void mem_cMemRow::SendData(M3EcSlaveShm * slave_shm)
{
	char Buffer[4] = {0,0,0,0};
	int local_flag = 1;

	if((m_bEmpty == true) && (m_eType != Configuration))
	{
		return;
	}

	//printf("Entering SendData...\n");

	//while(Buffer[0] != COMMAND_ACK)
	{
		if(m_eType == Program)// && m_Address!=0)
		{
			 
			Buffer[0] = (m_Address)       & 0xFF;
			Buffer[1] = (m_Address >> 8)  & 0xFF;
			Buffer[2] = (m_Address >> 16) & 0xFF;

			printf("\n\nPROGRAM...\n");
			printf("Sending Write command....Buffer[0] = %d\n", Buffer[0]);
			printf("Sending Write command....Buffer[1] = %d\n", Buffer[1]);
			printf("Sending Write command....Buffer[2] = %d\n", Buffer[2]);

			if (WriteCommBlock(slave_shm, COMMAND_WRITE_PM, Buffer, 3)<0)
			{
				printf("COMMAND_WRITE_PM failed\n");
				return;
			}
			int nw=WriteCommBlock(slave_shm,COMMAND_WRITE_PM_CON, m_pBuffer, m_RowSize * 3);
			if (nw!=m_RowSize * 3)
			{
				printf("COMMAND_WRITE_PM_CON failed. Wrote %d. Expected %d.\n",nw,m_RowSize * 3);
				return;
			}
		}
#if 0
		else if(m_eType == EEProm)
		{
			Buffer[0] = COMMAND_WRITE_EE;
			Buffer[1] = (m_Address)       & 0xFF;
			Buffer[2] = (m_Address >> 8)  & 0xFF;
			Buffer[3] = (m_Address >> 16) & 0xFF;

			printf("\n\nEEPROM...\n");
			printf("Sending Write command....Buffer[0] = %d\n", Buffer[0]);
			printf("Sending Write command....Buffer[1] = %d\n", Buffer[1]);
			printf("Sending Write command....Buffer[2] = %d\n", Buffer[2]);

			//WriteCommBlock(shm, COMMAND_WRITE_PM, Buffer, 4);
			//WriteCommBlock(shm,COMMAND_WRITE_PM_CON, m_pBuffer, m_RowSize * 3);

		}
		else if((m_eType == Configuration) && (m_RowNumber == 0))
		{
		
			printf("\n\nCOMMAND_WRITE_CM...\n");
			Buffer[0] = (char)(m_bEmpty)& 0xFF;
			Buffer[1] = m_pBuffer[0];
			Buffer[2] = m_pBuffer[1];

			/*spi_buffer[spi_buffer_ctr++] = COMMAND_WRITE_CM;
			spi_buffer[spi_buffer_ctr++] = (char)(m_bEmpty)& 0xFF;
			spi_buffer[spi_buffer_ctr++] = m_pBuffer[0];
			spi_buffer[spi_buffer_ctr++] = m_pBuffer[1];*/

			printf("\n\nCONFIGURATION AND ROWNUMBER = 0...\n");
			printf("Sending Write command....Buffer[0] = %d\n", Buffer[0]);
			printf("Sending Write command....Buffer[1] = %d\n", Buffer[1]);
			printf("Sending Write command....Buffer[2] = %d\n", Buffer[2]);

			//WriteCommBlock(shm, COMMAND_WRITE_CM, Buffer, 3);
			//WriteCommBlock(shm,COMMAND_WRITE_CM_CON, m_pBuffer, m_RowSize * 3);		
		}
		else if((m_eType == Configuration) && (m_RowNumber != 0))
		{
			if((m_eFamily == dsPIC30F) && (m_RowNumber == 7))
			{
				return;
			}

			Buffer[0] = (char)(m_bEmpty)& 0xFF;
			Buffer[1] = m_pBuffer[0];
			Buffer[2] = m_pBuffer[1];

			/*spi_buffer[spi_buffer_ctr++] = Buffer[0];
			spi_buffer[spi_buffer_ctr++] = Buffer[1];
			spi_buffer[spi_buffer_ctr++] = Buffer[2];*/

			printf("\n\nCONFIGURATION AND ROWNUMBER != 0...\n");
			printf("Sending Write command....Buffer[0] = %d\n", Buffer[0]);
			printf("Sending Write command....Buffer[1] = %d\n", Buffer[1]);
			printf("Sending Write command....Buffer[2] = %d\n", Buffer[2]);
			printf("Sending Write command....Buffer[3] = %d\n", Buffer[3]);

			//WriteCommBlock(shm, COMMAND_WRITE_CM, Buffer, 3);
			//WriteCommBlock(pComDev, Buffer, 3);				       		
		}

		else
		{
			assert(!"Unknown memory type");
		}
#endif
		//printf("Leaving SendData...\n");
	}
}

void mem_cMemRow::SendConfigData(M3EcSlaveShm * slave_shm)
{
	char Buffer[4];

	/*for(int i = 0; i < spi_buffer_ctr; i++)
		printf("Configuration[%d] = %d\n", i, spi_buffer[i]);
	printf("buffer_ctr = %d\n", spi_buffer_ctr);

	printf("Writing...\n");
	WriteCommBlock(shm, spi_buffer, buffer_ctr);

	while(Buffer[0] != COMMAND_ACK) {		
		ReceiveData(pComDev, Buffer, 1);
		printf("Received data (SendDataToSPI) = %d\n", Buffer[0]);
	}*/
	
}

	/******************************************************************************/

