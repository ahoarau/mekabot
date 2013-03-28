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

#ifndef _mem_h
#define _mem_h

#include "m3rt/bootloader/m3_bootloader.h"
#include <m3rt/base/m3ec_def.h>



class mem_cMemRow
{
public:

	enum eType
	{
		Program,
		EEProm,
		Configuration
	};
	mem_cMemRow(eType Type, unsigned int StartAddr, int RowNumber, eFamily Family);
	bool InsertData(unsigned int Address, char * pData);
	void FormatData(void);
	void SendData  (M3EcSlaveShm * slave_shm);
	void SendConfigData(M3EcSlaveShm * slave_shm);
	void PrintData();
private:
	char           * m_pBuffer;
	unsigned int     m_Address;
	bool             m_bEmpty;
	eType            m_eType;
	unsigned short   m_Data[PM33F_ROW_SIZE*2];
	int              m_RowNumber;
	eFamily          m_eFamily;
	int		m_RowSize;	
};


#endif
