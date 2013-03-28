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

#ifndef  M3RT_COMPONENT_EC_H
#define  M3RT_COMPONENT_EC_H

#include <m3rt/base/component.h>
#include <m3rt/base/component_base.pb.h>
#include <m3rt/base/m3ec_def.h>
#include <m3rt/base/toolbox.h>



namespace m3rt
{
////////////////////////////////////////////////////////////////

class M3ComponentEc: public M3Component{
	public:
		M3ComponentEc(int p=EC_PRIORITY):M3Component(p),shm(NULL),pdo_id(0),virtual_mode(0){}
		friend class M3RtSystem;
	protected:
		void PrettyPrint();
		//Deprecated: DSP is 16bit word, Client is 32
		//So when doing a sizeof() on a PDO struct that contains
		//an array, can be off by 2 as the client will pad align to word siz
		//This functionality is no longer needed, but must be careful in the future
		//of this effect.
		//virtual size_t GetStatusPdoSize()=0;
		//virtual size_t GetCommandPdoSize()=0;
		virtual M3EtherCATStatus * GetEcStatus()=0;
		virtual void SetStatusFromPdo(unsigned char * data)=0;
		virtual void SetPdoFromCommand(unsigned char * data)=0;
		
		//Override these to pump virtual hardware data through the system
		virtual void SetStatusFromPdoVirtual(){};
		virtual void SetPdoFromCommandVirtual(){};
		
		bool SetSlaveEcShm(M3EcSlaveShm * slaves, int slaves_responding);
		virtual bool ReadConfig(const char * filename);
		void RegisterPdo(const char * name, int id){pdo_names.push_back(name);pdo_ids.push_back(id);} 
		bool IsPdoVersion(int id){return pdo_id==id;}
		bool IsVirtualMode(){return virtual_mode;}
		virtual void Shutdown(){if (shm) ResetCommandPdo(shm->cmd);}
		virtual void Startup();
		virtual void StepStatus();
		virtual void StepCommand();
		//By convention, on M3 EtherCAT devices will go into a safe reset
		//state when given a PDO of zero. Override if this is not the case.
		virtual void ResetCommandPdo(unsigned char * pdo){memset(pdo,0,MAX_PDO_SIZE_BYTES);}
	private:
		M3EcSlaveShm * shm;
		bool IsEcError();
		int pdo_id;
		vector<string> pdo_names;
		vector<int> pdo_ids;
		int tmp_cnt;
		bool virtual_mode;
};

}
#endif

