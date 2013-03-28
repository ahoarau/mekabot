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

#include <m3rt/base/component_ec.h>

namespace m3rt
{
	
//Search for matching slave ID
bool M3ComponentEc::SetSlaveEcShm(M3EcSlaveShm * slaves, int slaves_responding)
{
	M3EtherCATStatus * status = GetEcStatus();
	shm=NULL;
	if (virtual_mode)
	{
	  M3_INFO("Loading  EtherCAT  Slave %s (Serial %d BusID %d ProductCode %d) in Virtual Mode\n",
					GetName().c_str(),status->serial_number(), status->network_id(), status->product_code());
	  return true;
	}
	
	for (int i=0;i<slaves_responding;i++)
	{
	      
		M3EcSlaveShm * slave = &(slaves[i]);
		//M3_DEBUG("sn: %d  pc: %d  nid: %d\n",slave->serial_number,slave->product_code,slave->network_id);
		if (slave->active)
		{
			if (slave->serial_number==status->serial_number() && slave->product_code==status->product_code())
			{
				shm=slave;
				status->set_network_id(slave->network_id);
				M3_INFO("Found  active  EtherCAT  Slave for component %s (Serial %d BusID %d ProductCode %d)\n",
					GetName().c_str(),status->serial_number(), status->network_id(), status->product_code());
				return true;
			}
		}
	}
	M3_INFO("No active EtherCAT Slave found for component %s (Serial %d BusID %d ProductCode %d)\n",
			GetName().c_str(),status->serial_number(),status->network_id(),status->product_code());
	return false;
}

bool M3ComponentEc::ReadConfig(const char * filename)
{
	YAML::Node doc;
	int val;
	string version;
	if (!M3Component::ReadConfig(filename)) return false;
	GetYamlDoc(filename, doc);
	const YAML::Node& ec = doc["ethercat"];
	M3EtherCATStatus * status = GetEcStatus();
	ec["serial_number"] >> val;
	status->set_serial_number(val);
	ec["product_code"] >> val;
	status->set_product_code(val);
	ec["pdo_version"] >> version;
	status->set_pdo_version(version);
	 try
	  {
		  doc["virtual_mode"] >> val;
		  virtual_mode = (bool) val;
	  } catch(YAML::TypedKeyNotFound<string> e) 
	  {
		  virtual_mode = (bool) 0;
	  } 


	//Search to find the registered id to the given name
	int found=0;
	for (int i=0;i<pdo_names.size();i++)
	{
		if (pdo_names[i]==version)
		{
			pdo_id=pdo_ids[i];
			found=1;
			break;
		}
	}
	if (!found)
	{
		M3_ERR("Component %s was unable to find a registered PDO version %s upon loading\n",GetName().c_str(),status->pdo_version().c_str());
		return false;
	}
	pdo_ids.clear(); //no longer needed
	pdo_names.clear();
	
	return true;
}


void  M3ComponentEc::StepStatus()
{
    M3EtherCATStatus * status = GetEcStatus();
    if (virtual_mode)
    {
	status->set_online(1);
	status->set_operational(1);
	status->set_al_state(8);
	status->set_active(1);
	SetStatusFromPdoVirtual(); 
    }
    else
    {
	if (!shm)
	{
		SetStateSafeOp();
		return;
	}
	if (!IsStateError())
		SetStatusFromPdo(shm->status);
	
	status->set_online(shm->online);
	status->set_operational(shm->operational);
	status->set_al_state(shm->al_state);
	status->set_active(shm->active);
	//Do this last 
	if (IsEcError())
		SetStateError();
    }
}

void  M3ComponentEc::StepCommand()
{
  if (virtual_mode)
  {
    SetPdoFromCommandVirtual();
  }
  else
  {
	if (!shm) return;
	if (!IsStateOp())
		ResetCommandPdo(shm->cmd);
	else	
		SetPdoFromCommand(shm->cmd);
  }
}

//Wait to call IsEcError, needs a step to init.
void  M3ComponentEc::Startup(){SetStateSafeOp();}

		
bool M3ComponentEc::IsEcError()
{
	M3EtherCATStatus * status = GetEcStatus();
	if (!status->online())
	{
		if (!IsStateError())
		{
			M3_ERR("M3ComponentEc component %s is not online\n",GetName().c_str());
		}
		return true;
	}
	if (!status->operational())
	{
		if (!IsStateError())
		{
			M3_ERR("M3ComponentEc component %s is not operational\n",GetName().c_str());
		}
		return true;
	}
	if (!status->active())
	{
		if (!IsStateError())
		{
			M3_ERR("M3ComponentEc component %s is not active\n",GetName().c_str());
		}
		return true;
	}
	if (!status->al_state()==8)
	{
		if (!IsStateError())
		{
			M3_ERR("M3ComponentEc component %s state is not OP. Is %d\n",GetName().c_str(),status->al_state() );
		}
		return true;
	}
	return false;
}

void M3ComponentEc::PrettyPrint()
{
	M3Component::PrettyPrint();
	BannerPrint(60,"EtherCAT");
	BannerPrint(60,"Status");
	M3EtherCATStatus * status = GetEcStatus();
	M3_PRINTF("active : %d\n",status->active());
	M3_PRINTF("network_id : %d\n",status->network_id());
	M3_PRINTF("serial_number : %d\n",status->serial_number());
	M3_PRINTF("product_code : %d\n",status->product_code());
	M3_PRINTF("online : %d\n",status->online());
	M3_PRINTF("operational : %d\n",status->operational());
	M3_PRINTF("al_state : %d\n",status->al_state());
	M3_PRINTF("pdo_version : %s\n",status->pdo_version().c_str());
}


}	
