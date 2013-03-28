/* 
M3 -- Meka Robotics Robot Components
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

#ifndef M3_PWR_EC_H
#define M3_PWR_EC_H

#include <m3rt/base/component.h>
#include <m3rt/base/component_ec.h>
#include <m3/hardware/pwr_ec.pb.h>
#include <google/protobuf/message.h>


namespace m3{
using namespace std;



class M3PwrEc : public  m3rt::M3ComponentEc{
	public:
		M3PwrEc(): toggle(0),m3rt::M3ComponentEc()
		{
			RegisterVersion("default",DEFAULT);	//RBL
			RegisterVersion("iss",ISS);		//ISS. Same as DEFAULT
			RegisterVersion("hrl",HRL);
			RegisterPdo("pwr_pdo_v0",PWR_PDO_V0);	//HRL
			RegisterPdo("pwr_pdo_v1",PWR_PDO_V1);	//RBL
			RegisterPdo("pwr_pdo_v2",PWR_PDO_V2);	//Dropped mode_remote functionality
			enable_buzzer=0;
		}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
		void SetEnableBuzzer(int val){enable_buzzer=val;}
	protected:
		bool ReadConfig(const char * filename);
		M3EtherCATStatus * GetEcStatus(){return status.mutable_ethercat();}
		size_t GetStatusPdoSize();
		size_t GetCommandPdoSize();
		void SetStatusFromPdo(unsigned char * data);
		void SetPdoFromCommand(unsigned char * data);
		
	protected:
		enum {PWR_PDO_V1, PWR_PDO_V2,PWR_PDO_V0};
		enum {DEFAULT,ISS,HRL};
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
		M3PwrEcStatus status;
		M3PwrEcCommand command;
		M3PwrEcParam param;
		int tmp_cnt;
		int enable_buzzer;
		int toggle;
};

}
#endif


