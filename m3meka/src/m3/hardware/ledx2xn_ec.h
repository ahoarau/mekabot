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

#ifndef M3_LEDX2XN_EC_H
#define M3_LEDX2XN_EC_H

#include <m3rt/base/component.h>
#include <m3rt/base/component_ec.h>
#include <m3/hardware/ledx2xn_ec.pb.h>
#include <google/protobuf/message.h>
#include "m3rt/base/m3ec_def.h"
#include "m3/hardware/m3ec_pdo_v1_def.h"

namespace m3{
using namespace std;


class M3LedX2XNEc : public  m3rt::M3ComponentEc{
	public:
		M3LedX2XNEc(): n_branch_b(0),n_branch_a(0),id_a(0),id_b(0),m3rt::M3ComponentEc()
		{
			RegisterVersion("default",DEFAULT);		//ISS
			RegisterVersion("iss",ISS);			//ISS. No change from DEFAULT
			RegisterPdo("ledx2xn_pdo_v1",LEDX2XN_PDO_V1);	//ISS ledx2xn_pdo_v1
		}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
	protected:
		void Startup();
		bool ReadConfig(const char * filename);
		M3EtherCATStatus * GetEcStatus(){return status.mutable_ethercat();}
		size_t GetStatusPdoSize(){return sizeof(M3LedX2XNPdoV1Status);}
		size_t GetCommandPdoSize(){return sizeof(M3LedX2XNPdoV1Cmd);} 
		void SetStatusFromPdo(unsigned char * data);
		void SetPdoFromCommand(unsigned char * data);

		enum {LEDX2XN_PDO_V1};
		enum {DEFAULT,ISS};
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
		M3LedX2XNEcStatus status;
		M3LedX2XNEcCommand command;
		M3LedX2XNEcParam param;
		int n_branch_a;
		int n_branch_b;
		int id_a;
		int id_b;
		int tmp_cnt;
};

}
#endif


