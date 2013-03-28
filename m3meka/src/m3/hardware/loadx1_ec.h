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

#ifndef M3_LOADX1_EC_H
#define M3_LOADX1_EC_H

#include <m3rt/base/component.h>
#include <m3rt/base/component_ec.h>
#include <m3/hardware/loadx1_ec.pb.h>
#include "m3/hardware/pwr.h"
#include "m3/toolbox/toolbox.h"
#include <google/protobuf/message.h>
#include "m3rt/base/m3ec_def.h"
#include "m3/hardware/m3ec_pdo_v1_def.h"


namespace m3{
using namespace std;

class M3LoadX1Ec : public  m3rt::M3ComponentEc{
	public:
		M3LoadX1Ec(): m3rt::M3ComponentEc()
		{
			RegisterVersion("default",DEFAULT);		//RBL
			RegisterVersion("iss",ISS);			//ISS. No change from DEFAULT
			RegisterPdo("actx1_pdo_v1", ACTX1_PDO_V1);	//RBL
		}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
	protected:
		bool ReadConfig(const char * filename);
		M3EtherCATStatus * GetEcStatus(){return status.mutable_ethercat();}
		size_t GetStatusPdoSize();
		size_t GetCommandPdoSize();
		void SetStatusFromPdo(unsigned char * data);
		void SetPdoFromCommand(unsigned char * data);
		bool LinkDependentComponents();
		void ResetCommandPdo(unsigned char * pdo);
	protected:
		enum {ACTX1_PDO_V1};
		enum {DEFAULT,ISS};
		M3BaseStatus * GetBaseStatus();
		M3LoadX1EcStatus status;
		M3LoadX1EcCommand command;
		M3LoadX1EcParam param;
		int tmp_cnt;
		int chid;
		int ptype;
};


































































































}
#endif


