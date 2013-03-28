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

#ifndef M3_TACTILE_PPS22_EC_H
#define M3_TACTILE_PPS22_EC_H

#include <m3rt/base/component.h>
#include <m3rt/base/component_ec.h>
#include <m3/hardware/tactile_pps22_ec.pb.h>
#include <google/protobuf/message.h>
#include "m3rt/base/m3ec_def.h"
#include "m3/hardware/m3ec_pdo_v1_def.h"

namespace m3{
using namespace std;


class M3TactilePPS22Ec : public  m3rt::M3ComponentEc{
	public:
		M3TactilePPS22Ec():chid(0),m3rt::M3ComponentEc()
		{
			RegisterVersion("default",DEFAULT);		//HRL
			RegisterVersion("iss",ISS);			//ISS. No change from DEFAULT
			RegisterPdo("tactx2_pdo_v1",TACTX2_PDO_V1);	//HRL
		}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
	protected:
		virtual bool ReadConfig(const char * filename);
		virtual M3EtherCATStatus * GetEcStatus(){return status.mutable_ethercat();}
		virtual size_t GetStatusPdoSize(){return sizeof(M3TactX2PdoV1Status);}
		virtual size_t GetCommandPdoSize(){return 0;}
		virtual void SetStatusFromPdo(unsigned char * data);
		virtual void SetPdoFromCommand(unsigned char * data);
		virtual bool LinkDependentComponents();
		virtual void Startup();
	protected:
		enum {TACTX2_PDO_V1};
		enum {DEFAULT,ISS};
		M3BaseStatus * GetBaseStatus();
		M3TactilePPS22EcStatus status;
		M3TactilePPS22EcCommand command;
		M3TactilePPS22EcParam param;
		int chid;	
};

}
#endif


