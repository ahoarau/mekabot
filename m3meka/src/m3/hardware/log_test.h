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

#ifndef M3MEKA_LOG_TEST_H
#define M3MEKA_LOG_TEST_H


#include <m3/toolbox/log.h>
#include <m3/hardware/pwr.h>
#include <m3/hardware/log_test.pb.h>
#include <m3/hardware/pwr.pb.h>


///
///  This example shows how to use M3MekaLog base class
///  for custom logging of realtime components
///
///


namespace m3
{
	using namespace std;	
	using namespace m3rt;	

	
class M3MekaLogTest : public M3MekaLog
{
	public:
		M3MekaLogTest(): M3MekaLog()
		{			
			RegisterVersion("default",DEFAULT);		
		}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}		
		
	protected:
		enum {DEFAULT};
		bool ReadConfig(const char * filename);
		void Startup();		
		void StepCommand();
		M3MekaLogTestStatus status;
		M3MekaLogTestCommand command;
		M3MekaLogTestParam param;
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
		bool LinkDependentComponents();		
		
		string pwr_name;
		M3Pwr * pwr;		
};
}

#endif