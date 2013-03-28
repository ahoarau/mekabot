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

#include "log_test.h"
#include "m3rt/base/component_factory.h"



namespace m3 {

using namespace std;
using namespace m3rt;
  

bool M3MekaLogTest::LinkDependentComponents()
{
	
	pwr=(M3Pwr*) factory->GetComponent(pwr_name);
	if (pwr==NULL)
	{
		M3_INFO("M3Pwr component %s not found for component %s\n",pwr_name.c_str(),GetName().c_str());
		return false;
	}
	return true;
}

////////////////////////////////////////////////////////////


void M3MekaLogTest::Startup()
{  
	  
      int idx = factory->GetComponentIdx(GetName());
      components.push_back(factory->GetComponent(idx));
      
      M3MekaLog::Startup();
      return;

}


bool M3MekaLogTest::ReadConfig(const char * filename)
{
  YAML::Node doc;

  if (!M3MekaLog::ReadConfig(filename))
	  return false;
  GetYamlDoc(filename, doc);

    doc["pwr_component"] >> pwr_name;
  
   return true;
  
}



void M3MekaLogTest::StepCommand()
{
  if (IsStateSafeOp())
      return;
  

    M3PwrStatus * s = status.mutable_pwr_status();	  
    s->CopyFrom(*(pwr->GetStatus()));
    M3PwrCommand * c = status.mutable_pwr_cmd();	  
    c->CopyFrom(*(pwr->GetCommand()));
  
   M3MekaLog::StepCommand();
		    
    return;
}


}