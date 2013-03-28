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

#ifdef __RTAI__

#include "m3/hardware/async_io.h"
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_factory.h"
#include "inttypes.h"

namespace m3{
	
using namespace m3rt;
using namespace std;

///////////////////////////////////////////////////////


void M3AsyncIO::Startup()
{
  // initialize HW device here
  
  
  
    M3ComponentAsync::Startup();

}

void M3AsyncIO::StepStatus()
{    
  
    M3ComponentAsync::StepStatus();

}


void M3AsyncIO::StepCommand()
{
  
  
  
    M3ComponentAsync::StepCommand();

}


void M3AsyncIO::Shutdown()
{      
      // shutdown HW device here
  
      M3ComponentAsync::Shutdown();
}


void M3AsyncIO::StepAsync()
{      
      // do HW interfacing here
      // this gets called repeatedly from other thread
     
  
      M3AsyncIOThreadStatus * msg_to_copy_new_values_to = (M3AsyncIOThreadStatus *) GetStatusAsync();
      M3AsyncIOCommand * msg_to_get_cmds_from = (M3AsyncIOCommand * ) GetCommandAsync();
      M3AsyncIOParam * msg_to_get_params_from = (M3AsyncIOParam *) GetParamAsync();
      
      msg_to_copy_new_values_to->set_tmp(2.0);
      
      M3_DEBUG("got: %f\n", msg_to_get_cmds_from->tmp() );

      rt_sleep(10000000);

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3AsyncIO::ReadConfig(const char * filename)
{	
  
   
	int val;
	mReal mval;
	YAML::Node doc;

	if (!M3ComponentAsync::ReadConfig(filename))
		return false;
	GetYamlDoc(filename, doc);

	// read  config files here
	
	return true;
}

}

#endif