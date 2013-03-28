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

#ifndef M3_ASYNC_IO_H
#define M3_ASYNC_IO_H


#include "m3rt/base/component_async.h"

#ifdef __RTAI__
#include <rtai.h>
#include "rtai_sem.h"
#endif

#include "m3/hardware/async_io.pb.h"
#include <google/protobuf/message.h>


// Class to inherit from for async communications within M3


// do copies for messages
// add heartbeat
// switch to rtai
// whoever 


namespace m3
{
	using namespace std;

	
class M3AsyncIO : public m3rt::M3ComponentAsync
{
	public:
		M3AsyncIO(): m3rt::M3ComponentAsync(EC_PRIORITY),stop_thread(false)
		{
			RegisterVersion("default",DEFAULT);	//RBL
			RegisterVersion("iss",ISS);		//ISS. Updated safety thresholds to use motor model.
			
		}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}		
		google::protobuf::Message * GetParam(){return &param;}
		google::protobuf::Message * GetStatusThread(){return status.mutable_async();}
		google::protobuf::Message *  GetCommandAsync(){return &command_async;}
		google::protobuf::Message *  GetStatusAsync(){return &status_async;}
		google::protobuf::Message *  GetParamAsync(){return &param_async;}	
		google::protobuf::Message *  GetCommandShared(){return &command_shared;}
		google::protobuf::Message *  GetStatusShared(){return &status_shared;}
		google::protobuf::Message *  GetParamShared(){return &param_shared;}
		
		void StepAsync();
	protected:
		enum {DEFAULT, ISS};		
		
		
		bool ReadConfig(const char * filename);
		void Startup();
		void Shutdown();
		
		void StepStatus();
		void StepCommand();
								
		
		M3AsyncIOStatus status;
		M3AsyncIOCommand command;
		M3AsyncIOParam param;
								
		M3AsyncIOThreadStatus status_async;
		M3AsyncIOCommand command_async;		
		M3AsyncIOParam param_async;
		
		M3AsyncIOThreadStatus status_shared;
		M3AsyncIOCommand command_shared;		
		M3AsyncIOParam param_shared;
		
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}		

	private:
	    
	      int rc;
	      pthread_t thread;
	      bool stop_thread;
	      pthread_mutex_t cmd_mutex;
	      pthread_mutex_t status_mutex;
	
};


}

#endif


