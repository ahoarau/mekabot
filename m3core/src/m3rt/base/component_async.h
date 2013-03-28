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

#ifndef M3_COMPONENT_ASYNC_H
#define M3_COMPONENT_ASYNC_H

#include "m3rt/base/component.h"
#include <google/protobuf/message.h>

#ifdef __RTAI__	
#include <rtai.h>
#include "rtai_sem.h"
#endif


// Class to inherit from for async communications within M3


namespace m3rt
{
	using namespace std;

	static int num_asyncs = 0;
	
class M3ComponentAsync : public M3Component
{
	public:
		M3ComponentAsync(int p=EC_PRIORITY):M3Component(p),stop_thread(false),initializing(true),rc(NULL)
		{
			RegisterVersion("default",DEFAULT);	//RBL
			RegisterVersion("iss",ISS);		//ISS. Updated safety thresholds to use motor model.
			
		}
		
		virtual void StepAsync()=0;
		bool IsStopping(){return stop_thread;}
		void SignalStop(){stop_thread = true;}
		bool IsInitializing(){return initializing;}
		
#ifdef __RTAI__			
		SEM * cmd_mutex;
	        SEM * status_mutex;
#endif

		virtual google::protobuf::Message *  GetCommandAsync()=0;
		virtual google::protobuf::Message *  GetStatusAsync()=0;
		virtual google::protobuf::Message *  GetParamAsync()=0;
		
		virtual google::protobuf::Message *  GetCommandShared()=0;
		virtual google::protobuf::Message *  GetStatusShared()=0;
		virtual google::protobuf::Message *  GetParamShared()=0;
		
		virtual google::protobuf::Message * GetStatusThread()=0;
		bool initializing;
		
	protected:
		enum {DEFAULT, ISS};		
		
		
		virtual bool ReadConfig(const char * filename);
		virtual void Startup();
		virtual void Shutdown();
		virtual void StepStatus();
		virtual void StepCommand();			
		virtual bool LinkDependentComponents();				
	private:	      
	      long rc;

	      bool stop_thread;
	      int tmp;

};


}

#endif


