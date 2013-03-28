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
					
#ifndef M3RT_COMPONENT_H
#define  M3RT_COMPONENT_H

#include <string>
#include <m3rt/base/component_base.pb.h>
#include <google/protobuf/message.h>
#include <google/protobuf/descriptor.h>
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <m3rt/base/toolbox.h>

namespace m3rt
{
using namespace std;
class M3ComponentFactory;
class M3RtSystem;

/*
A component can be in one of 4 states:
		M3COMP_STATE_INIT = 0;
		M3COMP_STATE_ERR = 1;
		M3COMP_STATE_SAFEOP = 2;
		M3COMP_STATE_OP = 3;
	
	State INIT: On creation, Startup() not yet called.
	State OP: Initialized, running normally. Status and Command data is modified. A module must be placed in OP by an external process.
	State SAFEOP: Only Status data is modified. SAFEOP is set externally in case of system error.
	State ERR: A non-recoverable error or safety exception has been triggered. 
	
	Each component must implement  Startup, Shutdown, StepStateOp, StepStateSafeOp, StepStateError . 
	After successful Startup,     it should do a SetStateSafeOp else SetStateError
	After successful StepStateOp, it should do a SetStateOp else SetStateError
	After successful StepStateSafeOp, it should do a SetStateSafeOp else SetStateError
	After a StepStateError, 	  it should do a SetStateError
	
	The client process must manually place each SafeOp component in state Op.
*/

class M3Component{
	public:
		M3Component(int p=0):priority(p),version_id(-1),factory(NULL){GOOGLE_PROTOBUF_VERIFY_VERSION;}
		virtual ~M3Component(){}
		friend class M3RtSystem;
		friend class M3RtLogService;
		string GetName(){return GetBaseStatus()->name();}
		int  GetState(){return (int)GetBaseStatus()->state();}
		int  GetPriority(){return priority;}
		int  SetPriority(int p){priority=p;}
		void SetStateError(){GetBaseStatus()->set_state(M3COMP_STATE_ERR);}
		void SetStateOp(){if (!IsStateError()) GetBaseStatus()->set_state(M3COMP_STATE_OP);}
		void SetStateSafeOp(){if (!IsStateError()) GetBaseStatus()->set_state(M3COMP_STATE_SAFEOP);}
		bool IsStateError(){return GetBaseStatus()->state() == M3COMP_STATE_ERR;}
		bool IsStateSafeOp(){return GetBaseStatus()->state() == M3COMP_STATE_SAFEOP;}
		bool IsStateOp(){return GetBaseStatus()->state() == M3COMP_STATE_OP;}
		bool IsRateFast(){return GetBaseStatus()->rate()=="fast";}
		bool IsRateMedium(){return GetBaseStatus()->rate()=="medium";}
		bool IsRateSlow(){return GetBaseStatus()->rate()=="slow";}
		bool IsVersion(int id){return version_id==id;} 
		void RegisterVersion(const char * name, int id){version_names.push_back(name);version_ids.push_back(id);} 
		virtual void Startup()=0;
		virtual void Shutdown()=0;
		virtual void StepStatus()=0;
		virtual void StepCommand()=0;
		
		
		void SetTimestamp(int64_t ts){GetBaseStatus()->set_timestamp(ts);}
		void GetTimestamp(int64_t ts){GetBaseStatus()->timestamp();}
		void SetFactory(M3ComponentFactory * f){factory=f;}
		
		virtual void PrettyPrint();
		virtual google::protobuf::Message *  GetCommand()=0;
		virtual google::protobuf::Message *  GetStatus()=0;
		virtual google::protobuf::Message *  GetParam()=0;
		void ParseCommandTest(string & s){}
		virtual bool SerializeStatus(string & s);
	protected:
		void RegisterVersionID(const char * name, int id);
		virtual bool ParseCommand(string & s);
		virtual bool ParseParam(string & s);		
		virtual bool LinkDependentComponents(){return true;}
		virtual M3BaseStatus *  GetBaseStatus()=0;
	protected:
		virtual bool ReadConfig(const char * filename);
		M3ComponentFactory * factory;
		int priority;
		vector<string> version_names;
		vector<int> version_ids;
		int version_id;
};

//Factory defn.
typedef M3Component * create_comp_t();
typedef void destroy_comp_t(M3Component *);
extern map< string, create_comp_t *, less<string> > creator_factory;	//global
extern map< string, destroy_comp_t *, less<string> > destroyer_factory; //global

}

#endif

