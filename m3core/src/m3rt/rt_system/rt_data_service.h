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

#ifndef RT_DATA_SERVICE_H
#define RT_DATA_SERVICE_H

#include "m3rt/base/simple_server.h"
#include "m3rt/base/component_base.pb.h"
#include "m3rt/base/toolbox.h"
#include "m3rt/rt_system/rt_system.h"
#include <pthread.h>
#include <string>

#ifdef __RTAI__
#include <rtai.h>
#include "rtai_sem.h"
#else
#include <semaphore.h>
#include <pthread.h>
#endif

namespace m3rt
{
	using namespace std;
class M3RtDataService
{
public:
	M3RtDataService(M3RtSystem * s, int port):sys(s),data_thread_active(false),data_thread_error(false),data_thread_end(false),portno(port){}
	bool Startup();							//Start thread, open port
	void Shutdown();						//Stop thread, close port
	bool StartServer(){return server.Startup(portno);}
	bool Step();
	void ClientSubscribeStatus(string name);
	bool data_thread_active;
	bool data_thread_end;
	bool data_thread_error;
	static int instances;
private:
	M3StatusAll status;
	M3SimpleServer server;
	int portno;
	
	string sread;
	string swrite;
	M3RtSystem * sys;
	vector<string> status_names;
	long hdt;
#ifdef __RTAI__	
	SEM * ext_sem;
#else
	sem_t * ext_sem;
#endif
};

}
#endif
