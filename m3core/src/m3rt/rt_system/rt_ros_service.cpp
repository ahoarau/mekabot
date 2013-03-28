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

#include "m3rt/rt_system/rt_ros_service.h"
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_base.pb.h"
#include "m3rt/rt_system/rt_system.h"
#include <iostream>
#include <fstream>
#include <sstream>
//#include "m3/robots/humanoid.h"

#ifdef __RTAI__
#include <rtai.h>
#include <rtai_shm.h>
#include <rtai_sched.h>
#include <rtai_nam2num.h>
#include <rtai_sem.h>
#include <rtai_lxrt.h>
#endif

namespace m3rt
{
	using namespace std;
	using namespace ros;
	//using namespace m3;
static bool ros_thread_active=false;
static bool ros_thread_end=false;
///////////////////////////////////////////////////////////


static void* ros_thread(void * arg)
{
	M3RtRosService * svc = (M3RtRosService *)arg;
	ros_thread_active=true;
	ros_thread_end=false;
	
	while(!ros_thread_end)
	{
	      for (int i = 0; i < svc->publishers.size(); i++)
		svc->components_pub[i]->RosPublish(&(svc->publishers[i]));
	      ros::spinOnce();
	      usleep(10000);
		
	}	
	
	M3_INFO("Exiting M3 Ros Server Thread\n",0);
	//rt_task_delete(task);
	ros_thread_active=false;
	return 0;
}

////////////////////////////////////////////////////////////
// TODO:  move AddComponent stuff to AddComponent swig call
bool M3RtRosService::Startup()
{
	if (ros_thread_active)
	{
		M3_ERR("M3RtRosService thread already active\n",0);
		return false;
	}
	
	string s("ros/ros.yml");
	string path;
	
	if (GetEnvironmentVar(M3_ROBOT_ENV_VAR, path))
	{		
		path.append("/robot_config/");
		path.append(s);
	}
	ifstream fin(path.c_str());
	if (fin.fail())
	{		
		M3_ERR("Could not read %s \n", path.c_str());	
		return false;
	}	

   	YAML::Parser parser(fin);
   	YAML::Node doc;
   	parser.GetNextDocument(doc);
	fin.close();
	string name;
	
	ros::VP_string remappings;
	remappings.push_back(std::make_pair("foo", "bar"));
	ros::init(remappings,"m3_server");
	ros::NodeHandle node_handle;
	node_handle_ = &node_handle;
	
	/*for(int i=0; i<doc["ros_components"].size(); i++)
	{		
 		doc["ros_components"][i]>>name;		
		AddComponent(name);
	}*/
	
	vector<M3Component*>::iterator it;
	for(it=components.begin(); it!=components.end(); ++it)
	{
		services.push_back((*it)->RosInitCmd(node_handle_));
		services.push_back((*it)->RosInitParam(node_handle_));
		services.push_back((*it)->RosInitStatus(node_handle_));
		(*it)->RosExportParam(node_handle_);
	}
	
	for(int i=0; i<doc["ros_publishers"].size(); i++)
	{		
 		doc["ros_publishers"][i]>>name;		
		AddPublisher(name);
	}
	
	for(it=components_pub.begin(); it!=components_pub.end(); ++it)
	{
		publishers.push_back((*it)->RosInitPublish(node_handle_));
		(*it)->RosExportParam(node_handle_);
	}
	
#ifdef __RTAI__
	hlt=rt_thread_create((void*)ros_thread, (void*)this, 10000);
#else
	pthread_create((pthread_t *)&hlt, NULL, (void *(*)(void *))ros_thread, (void*)this);
#endif
	usleep(100000);
	if (!ros_thread_active)
	{
		M3_ERR("Unable to start M3RtRosService\n",0);
		return false;
	}
	
	return ros_thread_active;
}

/*bool M3RtRosService::RosCallback(m3ros::RosTestMsg::Request  &req, m3ros::RosTestMsg::Response &res)
{
 return true; 
}*/


////////////////////////////////////////////////////////////
void M3RtRosService::Shutdown()
{
	M3_INFO("Shutdown of M3RtRosService\n");
	ros_thread_end=true;
#ifdef __RTAI__
	rt_thread_join(hlt);
#else
	pthread_join((pthread_t)hlt, NULL);
#endif
	if (ros_thread_active) M3_WARN("M3RtRosService thread did not shut down correctly\n");
	
}

////////////////////////////////////////////////////////////
// TODO: make AddComponent call RosInit* functions to add components
//		Currently segfaulting... why?
// FIXME : Naming is confusing : component down here = ros service != component alone
//         Should be called something like components_serv.
bool M3RtRosService::AddComponent(string name)
{
	
	int idx=sys->GetComponentIdx(name);
	if (idx>=0)
	{
		for(int i=0;i<components.size(); i++)
			if(components[i]->GetName().compare(name)==0)
				return true;
		M3_INFO("Providing ROS service for component: %s\n",name.c_str());
		components.push_back(sys->GetComponent(idx));
		return true;
	}
 			
	
	M3_WARN("M3RtRosService component not available: %s\n",name.c_str());
	return false;
}		

bool M3RtRosService::AddPublisher(string name)
{
	
	int idx=sys->GetComponentIdx(name);
	if (idx>=0)
	{
		for(int i=0;i<components_pub.size(); i++)
			if(components_pub[i]->GetName().compare(name)==0)
				return true;
		M3_INFO("Providing ROS topic for component: %s\n",name.c_str());		
		components_pub.push_back(sys->GetComponent(idx));
		return true;
	}
 			
	
	M3_WARN("M3RtRosService component not available: %s\n",name.c_str());
	return false;
}	


////////////////////////////////////////////////////////////
bool M3RtRosService::Step()
{
	
	return true;
}		


	
}
