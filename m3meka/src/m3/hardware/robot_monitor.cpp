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

#include "m3/hardware/robot_monitor.h"
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_factory.h"


namespace m3{
	
using namespace m3rt;
using namespace std;

				 
void M3RobotMonitor::Startup()
{
	for (int i = 0; i < temp_names.size(); i++)
	{	  
	  status.add_temp_comp();
	  status.mutable_temp_comp(i)->set_component_name(temp_names[i]);
	}
	
	for (int i = 0; i < volt_names.size(); i++)
	{	  
	  status.add_volt_comp();
	  status.mutable_volt_comp(i)->set_component_name(volt_names[i]);
	}
}

void M3RobotMonitor::Shutdown()
{
	
	
}

void M3RobotMonitor::StepStatus()
{
	//TODO: set error messages, should we SetStateErr or keep in in ERR state if detected?
	for (int i=0;i<temp_comps.size();i++)
	{
	  if (temp_comps[i])
	  {
	      mReal val = temp_comps[i]->GetExtTemp(); //TODO: implement desired temperature method
	      status.mutable_temp_comp(i)->set_state(ROBOT_MONITOR_OKAY);
	      if (val > param.temp_comp(i).max_val_warn())
	      {
		status.mutable_temp_comp(i)->set_state(ROBOT_MONITOR_HIGH_WARN);
	      } 
	      if (val < param.temp_comp(i).min_val_warn())
	      {
		status.mutable_temp_comp(i)->set_state(ROBOT_MONITOR_LOW_WARN);
	      } 
	      if (val > param.temp_comp(i).max_val_err())
	      {
		status.mutable_temp_comp(i)->set_state(ROBOT_MONITOR_HIGH_ERR);
	      } 
	      if (val < param.temp_comp(i).min_val_err())
	      {
		status.mutable_temp_comp(i)->set_state(ROBOT_MONITOR_LOW_ERR);
	      } 
	  }
	}
	
	for (int i=0;i<volt_comps.size();i++)
	{
	  if (volt_comps[i])
	  {
	      mReal val = volt_comps[i]->GetBusVoltage();
	      status.mutable_volt_comp(i)->set_state(ROBOT_MONITOR_OKAY);
	      if (val > param.volt_comp(i).max_val_warn())
	      {
		status.mutable_volt_comp(i)->set_state(ROBOT_MONITOR_HIGH_WARN);
	      } 
	      if (val < param.volt_comp(i).min_val_warn())
	      {
		status.mutable_volt_comp(i)->set_state(ROBOT_MONITOR_LOW_WARN);
	      } 
	      if (val > param.volt_comp(i).max_val_err())
	      {
		status.mutable_volt_comp(i)->set_state(ROBOT_MONITOR_HIGH_ERR);
	      } 
	      if (val < param.volt_comp(i).min_val_err())
	      {
		status.mutable_volt_comp(i)->set_state(ROBOT_MONITOR_LOW_ERR);
	      } 
	  }
	}
}

void M3RobotMonitor::StepCommand()
{
	
	
}

bool M3RobotMonitor::LinkDependentComponents()
{	
	
	int n=0;
	if (temp_names.size()>0)
		temp_comps.assign(temp_names.size(),(M3Actuator*)NULL);
	
	for (int i=0;i<temp_names.size();i++)
	{
		if(temp_names[i].size())
		{
			temp_comps[i]=(M3Actuator*)factory->GetComponent(temp_names[i]);			
			if (temp_comps[i]!=NULL)
				n++;
		}
		else
			temp_comps[i]=NULL;
	}	
			
	if (n!=temp_names.size())
	{
		M3_ERR("M3RobotMonitor %s found %d temp components. Expected %d. Continuing with missing components. \n",
		       GetName().c_str(),n,temp_names.size());	
	}
	
	n=0;
	if (volt_names.size()>0)
		volt_comps.assign(volt_names.size(),(M3Pwr*)NULL);
	
	for (int i=0;i<volt_names.size();i++)
	{
		if(volt_names[i].size())
		{
			volt_comps[i]=(M3Pwr*)factory->GetComponent(volt_names[i]);			
			if (volt_comps[i]!=NULL)
				n++;
		}
		else
			volt_comps[i]=NULL;
	}	
			
	if (n!=volt_names.size())
	{
		M3_ERR("M3RobotMonitor %s found %d volt components. Expected %d. Continuing with missing components. \n",
		       GetName().c_str(),n,volt_names.size());	
	}
	
	return true;
	
}

bool M3RobotMonitor::ReadConfig(const char * filename)
{
	if (!M3Component::ReadConfig(filename))
		return false;
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	
	bool has_volt = true;
	try 
	{
	    YAML::Iterator it=doc["volt_components"].begin();
	}catch(YAML::TypedKeyNotFound<string> e) 
	{
	  has_volt = false;
	}
	
	int i = 0;
	if (has_volt)
	{
	  for(YAML::Iterator it=doc["volt_components"].begin();it!=doc["volt_components"].end();++it) 
	  {		
		  string key;
		   mReal val;
		  it.first() >> key;
		  volt_names.push_back(key);
		  param.add_volt_comp();
		  param.mutable_volt_comp(i)->set_component_name(key);
		  doc["volt_components"][key]["max_warn"] >> val;
		  param.mutable_volt_comp(i)->set_max_val_warn(val);
		  doc["volt_components"][key]["min_warn"] >> val;
		  param.mutable_volt_comp(i)->set_min_val_warn(val);
		  doc["volt_components"][key]["max_err"] >> val;
		  param.mutable_volt_comp(i)->set_max_val_err(val);
		  doc["volt_components"][key]["min_err"] >> val;
		  param.mutable_volt_comp(i)->set_min_val_err(val);
		  i++;
	  }
	}
	
	bool has_temp = true;
	try 
	{
	    YAML::Iterator it=doc["temp_components"].begin();
	}catch(YAML::TypedKeyNotFound<string> e) 
	{
	  has_temp = false;
	}
	i = 0;
	if (has_temp)
	{
	  for(YAML::Iterator it=doc["temp_components"].begin();it!=doc["temp_components"].end();++it) 
	  {		
		  string key;
		  mReal val;
		  it.first() >> key;
		  temp_names.push_back(key);
		  param.add_temp_comp();
		  param.mutable_temp_comp(i)->set_component_name(key);		  
		  doc["temp_components"][key]["max_warn"] >> val;
		  param.mutable_temp_comp(i)->set_max_val_warn(val);
		  doc["temp_components"][key]["min_warn"] >> val;
		  param.mutable_temp_comp(i)->set_min_val_warn(val);
		  doc["temp_components"][key]["max_err"] >> val;
		  param.mutable_temp_comp(i)->set_max_val_err(val);
		  doc["temp_components"][key]["min_err"] >> val;
		  param.mutable_temp_comp(i)->set_min_val_err(val);
		  i++;
	  }
	}
	return true;
}


} // m3 namespace
	