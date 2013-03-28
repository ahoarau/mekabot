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

#include <m3rt/base/component.h>

namespace m3rt
{


bool M3Component::ParseCommand(std::string & s)
{
	if (!GetCommand()->ParseFromString(s))
	{
		M3_ERR("Error in ParseCommand for %s\n",GetName().c_str());
		return false;
	}
	if (!GetCommand()->IsInitialized())
	{
		M3_INFO("Command Message not initialized for %s\n",GetName().c_str());
		return false;
	}
	//int n=GetCommand()->ByteSize();
	return true;
}
bool M3Component::ParseParam(std::string & s)
{
	if (!GetParam()->ParseFromString(s))
	{
		M3_ERR("Error in ParseParam for %s\n",GetName().c_str());
		return false;
	}
	if (!GetParam()->IsInitialized())
	{
		M3_INFO("Param Message not initialized for %s\n",GetName().c_str());
		return false;
	}
	//int n=GetParam()->ByteSize();
	return true;
}
bool M3Component::SerializeStatus(std::string & s)
{
	//M3_INFO("Serialize: %s with size %d %d %d\n",GetName().c_str(),s.size(),GetStatus()->ByteSize(),GetBaseStatus()->ByteSize());		
	//FixMe: We get occaisional ByteSize() error in message.cc line 235, protocol buffers, on Serialize to String
	//Fix appears to be to do with incorrect ByteSize in the EC messages (nested)
	//Ec byte size changes strangley depending on ctrl mode...
	int n=GetStatus()->ByteSize();
	int q=GetBaseStatus()->ByteSize();
	if (!GetStatus()->IsInitialized())
	{
		M3_INFO("Status Message not initialized for %s\n",GetName().c_str());
		return false;
	}
	if (!GetStatus()->SerializeToString(&s))
	{
		M3_ERR("Error in SerializeToString for %s\n",GetName().c_str());
		return false;
	}
	return true;
}
	
void M3Component::PrettyPrint()
{
	BannerPrint(80,"M3 Component");
	M3_PRINTF("Name: %s\n",GetName().c_str());
	M3_PRINTF("Version: %s\n",GetBaseStatus()->version().c_str());
	M3_PRINTF("Timestamp (uS): %lld\n",GetBaseStatus()->timestamp());
	const char* m3sys_state_names[]={
		"M3COMP_STATE_INIT",
		"M3COMP_STATE_ERR",
		"M3COMP_STATE_SAFEOP",
		"M3COMP_STATE_OP"};
		M3_PRINTF("State:  %s\n",m3sys_state_names[(int)GetBaseStatus()->state()]);
	BannerPrint(60,"Status");
	M3_PRINTF("%s\n",GetStatus()->DebugString().c_str());
	BannerPrint(60,"Param");
	M3_PRINTF("%s\n",GetParam()->DebugString().c_str());
	BannerPrint(60,"Command");
	M3_PRINTF("%s\n",GetCommand()->DebugString().c_str());
}

bool M3Component::ReadConfig(const char * filename)
{
	YAML::Node doc;
	string name;
	string version;
	GetYamlDoc(filename, doc);
	doc["name"] >> name;
	GetBaseStatus()->set_name(name);
	if (name=="") return false;
	try 
	{
		doc["version"]>>version;
		GetBaseStatus()->set_version(version);
	} catch(YAML::TypedKeyNotFound<string> e) 
	{
		//M3_WARN("Missing version key in config file for component %s. Defaulting to default\n",name.c_str());
		GetBaseStatus()->set_version("default");
	} 
	
	//Ugly solution...
	//Search to find the registered id to the given name
	int found=0;
	for (int i=0;i<version_names.size();i++)
	{
		if (version_names[i]==GetBaseStatus()->version())
		{
			version_id=version_ids[i];
			found=1;
			break;
		}
	}
	if (!found)
	{
		M3_ERR("Component %s was unable to find registered version %s upon loading\n",GetName().c_str(),GetBaseStatus()->version().c_str());
		return false;
	}
	version_ids.clear(); //no longer needed
	version_names.clear();
	return true;
}

}