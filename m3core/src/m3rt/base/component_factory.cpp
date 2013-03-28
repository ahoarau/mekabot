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

#include <m3rt/base/component_factory.h>
#include <m3rt/base/toolbox.h>
#include <dlfcn.h>
#include <iostream>
#include <map>
#include <list>
#include <vector>
#include <string>
#include <dlfcn.h>
#include <stdio.h>
#include <unistd.h>

namespace m3rt
{
	
using namespace std;
//global factory for making components
map< string, create_comp_t *, less<string> >  creator_factory;	//global
map< string, destroy_comp_t *, less<string> > destroyer_factory; //global

bool M3ComponentFactory::ReadConfig(const char * filename)
{
	YAML::Node doc;
	GetYamlDoc(filename, doc);

	for(unsigned i=0;i<doc["factory_rt_libs"].size();i++) 
	{
 		string lib;
    		doc["factory_rt_libs"][i] >> lib;
		AddComponentLibrary(lib);
    	}

	return true;
}

int M3ComponentFactory::GetComponentIdx(string name)
{
	for (int idx=0;idx<GetNumComponents();idx++)
	{	    
		if (name.compare(GetComponentName(idx))==0)
			return idx;
	}
	return -1;
}

M3Component * M3ComponentFactory::GetComponent(int idx)
{
	if (idx<=GetNumComponents()&&idx>=0)
		return m3_list[idx]; 
	else return NULL;
}

string  M3ComponentFactory::GetComponentType(int idx)
{
	if (idx<=GetNumComponents()) 
		return m3_types[idx]; 
	else return string("");
}
	
int 			M3ComponentFactory::GetNumComponents(){return m3_list.size();}
M3Component * 	M3ComponentFactory::GetComponent(string name){return GetComponent(GetComponentIdx(name));}

string  	M3ComponentFactory::GetComponentName(int idx)
{
	if (idx<=GetNumComponents())
		return GetComponent(idx)->GetName();
	else
		return string("");
}
	
bool M3ComponentFactory::AddComponentLibrary(string lib)
{
	void *dlib;
	dlib = dlopen(lib.c_str(), RTLD_LAZY);//RTLD_NOW);
	if(dlib == NULL)
	{
		M3_WARN("Unable to open M3 Component library %s. \nError: %s\n",lib.c_str(),dlerror());
		return false;
	}
	else
	{
		M3_INFO("Loaded M3 Component library: %s\n",lib.c_str());
		dl_list.push_back(dlib);
	}
	return true;
}

bool M3ComponentFactory::Startup()
{
	if (!ReadConfig(M3_COMP_LIB_FILENAME))
		return false;
	if (dl_list.size()==0)
	{
		M3_ERR("No M3 Component libraries available\n",0);
		return false;
	}
   // create an array of the type names
	BannerPrint(60,"Available component types");
	map<string, create_comp_t *, less<string> >::iterator i;
	for(i=creator_factory.begin(); i!=creator_factory.end();i++)
	{
		dl_types.push_back(i->first);
		M3_INFO("Component type: %s\n",i->first.c_str());
	}
	return true;
}

void M3ComponentFactory::Shutdown()
{
	ReleaseAllComponents();
	vector<void *>::iterator k; 
	int i=0;
	for(k=dl_list.begin(); k!=dl_list.end(); k++)
	{
		dlclose(*k);
		i++;
	}
	dl_list.clear();
	dl_types.clear();
}


bool M3ComponentFactory::ReleaseComponent(M3Component * c)
{
	vector<M3Component *>::iterator ci;
	vector<string>::iterator si;
	int idx=0;
	for (ci=m3_list.begin();ci!=m3_list.end();++ci)
	{
		if ((*ci)==c)
		{
			//M3_INFO("Release of %s %s\n",c->GetName().c_str(),m3_types[idx].c_str());
			destroyer_factory[m3_types[idx]](c);
			m3_list.erase(ci);
			si=m3_types.begin()+idx;
			m3_types.erase(si);
			return true;
		}
		idx++;
	}
	M3_WARN("Unable to destroy component %s\n",c->GetName().c_str());
	return false;
}

void M3ComponentFactory::ReleaseAllComponents()
{
	while(m3_list.size())
		ReleaseComponent(m3_list[0]);
}

M3Component* M3ComponentFactory::CreateComponent(string type)
{
	M3Component * m=NULL;
	vector<string>::iterator si;
	for (si=dl_types.begin();si!=dl_types.end();++si)
	{
		if ((*si).compare(type)==0)
		{
			m=creator_factory[type]();
			if (m!=NULL)
			{
				//M3_INFO("Create of: %s \n",type.c_str());
				m3_list.push_back(m);
				// If type ends in '_virtual', we want want to store it as it's base type
				int pos = type.find("virtual");
				if (pos != string::npos)
				{					
					type = type.substr(0,pos-1);				
				}
				m3_types.push_back(type);
			}
			break;
		}
	}
	if (m==NULL)
	{
		M3_WARN("Unable to create component of type %s \n",type.c_str());
	}
	return m;
}


}
