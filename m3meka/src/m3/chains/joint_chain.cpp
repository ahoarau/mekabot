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

#include "m3/chains/joint_chain.h"

namespace m3
{
	
using namespace m3rt;
using namespace std;
using namespace KDL;

///////////////////////////////////////////////////////
//Success so long as least one dependent comp. available
bool M3JointChain::LinkDependentComponents()
{		
	if (dynamatics_name!="")
		dynamatics=(M3Dynamatics*)factory->GetComponent(dynamatics_name);
			
	if (dynamatics==NULL)
		M3_INFO("M3Dynamatics component %s not found for component %s. Proceeding without it...\n",dynamatics_name.c_str(),GetName().c_str());
	
	return M3JointArray::LinkDependentComponents();
	
}


void M3JointChain::Startup()
{
	M3JointArray::Startup();
	G.resize(ndof);
}


void M3JointChain::SetG(int i, mReal val)
{
	if (joints[i]!=NULL)
	{
		G(i) = val;
		joints[i]->SetTorqueGravity(val);
	}

}

//Called after SetG is done all all joints.
//Now add in gravity terms from 'coupled joints' 
//This is a bit of a hack/workaround specific to MT2.J1. 
//Todo: generalize
void M3JointChain::SetGCoupled()
{
	for (int i=0;i<joints.size();i++)
	{
		if (joints[i]!=NULL)
		{
			M3Joint * cpj=joints[i]->GetCoupledJoint();
			if (cpj!=NULL)
			{
				for (int j=0;j<joints.size();j++)
				{
					if (joints[j]!=NULL && cpj==joints[j]) //found it
					{
						G(j)=G(j)+G(i);//couple in gravity term
						joints[j]->SetTorqueGravity(G(j));
					}
				}
			}
		}
 	}
}


bool M3JointChain::ReadConfig(const char * filename)
{	
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	
	doc["dynamatics_component"] >> dynamatics_name;	

	return M3JointArray::ReadConfig(filename);
}

}