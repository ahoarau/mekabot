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

#include "m3/chains/head.h"


namespace m3
{
	
using namespace m3rt;
using namespace std;

bool M3Head::ReadConfig(const char * filename)
{	
	if (!M3JointChain::ReadConfig(filename))
		return false;

	YAML::Node doc;
	GetYamlDoc(filename, doc);
	
	vector<mReal> left_trans;
	vector<mReal> right_trans;
	vector<mReal> left_rot;
	vector<mReal> right_rot;

	doc["eyes"]["right"]["rotation"]>>right_rot;
	doc["eyes"]["right"]["translation"]>>right_trans;
	doc["eyes"]["left"]["rotation"]>>left_rot;
	doc["eyes"]["left"]["translation"]>>left_trans;

	Rotation re_rot;
	Vector re_vec;
	Rotation le_rot;
	Vector le_vec;

	for (int i=0; i<9; i++)
	{
		re_rot.data[i] = right_rot[i];
		le_rot.data[i] = left_rot[i];
	}

	for (int i=0; i<3; i++)
	{
		re_vec[i] = right_trans[i];
		le_vec[i] = left_trans[i];
	}
	left_eye_offset = Frame(le_rot, le_vec);
	right_eye_offset = Frame(re_rot, re_vec);
}

}