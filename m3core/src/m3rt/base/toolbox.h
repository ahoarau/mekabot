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

#ifndef  M3RT_TOOLBOX_H
#define  M3RT_TOOLBOX_H

#include <vector>
#include <string>
#include <m3rt/base/m3rt_def.h>
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>


namespace m3rt
{
using namespace std;
#define M3_PRINTF printf
void M3_WARN(const char * format, ...);
void M3_ERR(const char * format, ...);
void M3_INFO(const char * format, ...);
void M3_DEBUG(const char * format, ...);
void BannerPrint(int width, const char *format, ...);
bool GetEnvironmentVar(const char * var, string &s);
vector<mReal> YamlReadVectorM(string s);
vector<string> YamlReadVectorString(string s);
vector<mReal> YamlReadVectorM(const YAML::Node& seq);
unsigned int xtoi(const char* xs);
void WriteYamlDoc(const char * filename, YAML::Emitter &doc);
void GetYamlDoc(const char * filename, YAML::Node & doc);
void operator >> (const YAML::Node& node, vector<mReal> & v);
}




#endif

