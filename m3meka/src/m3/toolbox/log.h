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

#ifndef M3MEKA_LOG_H
#define M3MEKA_LOG_H

#include <google/protobuf/message.h>

#include <m3/toolbox/toolbox.h>
#include <m3/toolbox/log.pb.h>
#include <m3rt/base/m3ec_def.h>
#include <m3rt/base/component.h>
#include <m3rt/base/component_ec.h>


#define MAX_PAGE_QUEUE 300 //In case log service not stopped properly, force shutdown

namespace m3
{
	using namespace std;	
	using namespace m3rt;	

	
class M3MekaLog : public m3rt::M3Component
{
	public:
		M3MekaLog(): m3rt::M3Component(CALIB_PRIORITY),start_idx(0),num_page_write(0),num_kbyte_write(0),num_kbytes_in_buffer(0),entry_idx(0),page_idx_read(0),page_idx_write(0),pages_written(0)
		{			
			RegisterVersion("default",DEFAULT);		
		}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}		
		bool WritePagesToDisk();
		bool WriteEntry(bool final);	
	protected:
		enum {DEFAULT};
		bool ReadConfig(const char * filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		M3MekaLogStatus status;
		M3MekaLogCommand command;
		M3MekaLogParam param;
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
		bool LinkDependentComponents();		
		M3StatusLogPage * GetNextPageToRead();
		M3StatusLogPage * GetNextPageToWrite();
		void MarkPageEmpty();
		void MarkPageFull();
		vector<M3Component *> components;
		vector<string> comp_names;
		string GetNextFilename(int num_entry);
		string log_name;
		string path;
		M3StatusAll * entry;
		vector<M3StatusLogPage*> pages;
		vector<bool> is_page_full;		
		int start_idx;
		int downsample_cnt;
		int downsample_rate;
		M3StatusLogPage * page;
		int page_size;
		int hlt;
		int verbose;
		int num_page_write;
		int num_kbyte_write;
		int num_kbytes_in_buffer;
		int entry_idx;
		int page_idx_write;
		int page_idx_read;
		int pages_written;
		mReal freq;
		bool enable;
};
}

#endif