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

#ifndef RT_LOG_SERVICE_H
#define RT_LOG_SERVICE_H

#include "m3rt/base/component.h"
#include "m3rt/base/component_base.pb.h"
#include "m3rt/base/toolbox.h"
#include <string>

#ifdef __RTAI__
#include <rtai.h>
#include "rtai_sem.h"
#endif


namespace m3rt
{
	using namespace std;

#define MAX_PAGE_QUEUE 300 //In case log service not stopped properly, force shutdown

class M3RtLogService
{
public:
	M3RtLogService(M3RtSystem * s, string n, string p, mReal freq,int ps,int vb):
		sys(s),name(n),path(p),start_idx(0),page(NULL),entry(NULL),page_size(ps),verbose(vb),num_page_write(0),num_kbyte_write(0),num_kbytes_in_buffer(0),entry_idx(0),page_idx_read(0),page_idx_write(0),pages_written(0)
	{
		downsample_rate = MAX(0,((int)((mReal)RT_TASK_FREQUENCY)/freq)-1); 
		downsample_cnt=0;
	}
	bool Startup();					//Called by M3RtService
	void Shutdown();				//Called by M3RtService
	bool WritePagesToDisk();			//Called by M3RtLogService thread
	void AddComponent(string name);			//Called by M3RtService
	bool Step();					//Called by M3RtSystem
	bool WriteEntry(bool final);	
	M3StatusLogPage * GetNextPageToRead();
	M3StatusLogPage * GetNextPageToWrite();
	void MarkPageEmpty();
	void MarkPageFull();
private:
	
	string GetNextFilename(int num_entry);
	string name;
	string path;
	M3StatusAll * entry;
	vector<M3StatusLogPage*> pages;
	vector<bool> is_page_full;
	vector<M3Component *> components;
	int start_idx;
	int downsample_cnt;
	int downsample_rate;
	M3StatusLogPage * page;
	M3RtSystem * sys;
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
};

}
#endif
