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

#include "m3rt/rt_system/rt_log_service.h"
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_base.pb.h"
#include "m3rt/rt_system/rt_system.h"
#include <iostream>
#include <fstream>
#include <sstream>

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
static bool log_thread_active=false;
static bool log_thread_end=false;
///////////////////////////////////////////////////////////


static void* log_thread(void * arg)
{
	M3RtLogService * svc = (M3RtLogService *)arg;
	log_thread_active=true;
	log_thread_end=false;
	// TODO: Add the semaphore back in?
#ifdef __RTAI__	
	RT_TASK *task;
	task = rt_task_init_schmod(nam2num("M3LSV"), 0, 0, 0, SCHED_FIFO, 0xF); 
	if (task==NULL)
	{
		M3_ERR("Failed to create M3RtLogService RT Task\n",0);
		return 0;
	}
 	rt_allow_nonroot_hrt();
	mlockall(MCL_CURRENT | MCL_FUTURE);
	rt_make_soft_real_time();
	/*svc->ext_sem=(SEM*)rt_get_adr(nam2num(SEMNAM_M3LEXT));
	if (!svc->ext_sem)
	{
		rt_task_delete(task);
		M3_ERR("M3RtLogService unable to create the SEMNAM_M3LEXT semaphore.\n",0);
		return 0;
	}
	else
		M3_INFO("M3RtLogService allocated ext_sem semaphore  %08x \n",svc->ext_sem);*/
#endif	
	while(!log_thread_end)
	{
		if (!svc->WritePagesToDisk())
			break;
		usleep(100000);
	}	
	svc->WriteEntry(true);
	svc->WritePagesToDisk();
	M3_DEBUG("Exiting M3 Log Server Thread\n",0);
#ifdef __RTAI__	
	rt_task_delete(task);
#endif
	log_thread_active=false;
	return 0;
}
////////////////////////////////////////////////////////////
bool M3RtLogService::Startup()
{
	if (log_thread_active)
	{
		M3_ERR("M3RtLogService thread already active\n",0);
		return false;
	}
	
	//NOTE: This serialization needs to be done once only
	//Likely a semaphore is needed on rt_system
	//Otherwise can gen a protobuf error. Moved this out of the loop below.
	vector<string> datums;
	vector<M3Component*>::iterator k;
	  for(k=components.begin(); k!=components.end(); ++k)
	  {
		  string d;
		  if (!(*k)->SerializeStatus(d))
			  return false;
		  datums.push_back(d);
	  }	   
	 
	//  Allocate all the memory now cause we don't want to do it in realtime loops
	for (int i = 0; i < MAX_PAGE_QUEUE; i++)
	{
	  page=new M3StatusLogPage();
	  for (int j = 0; j < page_size; j++)
	  {
	    int si=0;
	    M3StatusAll * entry = page->add_entry();
	    for(k=components.begin(); k!=components.end(); ++k)
	    {
		    entry->add_datum(datums[si]);
		    entry->add_name((*k)->GetName());
		    si++;
	    }	   
	  }	   
	  pages.push_back(page);
	  is_page_full.push_back(false);	  	
	}
	page = pages[0];

#ifdef __RTAI__
	hlt=rt_thread_create((void*)log_thread, (void*)this, 10000);
#else
	pthread_create((pthread_t *)&hlt, NULL, (void *(*)(void *))log_thread, (void*)this);
#endif
	usleep(100000);
	if (!log_thread_active)
	{
		M3_ERR("Unable to start M3RtLogService\n",0);
		return false;
	}
	return log_thread_active;
}
////////////////////////////////////////////////////////////
void M3RtLogService::Shutdown()
{
	M3_DEBUG("M3RtLogService %s: Pages Written: %d\n",name.c_str(),num_page_write);
	M3_DEBUG("M3RtLogService %s: KByte Written: %d\n",name.c_str(),num_kbyte_write);
	M3_DEBUG("M3RtLogService %s. Shutting down...\n",name.c_str());
	
	log_thread_end=true;
#ifdef __RTAI__
	rt_thread_join(hlt);
#else
	pthread_join((pthread_t)hlt, NULL);
#endif
	if (log_thread_active) M3_WARN("M3RtLogService thread did not shut down correctly\n");
	while (pages.size()) 
	{
		M3StatusLogPage * p=pages[0];
		pages.erase(pages.begin());
		delete p;
	}
	
	page=NULL;
}

////////////////////////////////////////////////////////////
void M3RtLogService::AddComponent(string name)
{
	int idx=sys->GetComponentIdx(name);
	if (idx>=0)
	{
		for(int i=0;i<components.size(); i++)
			if(components[i]->GetName().compare(name)==0)
				return;
		M3_DEBUG("Logging component: %s\n",name.c_str());
		components.push_back(sys->GetComponent(idx));
		return;
	}
	M3_WARN("M3RtLogService component not available: %s\n",name.c_str());
}		
////////////////////////////////////////////////////////////
bool M3RtLogService::Step()
{
  
	if (downsample_cnt==0)
	{
		downsample_cnt=downsample_rate;		
		M3StatusAll * entry = page->mutable_entry(entry_idx);
		string * datum;
		vector<M3Component*>::iterator j;
		int k = 0;
		for(j=components.begin(); j!=components.end(); ++j)
		{
			datum = entry->mutable_datum(k);
			if (!(*j)->SerializeStatus(*datum))
				return false;
			entry->set_datum(k,*datum);
			entry->set_name(k,(*j)->GetName());
			k++;
		}
		entry_idx++;
		return WriteEntry(false);		
	}
	downsample_cnt--;	
	return true;
}		
////////////////////////////////////////////////////////////
bool  M3RtLogService::WriteEntry(bool final)
{
	if (page == NULL)
		return false;
	if (entry_idx >= page_size || final)
	{		
		if (final)
			page=NULL;
		else 
		{				
			entry_idx = 0;
			MarkPageFull();
			page = GetNextPageToWrite();
		}					
	}
	return true;
}
////////////////////////////////////////////////////////////
string  M3RtLogService::GetNextFilename(int num_entry)
{
	int i,nl;
	string filename=path+"/"+name+"_";
	ostringstream os,os2;
	os << start_idx;
	string t_str = os.str(); //retrieve as a string
	nl=6-t_str.size();
	for (i=0;i<nl;i++)
		filename+="0";
	filename += t_str;
	filename+="_";
	os2 << start_idx+num_entry-1;
	t_str = os2.str(); //retrieve as a string
	nl=6-t_str.size();
	for (i=0;i<nl;i++)
		filename+="0";
	filename += t_str;
	filename +=".pb.log";
	start_idx+=num_entry;
	return filename;
}
////////////////////////////////////////////////////////////

void M3RtLogService::MarkPageFull()
{
  if (is_page_full[page_idx_write])
    M3_ERR("Buffer over run in log pages...\n");
  
  is_page_full[page_idx_write] = true;
}

void M3RtLogService::MarkPageEmpty()
{
  is_page_full[page_idx_read] = false;
}

M3StatusLogPage * M3RtLogService::GetNextPageToWrite()
{  
  page_idx_write++;
  if (page_idx_write >= MAX_PAGE_QUEUE)
  {
    page_idx_write = 0;
  }
  
  return pages[page_idx_write];    
}

M3StatusLogPage * M3RtLogService::GetNextPageToRead()
{
  M3StatusLogPage * next_page;
  
  if (is_page_full[page_idx_read])
  {
    next_page = pages[page_idx_read];
    page_idx_read++;
    if (page_idx_read >= MAX_PAGE_QUEUE)
    {
      page_idx_read = 0;
    }
  }
  else
    next_page = NULL;
    
  return next_page;      
}

bool M3RtLogService::WritePagesToDisk()
{
	if (page==NULL)
		return false;
	//if (verbose)
	//  M3_INFO("M3RtLogService Pages: %d\n",pages.size());
	M3StatusLogPage * p = GetNextPageToRead();	
	
	while (p)
	{				
		string filename=GetNextFilename(p->entry_size());
		if (verbose)
		  M3_DEBUG("Writing logfile %d: %s of size %dK\n",pages_written,filename.c_str(),p->ByteSize()/1024);
		pages_written++;
		fstream output(filename.c_str(), ios::out | ios::trunc | ios::binary);
		if (!p->SerializeToOstream(&output)) 
		{
			M3_ERR("Failed to write logfile %s.",filename.c_str());
			return false;
		}
		num_page_write++;		
		num_kbyte_write+=p->ByteSize()/1024;		
		MarkPageEmpty();
		p = GetNextPageToRead();
	}
	return true;
}

	
}