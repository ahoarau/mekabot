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

#include "log.h"
#include <stdio.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "m3rt/base/component_factory.h"

#ifdef __RTAI__
#include <rtai.h>
#include <rtai_shm.h>
#include <rtai_sched.h>
#include <rtai_nam2num.h>
#include <rtai_sem.h>
#include <rtai_lxrt.h>
#endif


namespace m3 {

using namespace std;
using namespace m3rt;

static bool log_thread_active=false;
static bool log_thread_end=false;
  

static void* log_thread(void * arg)
{
	M3MekaLog * svc = (M3MekaLog *)arg;
	log_thread_active=true;
	log_thread_end=false;
	
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
#endif	
	while(!log_thread_end)
	{
		if (!svc->WritePagesToDisk())
			break;
		//usleep(100000);
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

bool M3MekaLog::LinkDependentComponents()
{
	
	for (int i = 0; i < comp_names.size(); i++)
	{
	   M3Component * a = (M3Component *)factory->GetComponent(comp_names[i]);
	   if (a != NULL)
	   {
	    components.push_back(a);
	    M3_INFO("M3Component %s linked for %s\n", comp_names[i].c_str(), GetName().c_str());
	   }
	   else
	     M3_WARN("M3Component comp %s missing for %s\n",comp_names[i].c_str(), GetName().c_str() );
	} 
	
	return true;
}


////////////////////////////////////////////////////////////

void M3MekaLog::Startup()
{  
	page_size = int(2*freq/5);	
	
	command.set_enable(enable);
	
	  if (GetEnvironmentVar(M3_ROBOT_ENV_VAR, path))
	  {		
		  time_t rawtime;
		  struct tm *timeinfo;
		  char buf[80];
			
		  path.append("/robot_log/");		
		  
		  log_name.append("_");
		  
		  time(&rawtime);
		  timeinfo = localtime(&rawtime);
		  strftime(buf,80,"%m_%d_%H_%M_%S",timeinfo);
		  string s = string(buf);
		  
		  log_name.append(s);
		  
		  path.append(log_name);
		  
		  status.set_path(path);
		  status.set_log_name(log_name);
		  		  		  
		  string t = "mkdir -p ";
		  t.append(path);
		  int r = system(t.c_str());
		  M3_DEBUG("%s \n",t.c_str());
		  M3_DEBUG("res = %d\n",r);
		  
	  }	
	  downsample_rate = MAX(0,((int)((mReal)RT_TASK_FREQUENCY)/freq)-1); 
	  downsample_cnt = 0;
	  
	  //int idx = factory->GetComponentIdx(GetName());
	  //components.push_back(factory->GetComponent(idx));
	  
	  //NOTE: This serialization needs to be done once only
	  //Likely a semaphore is needed on rt_system
	  //Otherwise can gen a protobuf error. Moved this out of the loop below.
	  vector<string> datums;
	  vector<M3Component*>::iterator k;
	  
	  /*string d;
	  SerializeStatus(d);
	  datums.push_back(d);*/
	  
	  for(k=components.begin(); k!=components.end(); ++k)
	  {
		  string d;
		  (*k)->SerializeStatus(d);
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
		  return;
		  //return false;
	  }
	
	//return log_thread_active;
	return;
  
}

void M3MekaLog::Shutdown()
{
      if (enable)
      {
	if (verbose)
	{
	  M3_INFO("M3RtLogService %s: Pages Written: %d\n",log_name.c_str(),num_page_write);
	  M3_INFO("M3RtLogService %s: KByte Written: %d\n",log_name.c_str(),num_kbyte_write);
	  M3_INFO("M3RtLogService %s. Shutting down...\n",log_name.c_str());
	}
	
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
}


bool M3MekaLog::ReadConfig(const char * filename)
{
  YAML::Node doc;

  if (!M3Component::ReadConfig(filename))
	  return false;
  GetYamlDoc(filename, doc);

  try 
	{
	  string tmp;
	  for(int i=0; i < doc["components"].size(); i++) 
	  {
	      doc["components"][i] >> tmp;
	       comp_names.push_back(tmp);
	  }	   
	} catch(YAML::TypedKeyNotFound<string> e) 
	{
		
	} 
  
   doc["log_name"] >> log_name;
   doc["freq"] >> freq;
   doc["verbose"] >> verbose;
  int tmp;
   doc["enable"] >> tmp;
   enable = (bool)tmp;
   
   return true;
  
}

void M3MekaLog::StepStatus()
{
    if (IsStateError())
	return;
    
	
}

void M3MekaLog::StepCommand()
{
  if (IsStateSafeOp())
      return;
  
  enable = (bool)command.enable();
  
  if (enable)
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
				return;
				//return false;
			entry->set_datum(k,*datum);
			entry->set_name(k,(*j)->GetName());
			k++;
		}
		
		/*datum = entry->mutable_datum(k);
		SerializeStatus(*datum);
		entry->set_datum(k,*datum);
		entry->set_name(k,GetName());*/
		
		entry_idx++;
		//return WriteEntry(false);
		WriteEntry(false);
		return;
	}
	downsample_cnt--;	
    }
    return;
}

bool  M3MekaLog::WriteEntry(bool final)
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

void M3MekaLog::MarkPageFull()
{
  if (is_page_full[page_idx_write])
    M3_ERR("Buffer over run in log pages...\n");
  
  is_page_full[page_idx_write] = true;
}

void M3MekaLog::MarkPageEmpty()
{
  is_page_full[page_idx_read] = false;
}

M3StatusLogPage * M3MekaLog::GetNextPageToWrite()
{  
  page_idx_write++;
  if (page_idx_write >= MAX_PAGE_QUEUE)
  {
    page_idx_write = 0;
  }
  
  return pages[page_idx_write];    
}

M3StatusLogPage * M3MekaLog::GetNextPageToRead()
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

bool M3MekaLog::WritePagesToDisk()
{
	if (page==NULL)
		return false;
	
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


string  M3MekaLog::GetNextFilename(int num_entry)
{
	int i,nl;
	string filename=path+"/"+log_name+"_";
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



}