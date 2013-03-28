#! /usr/bin/python
import time
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.component_factory as mcf
import yaml
import os.path


def do_log(proxy):
       print 'Starting log...select component to log'
       proxy.start(start_data_svc=False)
       comp_name=m3t.user_select_components_interactive(proxy.get_available_components())[0]
       comp=mcf.create_component(comp_name)
       proxy.register_log_component(comp)
       print 'Enter log name [foo]'       
       try_again = True
       while try_again:
              logname=m3t.get_string('foo')
              if not os.path.isdir(m3t.get_m3_log_path()+logname):
                     try_again = False
              else:
                     print 'Log directory already exists.  Please enter another name.'
       print 'Enter sample frequence (0-X Hz) [250.0]'
       freq=m3t.get_float(250.0)
       default_samps = int(2*freq/5)
       print 'Enter samples per file ['+str(default_samps) +']'
       samples=m3t.get_int(default_samps)
       print 'Enter sample time (s) [5.0]'
       duration=m3t.get_float(5.0)
       print 'Hit enter to begin log...'
       raw_input()
       proxy.start_log_service(logname,sample_freq_hz=freq,samples_per_file=samples)
       ts=time.time()
       while time.time()-ts<duration:
              time.sleep(0.25)
              print 'Logging ',logname,' Time: ',time.time()-ts
       proxy.stop_log_service()



def do_scope(proxy):
       print 'Starting scope...'
       print 'Enter log name [foo]'
       logname=m3t.get_string('foo')
       ns=proxy.get_log_num_samples(logname)
       print 'Num samples available: ',ns
       if ns==0:
              return
       print '-------- Available components --------'
       names=proxy.get_log_component_names(logname)
       for i in range(len(names)):
              print names[i]
       print '--------------------------------------'
       print 'Select component: [',names[0],']'
       name=m3t.get_string(names[0])
       comp=mcf.create_component(name)
       proxy.register_log_component(comp)
       scope=m3t.M3Scope(xwidth=ns-1)
       print '--------------------------------------'
       print 'Num samples available: ',ns
       print 'Enter start sample idx: [0]'
       start=max(0,m3t.get_int(0))
       print 'Enter end sample idx: [',ns-1,']'
       end=min(ns-1,m3t.get_int(ns-1))
       print '--------------------------------------'
       print 'Select field to monitor'
       
       field=m3t.user_select_msg_field(comp.status)
       
       repeated = False
       idx = 0
       if hasattr(m3t.get_msg_field_value(comp.status,field),'__len__'):
              repeated = True
              print 'Select index of repeated field to monitor: [0]'
              idx = m3t.get_int(0)              
       
       for i in range(start,end):
              proxy.load_log_sample(logname,i)
              if repeated:
                     v=m3t.get_msg_field_value(comp.status,field)[idx]
              else:
                     v=m3t.get_msg_field_value(comp.status,field)
              scope.plot(v)
              time.sleep(0.05)

def do_plot(proxy):
       print 'Starting plot...'
       print 'Enter log name [foo]'
       logname=m3t.get_string('foo')
       ns=proxy.get_log_num_samples(logname)
       if ns==0:
              return
       print '-------- Available components --------'
       names=proxy.get_log_component_names(logname)
       for i in range(len(names)):
              print names[i]
       print '--------------------------------------'
       print 'Select component: [',names[0],']'
       name=m3t.get_string(names[0])
       print '--------------------------------------'
       print 'Num samples available: ',ns
       print 'Enter start sample idx: [0]'
       start=max(0,m3t.get_int(0))
       print 'Enter end sample idx: [',ns-1,']'
       end=min(ns-1,m3t.get_int(ns-1))
       print '--------------------------------------'
       print ' Select field to plot...'
       comp=mcf.create_component(name)
       proxy.register_log_component(comp)
       field=m3t.user_select_msg_field(comp.status)
              
       repeated = False
       idx = 0
       if hasattr(m3t.get_msg_field_value(comp.status,field),'__len__'):
              repeated = True
              print 'Select index of repeated field to monitor: [0]'
              idx = m3t.get_int(0)              
              
       print 'Fetching data...'
       data=[]

       for i in range(start,end):
              proxy.load_log_sample(logname,i)
              if repeated:
                     v=m3t.get_msg_field_value(comp.status,field)[idx]
              else:
                     v=m3t.get_msg_field_value(comp.status,field)       
              data.append(v)
              #       m3t.plot(data)
       m3t.mplot(data,range(len(data)))

def do_dump(proxy):
       print 'Starting dump...'
       print 'Enter log name [foo]'
       logname=m3t.get_string('foo')
       ns=proxy.get_log_num_samples(logname)
       if ns==0:
              return
       print '-------- Available components --------'
       names=proxy.get_log_component_names(logname)
       for i in range(len(names)):
              print names[i]
       print '--------------------------------------'
       print 'Select component: [',names[0],']'
       name=m3t.get_string(names[0])
       print '--------------------------------------'
       print 'Num samples available: ',ns
       print 'Enter start sample idx: [0]'
       start=max(0,m3t.get_int(0))
       print 'Enter end sample idx: [',ns-1,']'
       end=min(ns-1,m3t.get_int(ns-1))
       print '--------------------------------------'
       comp=mcf.create_component(name)
       proxy.register_log_component(comp)
       fields=[]
       print 'Dump all data?[n]'       
       if m3t.get_yes_no('n'):
              '''print 'Fetching data...'
              fn=m3t.get_m3_log_path()+logname+'/'+logname+'_dump.yaml'
              f=file(fn,'w')
              print 'Saving...',fn
              f.write(yaml.safe_dump(comp.status, default_flow_style=False,width=200))
              f.close()'''
              fields=comp.status.DESCRIPTOR.fields_by_name.keys()
              print fields
       else:
              print ' Select fields to plot...'
              while True:
                     fields.append(m3t.user_select_msg_field(comp.status))
                     print 'Another field [n]?'
                     if not m3t.get_yes_no('n'):
                            break
       print 'Fetching data...'
       data={}
       for k in fields:
              data[k]=[]
              print '---------------------------'
              print k
              print m3t.get_msg_field_value(comp.status,k)
              print dir(m3t.get_msg_field_value(comp.status,k))
              
              if hasattr(m3t.get_msg_field_value(comp.status,k),'__len__'):
                     for j in range(len(m3t.get_msg_field_value(comp.status,k))):
                            data[k].append([])
                     
                     
       for i in range(start,end):
              proxy.load_log_sample(logname,i)
              for k in fields:
                     repeated = False
                     # skip protobuf subclasses (like base) for now.  note we'll want this for humanoid
                     if hasattr(m3t.get_msg_field_value(comp.status,k),'__metaclass__'):
                            pass
                     elif hasattr(m3t.get_msg_field_value(comp.status,k),'__len__'):       
                            for j in range(len(m3t.get_msg_field_value(comp.status,k))):
                                   data[k][j].append(m3t.get_msg_field_value(comp.status,k)[j])
                     else:
                            data[k].append(m3t.get_msg_field_value(comp.status,k))
       fn=m3t.get_m3_log_path()+logname+'/'+logname+'_dump.yaml'
       f=file(fn,'w')
       print 'Saving...',fn
       print data
       f.write(yaml.safe_dump(data, default_flow_style=False,width=200))
       f.close()

def print_usage():
       print '------- M3 RT Logger -------'
       print 'l: log'
       print 's: scope'
       print 'p: plot'
       print 'd: dump to yaml'
       print 'q: quit'

if __name__ == '__main__':
       proxy = m3p.M3RtProxy()
       try:
              while True:
                     print_usage()
                     cmd=raw_input()
                     if cmd=='q':
                            break
                     if cmd=='l':
                            do_log(proxy)
                     if cmd=='s':
                            do_scope(proxy)    
                     if cmd=='p':
                            do_plot(proxy)   
                     if cmd=='d':
                            do_dump(proxy)
       except (KeyboardInterrupt,EOFError):
              pass
       #proxy.stop() 
       #proxy.stop(force_safeop=False) #allow other clients to continue running
