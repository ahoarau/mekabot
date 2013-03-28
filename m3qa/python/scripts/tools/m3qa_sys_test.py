#! /usr/bin/python
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.pwr
import m3.component_factory as m3f
import m3qa.sys_test as mst
import time

print 'Robot should be in roughly neutral posture'
print 'Hit enter to begin'
raw_input()
st = mst.M3SysTest()
proxy = m3p.M3RtProxy()
proxy.start()
proxy.make_operational_all()

print 'Starting test...'
#Allow time for data to initialize
#for i in range(5):
#    proxy.step()
time.sleep(0.5)
err_cnt=0 
err_msg=[]
if st.config.has_key('not_val'):
    e,m=st.test_sensor_not_val(st.config['not_val'],proxy)
    err_cnt+=e
    err_msg+=m

for c in st.config_include['not_val']:
    e,m=st.test_sensor_not_val(c,proxy)
    err_cnt+=e
    err_msg+=m
    
if st.config.has_key('in_range'):
    e,m=st.test_sensor_in_range(st.config['in_range'],proxy)
    err_cnt+=e
    err_msg+=m

for c in st.config_include['in_range']:
    e,m=st.test_sensor_in_range(c,proxy)
    err_cnt+=e
    err_msg+=m
    
e,m=st.test_dropped_packets()
err_cnt+=e
err_msg+=m
e,m=st.test_slaves_present(st.config['num_slaves'])
err_cnt+=e
err_msg+=m
print
print 'Testing finished. Errors recorded: ',err_cnt
if err_cnt:
    print '----------------------------'
    for e in err_msg:
        print e
    print '----------------------------'
    print
st.write_log()

proxy.stop()