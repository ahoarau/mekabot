import m3.rt_proxy as m3p
proxy = m3p.M3RtProxy()
import m3.robot_monitor as rm
proxy.start()
r=rm.M3RobotMonitor('m3robot_monitor_rm0')
proxy.subscribe_status(r)
proxy.step()
proxy.step()
r.print_status()