#include "ros/ros.h"
#include "m3_client/M3JointArrayCmd.h"
#include "m3_client/M3JointArrayParam.h"
#include "m3_client/M3JointArrayStatus.h"
#include "m3/chains/joint_array_mode.pb.h"
#include <cstdlib>
#include "yaml-cpp/yaml.h"
#include <fstream>

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

using namespace ros;
using namespace m3_client;
using namespace std;

class M3JointArrayClient{
	public:
		M3JointArrayClient(string joint_array_name, NodeHandle * node_handle)
		{		  
		  status_client = node_handle->serviceClient<M3JointArrayStatus>(joint_array_name+"_status");
		  param_client = node_handle->serviceClient<M3JointArrayParam>(joint_array_name+"_param");
		  cmd_client = node_handle->serviceClient<M3JointArrayCmd>(joint_array_name+"_cmd");
		  
		  if(!status_client.call(status))  		
		    ROS_ERROR(("Failed to connect to "+joint_array_name+"_status service.").c_str());		 
		  
		  ndof = status.response.theta.size();
		  ResizeCmd(ndof);
		  	 	
		  max_slew.resize(ndof);
		  
		  YAML::Node doc_arm;		
		  GetYamlDoc(GetCompDir(joint_array_name) + "/" + joint_array_name + ".yml", doc_arm);		  		  
		  
		  for(YAML::Iterator it=doc_arm["joint_components"].begin();it!=doc_arm["joint_components"].end();++it) 
		  {
			  string joint, j;
			  it.first() >> j;
			  it.second() >> joint;
				  
			  YAML::Node doc_joint;		
			  GetYamlDoc(GetCompDir(joint) + "/" + joint + ".yml", doc_joint);
			  int id=atoi(j.substr(1).c_str()); //"J0" gives 0, etc
			  max_slew[id] = doc_joint["param"]["max_q_slew_rate"];	  
		  }
				  
		  for (int i = 0; i < ndof; i++)
		    SetModeOff(i); 
		}
		void SetPwm(unsigned int  idx, double pwm);
		void SetTorque_mNm(unsigned int  idx, double torque);
		void SetThetaDotDeg(unsigned int  idx, double theta_dot);
		double GetCurrent(unsigned int idx);
		double GetTorque_mNm(unsigned int  idx);
		double GetTorqueDot_mNm(unsigned int  idx);
		double GetThetaDotDeg(unsigned int  idx); 
		double GetThetaDotDotDeg(unsigned int  idx); 
		double GetAmpTemp(unsigned int idx);
		double GetMotorTemp(unsigned int idx);
		double GetPwm(unsigned int  idx);
		void SetThetaDeg(unsigned int  idx, double theta);
		double GetThetaDeg(unsigned int  idx); 
		void SetSlewRate(unsigned int  idx, double slew_rate);  // degs/sec
		void SetSlewRateProportional(unsigned int  idx, double slew_rate);  // 0.0 -> 1.0
		void SetModePwm(unsigned int  idx);
		void SetModeTorque(unsigned int  idx);
		void SetModeThetaGcMj(unsigned int  idx);
		void SetModeThetaMj(unsigned int  idx);
		void SetModeThetaGc(unsigned int  idx);
		void SetModeTheta(unsigned int  idx);
		void SetModeOff(unsigned int  idx);
		void SetStiffness(unsigned int  idx, double stiffness);  // 0.0 - 1.0
		void UpdateStatus(){status_client.call(status);}
		void SendCommand(){cmd_client.call(cmd);}
	private:
		string GetCompDir(string component);
		bool GetEnvironmentVar(const char * var, string &s);
		void GetYamlDoc(string filename, YAML::Node & doc);
		void ResizeCmd(int ndof);
		M3JointArrayCmd cmd;  
		M3JointArrayStatus status;
		M3JointArrayParam param;		
		ServiceClient cmd_client;
		ServiceClient param_client;
		ServiceClient status_client;
		int ndof;	
		vector<double> max_slew;
};
		