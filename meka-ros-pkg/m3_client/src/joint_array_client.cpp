#include "joint_array_client.h"
  
using namespace ros;
using namespace m3_client;
using namespace std;


void M3JointArrayClient::ResizeCmd(int ndof)
{
  
  cmd.request.tq_desired.resize(ndof);
  cmd.request.ctrl_mode.resize(ndof);
  cmd.request.q_stiffness.resize(ndof);
  cmd.request.q_desired.resize(ndof);
  cmd.request.qdot_desired.resize(ndof);
  cmd.request.q_slew_rate.resize(ndof);
  cmd.request.pwm_desired.resize(ndof);
}

void M3JointArrayClient::GetYamlDoc(string filename, YAML::Node & doc)
{		
	string path;
	
	if (GetEnvironmentVar("M3_ROBOT", path))
	{		
		path.append("/robot_config/");
		path.append(filename);
	}
	ifstream fin(path.c_str());
	if (fin.fail())
	{		
		ROS_ERROR("could not read %s \n", path.c_str());	
	}

   	YAML::Parser parser(fin);
   	
   	parser.GetNextDocument(doc);
	fin.close();
	return;
}

bool M3JointArrayClient::GetEnvironmentVar(const char * var, string &s)
{
	char *p=getenv(var);
	if (p!=NULL)
	{
		s.assign(p);
		return true;
	}
	return false;
}

void M3JointArrayClient::SetSlewRateProportional(unsigned int idx, double slew_rate)
{ 
   if (idx < cmd.request.q_slew_rate.size())
     cmd.request.q_slew_rate[idx] = CLAMP(slew_rate, 0.0, 1.0)*max_slew[idx];
}

string M3JointArrayClient::GetCompDir(string component)
{
    YAML::Node doc;
    GetYamlDoc("m3_config.yml",doc);
    
    if(!doc.FindValue("rt_components")) 
    {
	    ROS_ERROR("No rt_components key in m3_config.yml.");
	    return "";
    }
    
    for(YAML::Iterator it=doc["rt_components"].begin();it!=doc["rt_components"].end();++it) 
    {
	    string dir;

	    it.first() >> dir;
	    
	    for(YAML::Iterator it_dir=doc["rt_components"][dir.c_str()].begin();
		    it_dir!=doc["rt_components"][dir.c_str()].end();++it_dir) 
	    {
		    string  name, type;
		    it_dir.first() >> name;
		    it_dir.second() >> type;
		    if (name == component)		    		    		      		    
		      return dir;
	    }
	    
    }
    ROS_ERROR("No Robot Found.");
    return "";
}


void M3JointArrayClient::SetThetaDeg(unsigned int idx, double theta)
{
   if (idx < cmd.request.q_desired.size())
     cmd.request.q_desired[idx] = theta;
}

double M3JointArrayClient::GetThetaDeg(unsigned int  idx)
{
   if (idx < status.response.theta.size())
     return status.response.theta[idx];
   else
     return 0;
}

double M3JointArrayClient::GetMotorTemp(unsigned int  idx)
{
   if (idx < status.response.motor_temp.size())
     return status.response.motor_temp[idx];
   else
     return 0;
}

double M3JointArrayClient::GetAmpTemp(unsigned int  idx)
{
   if (idx < status.response.amp_temp.size())
     return status.response.amp_temp[idx];
   else
     return 0;
}

double M3JointArrayClient::GetThetaDotDotDeg(unsigned int  idx)
{
   if (idx < status.response.thetadotdot.size())
     return status.response.thetadotdot[idx];
   else
     return 0;
}

double M3JointArrayClient::GetThetaDotDeg(unsigned int  idx)
{
   if (idx < status.response.thetadot.size())
     return status.response.thetadot[idx];
   else
     return 0;
}

double M3JointArrayClient::GetTorqueDot_mNm(unsigned int  idx)
{
   if (idx < status.response.torquedot.size())
     return status.response.torquedot[idx];
   else
     return 0;
}

double M3JointArrayClient::GetTorque_mNm(unsigned int  idx)
{
   if (idx < status.response.torque.size())
     return status.response.torque[idx];
   else
     return 0;
}


double M3JointArrayClient::GetCurrent(unsigned int  idx)
{
   if (idx < status.response.current.size())
     return status.response.current[idx];
   else
     return 0;
}

double M3JointArrayClient::GetPwm(unsigned int  idx)
{
   if (idx < status.response.pwm_cmd.size())
     return status.response.pwm_cmd[idx];
   else
     return 0;
}

void M3JointArrayClient::SetPwm(unsigned int idx, double pwm)
{
   if (idx < cmd.request.pwm_desired.size())
     cmd.request.pwm_desired[idx] = pwm;
}

void M3JointArrayClient::SetTorque_mNm(unsigned int idx, double torque)
{
   if (idx < cmd.request.tq_desired.size())
     cmd.request.tq_desired[idx] = torque;
}

void M3JointArrayClient::SetThetaDotDeg(unsigned int idx, double theta_dot)
{
   if (idx < cmd.request.qdot_desired.size())
     cmd.request.qdot_desired[idx] = theta_dot;
}

void M3JointArrayClient::SetSlewRate(unsigned int idx, double slew_rate)
{
   if (idx < cmd.request.q_slew_rate.size())
     cmd.request.q_slew_rate[idx] = slew_rate;
}

void M3JointArrayClient::SetModePwm(unsigned int  idx)
{
   if (idx < cmd.request.ctrl_mode.size())
     cmd.request.ctrl_mode[idx] = (int)JOINT_ARRAY_MODE_PWM;
}

void M3JointArrayClient::SetModeTorque(unsigned int  idx)
{
   if (idx < cmd.request.ctrl_mode.size())
     cmd.request.ctrl_mode[idx] = (int)JOINT_ARRAY_MODE_TORQUE;
}

void M3JointArrayClient::SetModeThetaGcMj(unsigned int  idx)
{
   if (idx < cmd.request.ctrl_mode.size())
     cmd.request.ctrl_mode[idx] = (int)JOINT_ARRAY_MODE_THETA_GC_MJ;
}

void M3JointArrayClient::SetModeThetaMj(unsigned int  idx)
{
   if (idx < cmd.request.ctrl_mode.size())
     cmd.request.ctrl_mode[idx] = (int)JOINT_ARRAY_MODE_THETA_MJ;
}

void M3JointArrayClient::SetModeThetaGc(unsigned int  idx)
{
   if (idx < cmd.request.ctrl_mode.size())
     cmd.request.ctrl_mode[idx] = (int)JOINT_ARRAY_MODE_THETA_GC;
}

void M3JointArrayClient::SetModeTheta(unsigned int  idx)
{
   if (idx < cmd.request.ctrl_mode.size())
     cmd.request.ctrl_mode[idx] = (int)JOINT_ARRAY_MODE_THETA;
}

void M3JointArrayClient::SetModeOff(unsigned int  idx)
{
   if (idx < cmd.request.ctrl_mode.size())
     cmd.request.ctrl_mode[idx] = (int)JOINT_ARRAY_MODE_OFF;
}

void M3JointArrayClient::SetStiffness(unsigned int  idx, double stiffness)
{
   if (idx < cmd.request.q_stiffness.size())
     cmd.request.q_stiffness[idx] = stiffness;
}



