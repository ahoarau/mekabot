#include "humanoid_client.h"

using namespace ros;
using namespace m3_client;
using namespace std;

M3HumanoidClient::M3HumanoidClient(NodeHandle * node_handle)
{  
  cmd_client = node_handle->serviceClient<M3HumanoidCmd>("humanoid_cmd");  
  status_client = node_handle->serviceClient<M3HumanoidStatus>("humanoid_status");
  param_client = node_handle->serviceClient<M3HumanoidParam>("humanoid_param");
  
  status_right_arm.request.chain = (unsigned char)RIGHT_ARM;
  status_left_arm.request.chain = (unsigned char)LEFT_ARM;      
  status_torso.request.chain = (unsigned char)TORSO;      
  status_head.request.chain = (unsigned char)HEAD;     
  
  cmd_right_arm.request.chain = (unsigned char)RIGHT_ARM;
  cmd_left_arm.request.chain = (unsigned char)LEFT_ARM;      
  cmd_torso.request.chain = (unsigned char)TORSO;      
  cmd_head.request.chain = (unsigned char)HEAD;   
  
  param_right_arm.request.chain = (unsigned char)RIGHT_ARM;
  param_left_arm.request.chain = (unsigned char)LEFT_ARM;      
  param_torso.request.chain = (unsigned char)TORSO;      
  param_head.request.chain = (unsigned char)HEAD;   
  
  if(!status_client.call(status_right_arm))  
  {
    ROS_ERROR("Failed to connect to right arm humanoid_status service.");      
  }
  if(!status_client.call(status_left_arm))  
  {
    ROS_ERROR("Failed to connect to left arm humanoid_status service.");      
  }
  if(!status_client.call(status_torso))  
  {
    ROS_ERROR("Failed to connect to torso humanoid_status service.");      
  }
  if(!status_client.call(status_head))  
  {
    ROS_ERROR("Failed to connect to head humanoid_status service.");      
  }
  
  ndof_right_arm = status_right_arm.response.theta.size();  
  ndof_left_arm = status_left_arm.response.theta.size();
  ndof_torso = status_torso.response.theta.size();
  ndof_head = status_head.response.theta.size();

  ResizeCmd(RIGHT_ARM, ndof_right_arm);
  ResizeCmd(LEFT_ARM, ndof_left_arm);
  ResizeCmd(TORSO, ndof_torso);
  ResizeCmd(HEAD, ndof_head);
  
  max_slew_right_arm.resize(ndof_right_arm);
  max_slew_left_arm.resize(ndof_left_arm);
  max_slew_torso.resize(ndof_torso);
  max_slew_head.resize(ndof_head);
  
  ReadConfig();
  
  for (int i = 0; i < ndof_right_arm; i++)
    SetModeOff(RIGHT_ARM,i);        
  for (int i = 0; i < ndof_left_arm; i++)
    SetModeOff(LEFT_ARM,i);      
  for (int i = 0; i < ndof_torso; i++)
    SetModeOff(TORSO,i);    
  for (int i = 0; i < ndof_head; i++)
    SetModeOff(HEAD,i);    
  SetMotorPowerOff();
}

string M3HumanoidClient::GetRobotName()
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
		    if (type == "m3humanoid")		    
		      return name;
	    }
	    
    }
    ROS_ERROR("No Robot Found.");
    return "";
}

string M3HumanoidClient::GetCompDir(string component)
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

bool M3HumanoidClient::GetEnvironmentVar(const char * var, string &s)
{
	char *p=getenv(var);
	if (p!=NULL)
	{
		s.assign(p);
		return true;
	}
	return false;
}

void M3HumanoidClient::GetYamlDoc(string filename, YAML::Node & doc)
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

bool M3HumanoidClient::ReadConfig()
{	
	// Get Paramters for Payloads
	string name = GetRobotName();	
	string filename = GetCompDir(name) + "/" + name + ".yml";
		
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	
	const YAML::Node * ra_node;
	const YAML::Node * la_node;
	const YAML::Node * t_node;
	const YAML::Node * h_node;
		
	try {
		ra_node = &(doc["chains"]["right_arm"]);
	} catch(YAML::KeyNotFound& e) {
		ra_node = NULL;
	}
	
	try {
		la_node = &(doc["chains"]["left_arm"]);
	} catch(YAML::KeyNotFound& e) {
		la_node = NULL;
	}
	
	try {
		t_node = &(doc["chains"]["torso"]);
	} catch(YAML::KeyNotFound& e) {
		t_node = NULL;
	}
	
	try {
		h_node = &(doc["chains"]["head"]);
	} catch(YAML::KeyNotFound& e) {		
		h_node = NULL;
	}
	 					
	if (ra_node) 
	{
	  LoadParam(ra_node, &param_right_arm);	
	  LoadMaxSlew(ra_node, &max_slew_right_arm);
	}
	if (la_node) 
	{
	  LoadParam(la_node, &param_left_arm);	
	  LoadMaxSlew(la_node, &max_slew_left_arm);  
	}
	if (t_node) 
	{
	  LoadParam(t_node, &param_torso);	
	  LoadMaxSlew(t_node, &max_slew_torso);  
	}
	if (h_node)		
	{
	  LoadParam(h_node, &param_head);	
	  LoadMaxSlew(h_node, &max_slew_head);  
	}
			
	return true;
}

void M3HumanoidClient::LoadMaxSlew(const YAML::Node * node, vector<double> * max_slew)
{
  string arm_name; 	
  (*node)["chain_component"] >> arm_name;		
  YAML::Node doc_arm;		
  GetYamlDoc(GetCompDir(arm_name) + "/" + arm_name + ".yml", doc_arm);
  
   for(YAML::Iterator it=doc_arm["joint_components"].begin();it!=doc_arm["joint_components"].end();++it) 
    {
	    string joint, j;
	    it.first() >> j;
	    it.second() >> joint;
	    	    
	    YAML::Node doc_joint;		
	    GetYamlDoc(GetCompDir(joint) + "/" + joint + ".yml", doc_joint);
	    int id=atoi(j.substr(1).c_str()); //"J0" gives 0, etc
	    (*max_slew)[id] = doc_joint["param"]["max_q_slew_rate"];	  
    }
}

void M3HumanoidClient::LoadParam(const YAML::Node * node, M3HumanoidParam * param)
{
  string arm_name, dyn_name; 	
  (*node)["chain_component"] >> arm_name;		
  YAML::Node doc_arm;		
  GetYamlDoc(GetCompDir(arm_name) + "/" + arm_name + ".yml", doc_arm);
  doc_arm["dynamatics_component"] >> dyn_name;		
  YAML::Node doc_dyn;		
  GetYamlDoc(GetCompDir(dyn_name) + "/" + dyn_name + ".yml", doc_dyn);
  const YAML::Node& ymlparam = doc_dyn["param"];
  double rtemp;
  ymlparam["payload_mass"] >> rtemp;
  param->request.payload_mass = rtemp;	
  for(int i=0; i<ymlparam["payload_com"].size(); i++)
  {
      ymlparam["payload_com"][i] >> rtemp;
      param->request.payload_com[i] = rtemp;						
  }

  for(int i=0; i<ymlparam["payload_inertia"].size(); i++)
  {
      ymlparam["payload_inertia"][i] >> rtemp;
      param->request.payload_inertia[i] = rtemp;			
  }						
}

void M3HumanoidClient::ResizeCmd(M3Chain chain, int ndof)
{
  M3HumanoidCmd * cmd = GetCmd(chain);  
  
  if(cmd == NULL)
    return;
  
  cmd->request.tq_desired.resize(ndof);
  cmd->request.ctrl_mode.resize(ndof);
  cmd->request.q_stiffness.resize(ndof);
  cmd->request.q_desired.resize(ndof);
  cmd->request.qdot_desired.resize(ndof);
  cmd->request.q_slew_rate.resize(ndof);
  cmd->request.pwm_desired.resize(ndof);
}

M3HumanoidCmd * M3HumanoidClient::GetCmd(M3Chain chain)
{
  switch (chain)
  {
    case RIGHT_ARM:
      return &cmd_right_arm;
    case LEFT_ARM:
      return &cmd_left_arm;
    case TORSO:
      return &cmd_torso;
    case HEAD:
      return &cmd_head;      
  }
  return NULL;
}

M3HumanoidStatus * M3HumanoidClient::GetStatus(M3Chain chain)
{
  switch (chain)
  {
    case RIGHT_ARM:
      return &status_right_arm;
    case LEFT_ARM:
      return &status_left_arm;
    case TORSO:
      return &status_torso;
    case HEAD:
      return &status_head;      
  }
  return NULL;

}

M3HumanoidParam * M3HumanoidClient::GetParam(M3Chain chain)
{
  switch (chain)
  {
    case RIGHT_ARM:
      return &param_right_arm;
    case LEFT_ARM:
      return &param_left_arm;
    case TORSO:
      return &param_torso;
    case HEAD:
      return &param_head;      
  }
  return NULL;

}

void M3HumanoidClient::SetThetaDeg(M3Chain chain,unsigned int idx, double theta)
{
   M3HumanoidCmd * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->request.q_desired.size())
     cmd->request.q_desired[idx] = theta;
}

double M3HumanoidClient::GetThetaDeg(M3Chain chain,unsigned int  idx)
{
  M3HumanoidStatus * status = GetStatus(chain);
   if (status == NULL)
     return 0;
   if (idx < status->response.theta.size())
     return status->response.theta[idx];
   else
     return 0;
}

double M3HumanoidClient::GetThetaDotDeg(M3Chain chain,unsigned int  idx)
{
  M3HumanoidStatus * status = GetStatus(chain);
   if (status == NULL)
     return 0;
   if (idx < status->response.thetadot.size())
     return status->response.thetadot[idx];
   else
     return 0;
}

double M3HumanoidClient::GetThetaDotDotDeg(M3Chain chain,unsigned int  idx)
{
  M3HumanoidStatus * status = GetStatus(chain);
   if (status == NULL)
     return 0;
   if (idx < status->response.thetadotdot.size())
     return status->response.thetadotdot[idx];
   else
     return 0;
}

double M3HumanoidClient::GetTorque_mNm(M3Chain chain,unsigned int  idx)
{
  M3HumanoidStatus * status = GetStatus(chain);
   if (status == NULL)
     return 0;
   if (idx < status->response.torque.size())
     return status->response.torque[idx];
   else
     return 0;
}


double M3HumanoidClient::GetTorqueDot_mNm(M3Chain chain,unsigned int  idx)
{
  M3HumanoidStatus * status = GetStatus(chain);
   if (status == NULL)
     return 0;
   if (idx < status->response.torquedot.size())
     return status->response.torquedot[idx];
   else
     return 0;
}

double M3HumanoidClient::GetGravity(M3Chain chain,unsigned int  idx)
{
  M3HumanoidStatus * status = GetStatus(chain);
   if (status == NULL)
     return 0;
   if (idx < status->response.G.size())
     return status->response.G[idx];
   else
     return 0;
}

double M3HumanoidClient::GetPwm(M3Chain chain,unsigned int  idx)
{
  M3HumanoidStatus * status = GetStatus(chain);
   if (status == NULL)
     return 0;
   if (idx < status->response.pwm_cmd.size())
     return status->response.pwm_cmd[idx];
   else
     return 0;
}

double M3HumanoidClient::GetPayloadMass(M3Chain chain)
{
  M3HumanoidParam * param = GetParam(chain);
   if (param == NULL)
     return 0;
   
   return param->request.payload_mass;   
}

double M3HumanoidClient::GetPayloadInertia(M3Chain chain,unsigned int  idx)
{
  M3HumanoidParam * param = GetParam(chain);
   if (param == NULL)
     return 0;
   if (idx < param->request.payload_inertia.size())
     return param->request.payload_inertia[idx];
   else
     return 0;
}

double M3HumanoidClient::GetPayloadCom(M3Chain chain,unsigned int  idx)
{
  M3HumanoidParam * param = GetParam(chain);
   if (param == NULL)
     return 0;
   if (idx < param->request.payload_com.size())
     return param->request.payload_com[idx];
   else
     return 0;
}

void M3HumanoidClient::SetTorque_mNm(M3Chain chain,unsigned int idx, double torque)
{
  M3HumanoidCmd * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->request.pwm_desired.size())
     cmd->request.tq_desired[idx] = torque;
}

void M3HumanoidClient::SetThetaDotDeg(M3Chain chain,unsigned int idx, double theta_dot)
{
  M3HumanoidCmd * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->request.pwm_desired.size())
     cmd->request.qdot_desired[idx] = theta_dot;
}

void M3HumanoidClient::SetPwm(M3Chain chain,unsigned int idx, double pwm)
{
  M3HumanoidCmd * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->request.pwm_desired.size())
     cmd->request.pwm_desired[idx] = pwm;
}

void M3HumanoidClient::SetSlewRate(M3Chain chain,unsigned int idx, double slew_rate)
{
  M3HumanoidCmd * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->request.q_slew_rate.size())
     cmd->request.q_slew_rate[idx] = slew_rate;
}


void M3HumanoidClient::SetSlewRateProportional(M3Chain chain,unsigned int idx, double slew_rate)
{
  double max_slew;
  switch(chain)
  {
    case RIGHT_ARM:
      max_slew = max_slew_right_arm[idx];
      break;
    case LEFT_ARM:
      max_slew = max_slew_left_arm[idx];
      break;
    case TORSO:
      max_slew = max_slew_torso[idx];
      break;
    case HEAD:
      max_slew = max_slew_head[idx];
      break;            
  }
  
  M3HumanoidCmd * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->request.q_slew_rate.size())
     cmd->request.q_slew_rate[idx] = CLAMP(slew_rate, 0.0, 1.0)*max_slew;
}

void M3HumanoidClient::SetModePwm(M3Chain chain,unsigned int  idx)
{
  M3HumanoidCmd * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->request.ctrl_mode.size())
     cmd->request.ctrl_mode[idx] = (int)JOINT_ARRAY_MODE_PWM;
}

void M3HumanoidClient::SetModeTorque(M3Chain chain,unsigned int  idx)
{
  M3HumanoidCmd * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->request.ctrl_mode.size())
     cmd->request.ctrl_mode[idx] = (int)JOINT_ARRAY_MODE_THETA;
}

void M3HumanoidClient::SetModeThetaGcMj(M3Chain chain,unsigned int  idx)
{
  M3HumanoidCmd * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->request.ctrl_mode.size())
     cmd->request.ctrl_mode[idx] = (int)JOINT_ARRAY_MODE_THETA_GC;
}

void M3HumanoidClient::SetModeThetaMj(M3Chain chain,unsigned int  idx)
{
  M3HumanoidCmd * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->request.ctrl_mode.size())
     cmd->request.ctrl_mode[idx] = (int)JOINT_ARRAY_MODE_THETA_MJ;
}

void M3HumanoidClient::SetModeThetaGc(M3Chain chain,unsigned int  idx)
{
  M3HumanoidCmd * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->request.ctrl_mode.size())
     cmd->request.ctrl_mode[idx] = (int)JOINT_ARRAY_MODE_THETA_GC;
}

void M3HumanoidClient::SetModeTheta(M3Chain chain,unsigned int  idx)
{
  M3HumanoidCmd * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->request.ctrl_mode.size())
     cmd->request.ctrl_mode[idx] = (int)JOINT_ARRAY_MODE_THETA;
}

void M3HumanoidClient::SetModeOff(M3Chain chain,unsigned int  idx)
{
  M3HumanoidCmd * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->request.ctrl_mode.size())
     cmd->request.ctrl_mode[idx] = (int)JOINT_ARRAY_MODE_OFF;
}

void M3HumanoidClient::SetStiffness(M3Chain chain,unsigned int  idx, double stiffness)
{
   M3HumanoidCmd * cmd = GetCmd(chain);
   if (cmd == NULL)
     return;
   if (idx < cmd->request.q_stiffness.size())
     cmd->request.q_stiffness[idx] = stiffness;
}

void M3HumanoidClient::SetPayloadMass(M3Chain chain, double mass)
{
   M3HumanoidParam * param = GetParam(chain);
   if (param == NULL)
     return;
  param->request.payload_mass = mass;
}

void M3HumanoidClient::SetPayloadInertia(M3Chain chain,unsigned int  idx, double inertia)
{
   M3HumanoidParam * param = GetParam(chain);
   if (param == NULL)
     return;
  param->request.payload_inertia[idx] = inertia;
}

void M3HumanoidClient::SetPayloadCom(M3Chain chain,unsigned int  idx, double com)
{
   M3HumanoidParam * param = GetParam(chain);
   if (param == NULL)
     return;
  param->request.payload_com[idx] = com;
}


/*void M3HumanoidClient::EnableVelocityGains(M3Chain chain)
{
  M3HumanoidParam * param = GetParam(chain);
   if (param == NULL)
     return;
  param->request.use_velocities = true;
}

void M3HumanoidClient::DisableVelocityGains(M3Chain chain)
{
  M3HumanoidParam * param = GetParam(chain);
   if (param == NULL)
     return;
  param->request.use_velocities = false;
}

void M3HumanoidClient::EnableAccelerationGains(M3Chain chain)
{
  M3HumanoidParam * param = GetParam(chain);
   if (param == NULL)
     return;
  param->request.use_accelerations = false;  
}

void M3HumanoidClient::DisableAccelerationGains(M3Chain chain)
{
   M3HumanoidParam * param = GetParam(chain);
   if (param == NULL)
     return;
  param->request.use_accelerations = false;
}*/

void M3HumanoidClient::UpdateStatus()
{
      
  if (ndof_right_arm > 0)
    status_client.call(status_right_arm);
  if (ndof_left_arm > 0)
    status_client.call(status_left_arm);
  if (ndof_torso > 0)
    status_client.call(status_torso);
  if (ndof_head > 0)
    status_client.call(status_head);  
}

void M3HumanoidClient::SendCommand()
{
  if (ndof_right_arm > 0)
  {
     cmd_client.call(cmd_right_arm);
     status_client.call(status_right_arm);
  }
  if (ndof_left_arm > 0)
  {
    cmd_client.call(cmd_left_arm);
    status_client.call(status_left_arm);
  }
  if (ndof_torso > 0)
  {
    cmd_client.call(cmd_torso);
    status_client.call(status_torso);
  }
  if (ndof_head > 0)
  {
    cmd_client.call(cmd_head);
    status_client.call(cmd_head);
  }
}

void M3HumanoidClient::SetMotorPowerOn()
{
  cmd_right_arm.request.enable_motor = true;
  cmd_left_arm.request.enable_motor = true;
  cmd_torso.request.enable_motor = true;
  cmd_head.request.enable_motor = true;
}

void M3HumanoidClient::SetMotorPowerOff()
{
  cmd_right_arm.request.enable_motor = false;
  cmd_left_arm.request.enable_motor = false;
  cmd_torso.request.enable_motor = false;
  cmd_head.request.enable_motor = false;
}

int M3HumanoidClient::GetNdof(M3Chain chain)
{
  switch(chain)
  {
    case RIGHT_ARM:
      return ndof_right_arm;
    case LEFT_ARM:
      return ndof_left_arm;
    case TORSO:
      return ndof_torso;
    case HEAD:
      return ndof_head;      
  }
  return 0;
}

Eigen::Vector3d M3HumanoidClient::GetEndPosition(M3Chain chain)
{
  Eigen::Vector3d end_pos;
  M3HumanoidStatus * status = GetStatus(chain);
  for (int i = 0; i<3; i++)
    end_pos[i] = status->response.end_pos[i];
  
  return end_pos;
}

Eigen::Matrix3d M3HumanoidClient::GetEndRotation(M3Chain chain)
{
  Eigen::Matrix3d end_rot;
  M3HumanoidStatus * status = GetStatus(chain);
  for (int i = 0; i<3; i++)
  {
    for (int j = 0; j<3; j++)
      end_rot(i,j) = status->response.end_rot[(i*3)+(j)];
  }
  return end_rot;
}

Eigen::MatrixXd M3HumanoidClient::GetJacobian(M3Chain chain)
{
  M3HumanoidStatus * status = GetStatus(chain);
  Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(6,GetNdof(chain));
  
  for (int i = 0; i<6; i++)
  {
    for (int j = 0; j<GetNdof(chain); j++)
      jac(i,j) = status->response.J[(i*GetNdof(chain))+(j)];    
  }
  return jac;
}