#include "m3uta/controllers/example.h"
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_factory.h"
using namespace std;
using namespace KDL;

namespace m3uta{
	
using namespace m3rt;
using namespace std;
	
		
///////////////////////////////////////////////////////


void M3Example::Startup()
{
	if (bot==NULL)
		SetStateError();
	else
		SetStateSafeOp();

}

void M3Example::Shutdown()
{
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						  
bool M3Example::ReadConfig(const char * filename)
{
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	if (!M3Component::ReadConfig(filename))
		return false;
	doc["humanoid"] >> bot_name;
	double val;
	doc["param"]["max_fx"] >> val;
	param.set_max_fx(val);
	doc["param"]["max_fy"] >> val;
	param.set_max_fy(val);
	doc["param"]["max_fz"] >> val;
	param.set_max_fz(val);
	return true;
}

bool M3Example::LinkDependentComponents()
{
	//Need to find at least one arm
	bot=(M3Humanoid*) factory->GetComponent(bot_name);
	if (bot==NULL)
		M3_INFO("M3Humanoid component %s not found for component %s\n",bot_name.c_str(),GetName().c_str());
	if (bot==NULL)
		return false;
	return true;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3Example::StepStatus()
{
	status.set_foo(1.0);
}

void M3Example::StepCommand()
{
	int i;
	M3Arm * arm = bot->GetRightArm();
	tmp_cnt++;
	if (command.enable())
	{
	    
	    bot->SetMotorPowerOn();
	    Eigen::Matrix<double,6,1> wrench;
	    wrench[0]=CLAMP(command.fx(),-param.max_fx(),param.max_fx());
	    wrench[1]=CLAMP(command.fy(),-param.max_fy(),param.max_fy());
	    wrench[2]=CLAMP(command.fz(),-param.max_fz(),param.max_fz());
	    wrench[3]=0;
	    wrench[4]=0;
	    wrench[5]=0;
	    if (tmp_cnt%100==0)
	     M3_INFO("On: %f %f %f\n",command.fx(),command.fy(),command.fz());
	    Jacobian * J = bot->GetJacobian(M3Humanoid::RIGHT_ARM);
	    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> JT=J->data.transpose();
	    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> tq=JT*wrench;
	    
	      for (i=0;i<arm->GetNumDof();i++)
	      {
		  bot->SetCtrlModeTorqueGC(M3Humanoid::RIGHT_ARM,i);
		  bot->SetTorque(M3Humanoid::RIGHT_ARM,i,tq[i]);
		  if (tmp_cnt%100==0)
			M3_INFO("On: %d : %f\n",i,tq[i]);
	      }
	}
	else
	{
	  //if (tmp_cnt%100==0)
	   //   M3_INFO("Off\n");
	  bot->SetMotorPowerOff();
	  for (i=0;i<arm->GetNumDof();i++)
		  bot->SetCtrlModeTorqueOff(M3Humanoid::RIGHT_ARM,i);
		
	}
}

}