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

#include "m3/robot_ctrl/head_s2csp_ctrl.h"
#include "m3rt/base/component_factory.h"
#include "m3rt/base/toolbox.h"
#include "m3rt/base/m3rt_def.h"
#include "m3/robots/chain_name.h"
#include <math.h>

namespace m3
{
	
using namespace m3rt;
using namespace std;
using namespace KDL;


///////////////////////////////////////////////////////
bool M3HeadS2CSPCtrl::LinkDependentComponents()
{	
	bot=(M3Humanoid*)factory->GetComponent(bot_name);		
	if (bot==NULL)
	{
		M3_ERR("M3HeadS2CSPCtrl %s did not find component %s\n",GetName().c_str(),bot_name.c_str());	
		return false;
	}
	head=(M3Head*)factory->GetComponent(head_name);		
	if (head==NULL)
	{
		M3_ERR("M3HeadS2CSPCtrl %s did not find component %s\n",GetName().c_str(),bot_name.c_str());	
		return false;
	}
	return true;
}

void M3HeadS2CSPCtrl::Startup()
{		
	for (int i=0;i<3;i++)
	{

		command.add_target(0);
		status.add_xe(0);
		param.add_origin(0.0);
	}
	for (int i=0;i<7;i++)
	{
		status.add_theta_des(0);
		param.add_slew_des(slew_des[i]);
		param.add_theta_db(theta_db[i]);
		int window_us=0.5*1000000; //1/2 second
		int downsample = (window_us/RT_TASK_FREQUENCY)/100;
		joint_smooth[i].Resize(window_us,downsample);
	}
	command.set_enable(0);
	command.set_theta_des_j2(0);
	param.set_ratio_j0(1.0);
	SetStateSafeOp();
}


void M3HeadS2CSPCtrl::Shutdown()
{

}


void M3HeadS2CSPCtrl::StepCommand()
{	
	if (IsStateSafeOp() || IsStateError())
		return;
	
	if (!command.enable())
		return;
	
	mReal q0c=bot->GetThetaDeg(HEAD,0);
	mReal q1c=bot->GetThetaDeg(HEAD,1);
	mReal q3c=bot->GetThetaDeg(HEAD,3);
	mReal q4c=bot->GetThetaDeg(HEAD,4);
	mReal q5c=bot->GetThetaDeg(HEAD,5);
	
	theta_des[2]=command.theta_des_j2();
	
	xh=Vector(target_slew[0].Step(command.target(0),param.target_slew()),
		  target_slew[1].Step(command.target(1),param.target_slew()),
		  target_slew[2].Step(command.target(2),param.target_slew()));
	
	//Convert from head to eye frame		 
	wTh = bot->GetHeadBase2WorldTransform();
	xw = wTh*xh;
	wTe = bot->GetRightEye2WorldTransform();
		
	eTw = wTe.Inverse();
	
	xe = eTw*xw;

	mReal tl= sqrt(xe[0]*xe[0]+ xe[1]*xe[1]+ xe[2]*xe[2]); //length to target in camera frame
	//Hack to avoid singularities...need to redo w/o atan2 singularity in workspace
	xe[0]=MAX(tl*0.1,xe[0]); //point must be well in front of camera, otherwise atan2 flips.
	mReal eye_pan=q5c+RAD2DEG(atan2(xe[0],-1.0*xe[1]))-90.0; //angle that will pan the eye to target
	//if (tmp_cnt++%100==0)
	//  M3_INFO("Atan2J4 %f %f: %f + %f TL %f\n",xe[0],-1.0*xe[1],RAD2DEG(atan2(xe[0],-1.0*xe[1])),q5c,tl);
	theta_des[5]=eye_pan;
	theta_des[6]=eye_pan;
	
	
	mReal eye_tilt=q4c+RAD2DEG(atan2(xe[2],xe[0])); //angle that will tilt the eye to target
	theta_des[4]=eye_tilt;
	

	
	mReal neck_pan;
	if (ABS(q5c)<param.theta_db(1))
		neck_pan=q1c;
	else
		neck_pan=q1c+q5c;
	theta_des[1]=neck_pan;
	
	mReal head_pitch;
	if (ABS(q4c)<param.theta_db(3))
		head_pitch=q3c;
	else
		head_pitch=q3c+q4c;
	theta_des[3]=head_pitch;
	
	mReal neck_pitch;
	//Need to pass offset transform as param
	mReal qt0 = RAD2DEG(atan2(xh[2],xh[0])); //angle of target from vertical lower neck
	//if (tmp_cnt++%100==0)
	//	M3_INFO("qt0: %f %f %f\n",qt0,xw[2]-param.origin(2),xw[0]-param.origin(0));
	//if (ABS(q4c)<param.theta_db(0))
	//	neck_pitch=q0c;
	//else
	neck_pitch=qt0*param.ratio_j0();//q0c+param.ratio_j0()*q3c; //param.ratio_j0()*
	theta_des[0]=neck_pitch;
	
	/*for (int i=0;i<4;i++)
	{
		bot->SetThetaDeg(HEAD,i,theta_des[i]);
		bot->SetModeThetaMj(HEAD,i);//bot->SetModeTheta(HEAD,i);
		bot->SetThetaDotDeg(HEAD,i,param.slew_des(i));//bot->SetSlewRateProportional(HEAD,i,param.slew_des(i));
	}*/
	for (int i=0;i<4;i++)
	{
 		bot->SetThetaDeg(HEAD,i,joint_smooth[i].Step(theta_des[i]));
		bot->SetModeTheta(HEAD,i);
		bot->SetSlewRateProportional(HEAD,i,param.slew_des(i));
	}
	for (int i=4;i<7;i++)
	{
		bot->SetThetaDeg(HEAD,i,theta_des[i]);
		bot->SetModeTheta(HEAD,i);
		bot->SetSlewRateProportional(HEAD,i,param.slew_des(i));
	}
}

void M3HeadS2CSPCtrl::StepStatus()
{	
	if (IsStateError())
		return;
	status.set_xe(0,xe[0]);
	status.set_xe(1,xe[1]);
	status.set_xe(2,xe[2]);	
	for (int i=0;i<7;i++)
		status.set_theta_des(i,theta_des[i]);
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3HeadS2CSPCtrl::ReadConfig(const char * filename)
{
	if (!M3Component::ReadConfig(filename))
		return false;
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	mReal mval;
	doc["robot_component"] >> bot_name;
	doc["head_component"] >> head_name;
	doc["param"]["target_slew"] >> mval;
	param.set_target_slew(mval);
	vector<mReal> sd,db;
	doc["param"]["slew_des"]>>slew_des;
	doc["param"]["theta_db"]>>theta_db;
	doc["param"]["origin"]>>origin;
	doc["param"]["ratio_j0"]>>mval;
	param.set_ratio_j0(mval);
	return true;
}



}
