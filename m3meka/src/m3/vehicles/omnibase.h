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

#ifndef M3_OMNIBASE_H
#define M3_OMNIBASE_H

#include "holomni_pcv/Pcv.h"

#define DEG2RAD(a)	(mReal)2.0*M_PI*((mReal)a)/360.0
#define RAD2DEG(a)	(mReal)360.0*((mReal)a)/(2.0*M_PI)


#include "m3rt/base/component.h"
#include "m3/vehicles/vehicle.h"
#include "m3/chains/joint_array.h"
#include "m3/vehicles/omnibase.pb.h"
#include "m3/toolbox/toolbox.h"
#include "m3/hardware/pwr.h"


namespace m3
{
	using namespace std;
	
///////////////////////////////////////////////////////////////////////////

class M3Omnibase : public M3Vehicle
{
	public:
		M3Omnibase():M3Vehicle(),motor_array(NULL),pcv(NULL),tmp_cnt(0),
		  old_ctrl_mode(OMNIBASE_CTRL_OFF),old_traj_mode(OMNIBASE_TRAJ_OFF),cnt(0),pwr(0)
		{
			RegisterVersion("default",DEFAULT);	//RBL
			RegisterVersion("iss",ISS);		//ISS. Safe as DEFAULT
		}
		//void StepStatus();
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
	private:
		enum {DEFAULT, ISS};
		bool ReadConfig(const char * filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();
		void StepCartesianLocalCtrl();
		void StepCartesianGlobalCtrl();
		void StepOpSpaceForceCtrl();
		void StepOpSpaceTrajCtrl();
		void StepHomeCtrl();
		void StepOffCtrl();
		void StepCasterCtrl();
		void StepJoystickCtrl();
		void StepTrajGoalCtrl();
		void EnableBreakbeamEncoderZero(int idx);
		void DisableBreakbeamEncoderZero(int idx);
		void StepTrajViaCtrl();
		vector<M3OmniVia> vias;
		M3OmniVia new_via;
		M3OmniVia active_via;
		mReal WrapDeg(mReal deg);
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
		M3OmnibaseStatus status;
		M3OmnibaseCommand command;
		M3OmnibaseParam param;			
		vector<M3PID> pid_steer_vel;
		vector<M3PID> pid_roll_vel;
		vector<M3PID> pid_steer_theta;
		vector<bool> old_calibrated;
		M3JointArray * motor_array;
		string joint_array_name;
		vector<double> opspace_tq_desired;
		PCV * pcv;
		string holomni_pcv_config;
		Pcv_Command pcv_cmd;
		Pcv_Status pcv_status;
		OMNIBASE_CTRL_MODE old_ctrl_mode;
		OMNIBASE_TRAJ_MODE old_traj_mode;
		double max_tq[8];
		double min_tq[8];
		double max_pos_err[3];
		double max_vel_err[3];
		double max_cdxx[3];
		double min_cdxx[3];
		bool first_flip_out[4];
		mReal flip_out_detect_mult;
		mReal flip_out_detect_min;
		bool flip_out_detect_enable;
		int flip_out_detect_timeout[4];
		int flip_out_detect_timeout_cnt;
		int flip_out_detect_time[4];
		int flip_out_detect_time_cnt;
		bool flip_out_detect_disable_0;
		bool flip_out_detect_disable_1;
		bool flip_out_detect_disable_2;
		bool flip_out_detect_disable_3;
		int cnt;
		int tmp_cnt;
		M3Pwr * pwr;
		string pwr_name;
		//M3JointFilter angle_df;
};

}

#endif


