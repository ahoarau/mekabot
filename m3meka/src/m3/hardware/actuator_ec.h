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

#ifndef M3_ACTUATOR_EC_H
#define M3_ACTUATOR_EC_H

#include <m3rt/base/component.h>
#include <m3rt/base/component_ec.h>
#include <m3/hardware/actuator_ec.pb.h>
#include "m3/hardware/pwr.h"
#include "m3/toolbox/toolbox.h"
#include <google/protobuf/message.h>
#include "m3rt/base/m3ec_def.h"
#include "m3/hardware/m3ec_pdo_v0_def.h"
#include "m3/hardware/m3ec_pdo_v1_def.h"
#include "m3/hardware/m3ec_pdo_v2_def.h"
#include "m3/hardware/m3ec_pdo_v3_def.h"
#include "m3/hardware/m3ec_pdo_v4_def.h"

namespace m3{
using namespace std;


class M3ActuatorEc : public  m3rt::M3ComponentEc{
	public:
		M3ActuatorEc():ignore_pwm_slew(0), pwr(NULL),pdo_status_size(0),toggle(0),pdo_cmd_size(0),
			      pwm_ff(0),pwm_max_ext(0),error_printed(false),tmp_cnt(0),motor_power_slewed_on(false),
			      tq_err_cnt(0),qei_err_cnt(0), override_ext_temp(false), override_ext_temp_act_ec(NULL), m3rt::M3ComponentEc()
		{
			memset(&exs,0,sizeof(M3ActPdoV2StatusExt));
			memset(&exc,0,sizeof(M3ActPdoV2CmdExt));
			memset(&axc,0,sizeof(M3ActPdoV2Cmd));
			memset(&acc,0,sizeof(M3ActPdoV1Cmd));
			memset(&scc,0,sizeof(M3SeaPdoV0Cmd));
		
			RegisterVersion("default",DEFAULT);		//RBL
			RegisterVersion("iss",ISS);			//ISS. No change from DEFAULT
			RegisterVersion("esp",ESP);			//ESP. Moved torque feedforward from DSP to Component 
			RegisterVersion("iq",IQ);
			RegisterPdo("actx1_pdo_v3", ACTX1_PDO_V3);	//CRL
			RegisterPdo("actx1_pdo_v1", ACTX1_PDO_V1);	//RBL 
			RegisterPdo("actx2_pdo_v1", ACTX2_PDO_V1);	//RBL 
			RegisterPdo("actx3_pdo_v1", ACTX3_PDO_V1);	//RBL 
			RegisterPdo("actx4_pdo_v1", ACTX4_PDO_V1);	//RBL 
			
			RegisterPdo("actx1_pdo_v4", ACTX1_PDO_V4);	//IQ 
			RegisterPdo("actx2_pdo_v4", ACTX2_PDO_V4);	//IQ 
			RegisterPdo("actx3_pdo_v4", ACTX3_PDO_V4);	//IQ 
			
			RegisterPdo("tactx2_pdo_v1", TACTX2_PDO_V1);	//RBL 
			RegisterPdo("actx1_pdo_v2", ACTX1_PDO_V2);	//ISS 
			RegisterPdo("actx2_pdo_v2", ACTX2_PDO_V2);	//ISS 
			RegisterPdo("actx3_pdo_v2", ACTX3_PDO_V2);	//ISS 
			RegisterPdo("actx4_pdo_v2", ACTX4_PDO_V2);	//ISS 
			RegisterPdo("sea_pdo_v0", SEA_PDO_V0);		//HRL
		}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
		
		bool IsMotorPowerOn(){return pwr->IsMotorPowerOn();}
		void SetMotorEnable(int on){pwr->SetMotorEnable(on);}
		bool IsMotorPowerSlewedOn(){return motor_power_slewed_on;}
		void SetZeroEncoder(){param.set_config(param.config()|ACTUATOR_EC_CONFIG_CALIB_QEI_MANUAL);}
		void ClrZeroEncoder(){param.set_config(param.config()&~ACTUATOR_EC_CONFIG_CALIB_QEI_MANUAL);}
		void SetLimitSwitchNegZeroEncoder(){param.set_config(param.config()|ACTUATOR_EC_CONFIG_CALIB_QEI_LIMITSWITCH_NEG);}
		void SetLimitSwitchPosZeroEncoder(){param.set_config(param.config()|ACTUATOR_EC_CONFIG_CALIB_QEI_LIMITSWITCH_POS);}
		void ClrLimitSwitchNegZeroEncoder(){param.set_config(param.config()&~ACTUATOR_EC_CONFIG_CALIB_QEI_LIMITSWITCH_NEG);}
		void ClrLimitSwitchPosZeroEncoder(){param.set_config(param.config()&~ACTUATOR_EC_CONFIG_CALIB_QEI_LIMITSWITCH_POS);}
		bool IsLimitSwitchPosOn() {return status.flags()&ACTUATOR_EC_FLAG_POS_LIMITSWITCH;}
		bool IsLimitSwitchNegOn() {return status.flags()&ACTUATOR_EC_FLAG_NEG_LIMITSWITCH;}
		bool IsAuxSwitchOn() {return status.flags()&ACTUATOR_EC_FLAG_AUX_SWITCH;}
		bool IsEncoderCalibrated(){return status.flags()&ACTUATOR_EC_FLAG_QEI_CALIBRATED;}
		bool UseTorqueFF(){return param.config() & ACTUATOR_EC_CONFIG_TORQUE_FF;}
		void SetTorqueFF(mReal val){pwm_ff=(int)val;} //Added to DSP PID result (units pwm ticks)		
		void SetPwmMax(int val){pwm_max_ext=val;}
		bool IsCurrentFaultMom() {return status.flags()&M3ACT_FLAG_I_FAULT_MOM;}
		bool IsCurrentFaultCont() {return status.flags()&M3ACT_FLAG_I_FAULT_CONT;}
		int GetTorqueErr();
		int GetThetaErr();
		
	protected:
		bool ReadConfig(const char * filename);
		M3EtherCATStatus * GetEcStatus(){return status.mutable_ethercat();}
		void SetStatusFromPdo(unsigned char * data);
		void SetStatusFromPdoV0(unsigned char * data);
		void SetStatusFromPdoV1(unsigned char * data);
		void SetStatusFromPdoV2(unsigned char * data);
		void SetStatusFromPdoV3(unsigned char * data);
		void SetStatusFromPdoV4(unsigned char * data);
		void SetPdoFromCommand(unsigned char * data);
		bool LinkDependentComponents();
		void ResetCommandPdo(unsigned char * pdo);
		void SetPdoV4FromPdoV1Command(unsigned char * data);
		void SetPdoV2FromPdoV1Command(unsigned char * data);
		void SetPdoV0FromPdoV1Command(unsigned char * data);
		void StepStatus();
	protected:
		enum {GMB_PDO_V0,ACTX1_PDO_V1, ACTX2_PDO_V1, ACTX3_PDO_V1, ACTX4_PDO_V1, TACTX2_PDO_V1,
		      ACTX1_PDO_V2, ACTX2_PDO_V2, ACTX3_PDO_V2, ACTX4_PDO_V2,SEA_PDO_V0,
		      ACTX1_PDO_V3,
		      ACTX1_PDO_V4, ACTX2_PDO_V4, ACTX3_PDO_V4,};
		enum {DEFAULT,ISS, ESP, IQ};
		M3BaseStatus * GetBaseStatus();
		M3ActuatorEcStatus status;
		M3ActuatorEcCommand command;
		M3ActuatorEcParam param;
		M3Pwr * pwr;
		string pwr_name;
		M3TimeSlew pwr_slew;
		M3TimeSlew pwm_slew;
		int pdo_status_size;
		int pdo_cmd_size;
		int pwm_ff;
		int tmp_cnt;
		int chid;
		int pwm_max_ext;
		int ignore_pwm_slew;
		M3ActPdoV2StatusExt exs;
		M3ActPdoV2Cmd    axc;
		M3ActPdoV2CmdExt exc;
		M3ActPdoV1Cmd  acc;
		M3SeaPdoV0Cmd  scc;
		bool error_printed;
		bool motor_power_slewed_on;
		int toggle;
		int qei_err_cnt;
		int tq_err_cnt;
		bool has_brake;
		bool override_ext_temp;
		M3ActuatorEc * override_ext_temp_act_ec;
		string override_ext_temp_act_ec_name;
};


































































































}
#endif


