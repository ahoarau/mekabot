
#ifndef	_HUMANOID_CLIENT_H
#define	_HUMANOID_CLIENT_H

#include "ros/ros.h"
#include "m3_client/M3HumanoidCmd.h"
#include "m3_client/M3HumanoidParam.h"
#include "m3_client/M3HumanoidStatus.h"
#include "m3/chains/joint_array_mode.pb.h"
#include <cstdlib>
#include <iostream>
#include <fstream>
#include "yaml-cpp/yaml.h"
#include <string>
#include "m3/robots/chain_name.h"
#include <Eigen/Core>

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

using namespace ros;
using namespace m3_client;
using namespace std;

class M3HumanoidClient 
{
	public:
		M3HumanoidClient(NodeHandle * node_handle);
		void SetThetaDeg(M3Chain chain,unsigned int  idx, double theta);
		double GetThetaDeg(M3Chain chain,unsigned int  idx);
		double GetThetaDotDeg(M3Chain chain,unsigned int  idx);
		double GetThetaDotDotDeg(M3Chain chain,unsigned int  idx);
		double GetTorque_mNm(M3Chain chain,unsigned int  idx);
		double GetTorqueDot_mNm(M3Chain chain,unsigned int  idx);
		Eigen::Vector3d GetEndPosition(M3Chain chain); //3x1 position of End frame in Base coords (x,y,z)
		Eigen::Matrix3d GetEndRotation(M3Chain chain); //3x3 Rotation Mtx
		Eigen::MatrixXd GetJacobian(M3Chain chain); //6xndof Jacobian Frame ndof+1 to Frame 0
		double GetGravity(M3Chain chain,unsigned int idx);
		double GetPwm(M3Chain chain,unsigned int  idx);
		void SetTorque_mNm(M3Chain chain,unsigned int  idx, double torque);
		void SetThetaDotDeg(M3Chain chain,unsigned int  idx, double theta_dot);
		void SetPwm(M3Chain chain,unsigned int  idx, double pwm);
		void SetSlewRate(M3Chain chain,unsigned int  idx, double slew_rate);  // degs/sec
		void SetSlewRateProportional(M3Chain chain,unsigned int  idx, double slew_rate);  // 0.0 -> 1.0
		void SetModePwm(M3Chain chain,unsigned int  idx);
		void SetModeTorque(M3Chain chain,unsigned int  idx);
		void SetModeThetaGcMj(M3Chain chain,unsigned int  idx);
		void SetModeThetaMj(M3Chain chain,unsigned int  idx);
		void SetModeThetaGc(M3Chain chain,unsigned int  idx);
		void SetModeTheta(M3Chain chain,unsigned int  idx);
		void SetModeOff(M3Chain chain,unsigned int  idx);
		void SetStiffness(M3Chain chain,unsigned int  idx, double stiffness);  // 0.0 -> 1.0
		void SetMotorPowerOn();
		void SetMotorPowerOff();
		/*void EnableVelocityGains(M3Chain chain);
		void DisableVelocityGains(M3Chain chain);
		void EnableAccelerationGains(M3Chain chain);
		void DisableAccelerationGains(M3Chain chain);*/
		double GetPayloadMass(M3Chain chain);
		double GetPayloadInertia(M3Chain chain,unsigned int  idx);
		double GetPayloadCom(M3Chain chain,unsigned int  idx);
		void SetPayloadMass(M3Chain chain, double mass);
		void SetPayloadCom(M3Chain chain,unsigned int  idx, double com);
		void SetPayloadInertia(M3Chain chain,unsigned int  idx, double inertia);
		void UpdateStatus();
		void SendCommand();				
		int GetNdof(M3Chain chain);
	private:	  
		string GetRobotName();
		string GetCompDir(string component);
		bool ReadConfig();
		void LoadParam(const YAML::Node * node, M3HumanoidParam * param);
		void LoadMaxSlew(const YAML::Node * node, vector<double> * max_slew);
		bool GetEnvironmentVar(const char * var, string &s);
		void GetYamlDoc(string filename, YAML::Node & doc);
		M3HumanoidCmd * GetCmd(M3Chain chain);
		M3HumanoidStatus * GetStatus(M3Chain chain);
		M3HumanoidParam * GetParam(M3Chain chain);
		void ResizeCmd(M3Chain chain, int ndof);
		M3HumanoidCmd cmd_right_arm;  
		M3HumanoidStatus status_right_arm;
		M3HumanoidParam param_right_arm;
		M3HumanoidCmd cmd_left_arm;  
		M3HumanoidStatus status_left_arm;
		M3HumanoidParam param_left_arm;
		M3HumanoidCmd cmd_torso;  
		M3HumanoidStatus status_torso;
		M3HumanoidParam param_torso;
		M3HumanoidCmd cmd_head;  
		M3HumanoidStatus status_head;
		M3HumanoidParam param_head;		
		ServiceClient cmd_client;
		ServiceClient param_client;
		ServiceClient status_client;
		int ndof_right_arm;
		int ndof_left_arm;
		int ndof_torso;
		int ndof_head;
		vector<double> max_slew_right_arm;
		vector<double> max_slew_left_arm;
		vector<double> max_slew_torso;
		vector<double> max_slew_head;
};
		
		
#endif /* humanoid_client.h  */