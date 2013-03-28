#include "../humanoid_client.h"
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "humanoid_client_example");  
  ros::NodeHandle node_handle;
  M3HumanoidClient bot = M3HumanoidClient(&node_handle); 
  
  // Example usage for each M3HumanoidClient API member  
  bot.SetThetaDeg(RIGHT_ARM, 1, 20);
  ROS_INFO("Right J1 Angle: %f", bot.GetThetaDeg(RIGHT_ARM,1));
  ROS_INFO("Right J1 Vel: %f", bot.GetThetaDotDeg(RIGHT_ARM,1));
  ROS_INFO("Right J1 Accel: %f", bot.GetThetaDotDotDeg(RIGHT_ARM,1));
  ROS_INFO("Right J1 Torque: %f", bot.GetTorque_mNm(RIGHT_ARM,1));
  ROS_INFO("Right J1 TorqueDot: %f", bot.GetTorqueDot_mNm(RIGHT_ARM,1));
  Eigen::Vector3d end_pos = bot.GetEndPosition(RIGHT_ARM);  
  ROS_INFO("End Pos = x:%f, y:%f, z:%f", end_pos[0], end_pos[1], end_pos[2]);
  Eigen::Matrix3d end_rot = bot.GetEndRotation(RIGHT_ARM);
  ROS_INFO("End Rot = xx:%f, xy:%f, xz:%f", end_rot(0,0), end_rot(0,1), end_rot(0,2));
  ROS_INFO("End Rot = yx:%f, yy:%f, yz:%f", end_rot(1,0), end_rot(1,1), end_rot(1,2));
  ROS_INFO("End Rot = zx:%f, zy:%f, zz:%f", end_rot(2,0), end_rot(2,1), end_rot(2,2));
  Eigen::MatrixXd jac = bot.GetJacobian(RIGHT_ARM);
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < bot.GetNdof(RIGHT_ARM); j++)
      ROS_INFO("Jac(%i,%i): %f", i, j, jac(i,j));
  }
  ROS_INFO("Right J1 Gravity: %f", bot.GetGravity(RIGHT_ARM, 1));
  ROS_INFO("Right J1 PWM: %f", bot.GetPwm(RIGHT_ARM, 1));
  bot.SetTorque_mNm(RIGHT_ARM, 1, 10);
  bot.SetThetaDotDeg(RIGHT_ARM, 1, 15);
  bot.SetPwm(RIGHT_ARM, 1, 50);
  bot.SetSlewRate(RIGHT_ARM,1, 30);
  bot.SetSlewRateProportional(RIGHT_ARM,1, 0.8);
  bot.SetModePwm(RIGHT_ARM, 1);
  bot.SetModeTorque(RIGHT_ARM, 1);
  bot.SetModeThetaGcMj(RIGHT_ARM, 1);
  bot.SetModeThetaMj(RIGHT_ARM, 1);
  bot.SetModeThetaGc(RIGHT_ARM, 1);
  bot.SetModeTheta(RIGHT_ARM, 1);
  bot.SetModeOff(RIGHT_ARM, 1);
  bot.SetStiffness(RIGHT_ARM, 1, 0.7);
  bot.SetMotorPowerOn();
  bot.SetMotorPowerOff();
  bot.SetPayloadMass(RIGHT_ARM, 0.5);
  bot.SetPayloadCom(RIGHT_ARM, 0, 0.02);
  bot.SetPayloadInertia(RIGHT_ARM, 0, 0.2);
  ROS_INFO("Right Payload Mass: %f", bot.GetPayloadMass(RIGHT_ARM));
  ROS_INFO("Right Payload COM[0]: %f", bot.GetPayloadCom(RIGHT_ARM, 0));
  ROS_INFO("Right Payload Inertia[0]: %f", bot.GetPayloadInertia(RIGHT_ARM, 0));
  void UpdateStatus();
  void SendCommand();
 
 return 0;
}