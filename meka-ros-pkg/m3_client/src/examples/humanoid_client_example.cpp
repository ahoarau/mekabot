#include "../humanoid_client.h"


using namespace std;

void  RotateArm(M3HumanoidClient &bot, double right_elbow_theta, ros::Rate &r)
{
  for (int i = 0; i < 30; i++)
  {   
      bot.UpdateStatus();
      ROS_INFO("Right Elbow: %f", bot.GetThetaDeg(RIGHT_ARM,3));
      bot.SetThetaDeg(RIGHT_ARM,3,right_elbow_theta + i);
      bot.SendCommand();            
      r.sleep();
  }  
  for (int i = 30; i >= 0; i--)
  {        
      bot.UpdateStatus();
      ROS_INFO("Right Elbow: %f", bot.GetThetaDeg(RIGHT_ARM,3));
      bot.SetThetaDeg(RIGHT_ARM,3,right_elbow_theta + i);
      bot.SendCommand();
      r.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "humanoid_client_example");  
  ros::NodeHandle node_handle;
  M3HumanoidClient bot = M3HumanoidClient(&node_handle);  
  
 // Get Right Arm Elbow angle
 bot.UpdateStatus();
 double right_elbow_theta = bot.GetThetaDeg(RIGHT_ARM, 3);
 
  // Rotate right arm 30 degrees and back at 10 deg/sec
 ros::Rate r(10); // 10 hz
 bot.SetModeThetaGc(RIGHT_ARM, 3);
 bot.SetStiffness(RIGHT_ARM, 3, 0.8);
 bot.SetSlewRateProportional(RIGHT_ARM, 3, 1.0);
 bot.SetMotorPowerOn();
  
 RotateArm(bot, right_elbow_theta, r);
  
  bot.SetMotorPowerOff();
 
 return 0;
}