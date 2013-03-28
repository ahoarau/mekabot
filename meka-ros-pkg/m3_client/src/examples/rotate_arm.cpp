#include "rotate_arm.h"

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