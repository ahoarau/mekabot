#include "../hand_client.h"
#include "../humanoid_client.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hand_client_example");  
  ros::NodeHandle node_handle;
  M3HandClient hand = M3HandClient("m3hand_mh8", &node_handle);
  M3HumanoidClient bot = M3HumanoidClient(&node_handle);  // Just for turning on motor power
  
  int finger = 2;
 // Get J2 angle
 hand.UpdateStatus();
 double J2_theta = hand.GetThetaDeg(finger);
 
 // Rotate finger 2  30 degrees and back at 10 deg/sec
 ros::Rate r(10); // 10 hz
 hand.SetModeThetaGc(finger);
 hand.SetStiffness(finger, 0.8);
 hand.SetSlewRate(finger, 20);
 bot.SetMotorPowerOn();
  for (int i = 0; i < 30; i++)
  {   
      hand.UpdateStatus();
      ROS_INFO("Finger 2: %f", hand.GetThetaDeg(finger));
      hand.SetThetaDeg(finger,J2_theta + i);
      hand.SendCommand();            
      r.sleep();
  }  
  for (int i = 30; i >= 0; i--)
  {        
      hand.UpdateStatus();
      ROS_INFO("Finger 2: %f", hand.GetThetaDeg(finger));
      hand.SetThetaDeg(finger,J2_theta + i);
      hand.SendCommand();
      r.sleep();
  }
  
  bot.SetMotorPowerOff();
 
 return 0;
}