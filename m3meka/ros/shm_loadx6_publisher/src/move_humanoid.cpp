#include <iostream>

#include <ros/ros.h>

#include <m3ctrl_msgs/M3JointCmd.h>
#include "m3/hardware/joint_mode_ros.pb.h"
#include "m3/robots/chain_name.h"
#include "m3/hardware/smoothing_mode.pb.h"

class HumanoidDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_pub_;

public:
  //! ROS node initialization
  HumanoidDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_pub_ = nh_.advertise<m3ctrl_msgs::M3JointCmd>("/humanoid_command", 1);
  }

  //! Loop forever while sending drive commands based on keyboard input
  bool driveKeyboard()
  {
    char cmd[50];
     m3ctrl_msgs::M3JointCmd humanoid_cmd;
     
    humanoid_cmd.chain.resize(1);
    humanoid_cmd.stiffness.resize(1);
    humanoid_cmd.position.resize(1);
    humanoid_cmd.velocity.resize(1);
    humanoid_cmd.control_mode.resize(1);
    humanoid_cmd.smoothing_mode.resize(1);
    humanoid_cmd.chain_idx.resize(1);
     
    //  center robot first
    
    std::cout << "Example: commanding right arm J0.\n";
    std::cout << "Press any key to move to zero position.\n";
    std::cin.getline(cmd, 50);
    
    humanoid_cmd.chain[0] = (unsigned char)RIGHT_ARM;
    humanoid_cmd.chain_idx[0] = 0; //J0
    humanoid_cmd.control_mode[0] = (unsigned char)JOINT_MODE_ROS_THETA_GC; //Compliant position mode
    humanoid_cmd.smoothing_mode[0] = (unsigned char)SMOOTHING_MODE_SLEW; //Smooth trajectory
    humanoid_cmd.velocity[0] = 3.0; //Rad/s
    humanoid_cmd.stiffness[0] = 1.0; //0-1.0
    humanoid_cmd.position[0] = 0; //Desired position (Rad)
    humanoid_cmd.header.stamp = ros::Time::now();
    humanoid_cmd.header.frame_id = "humanoid_cmd";
    cmd_pub_.publish(humanoid_cmd);
  
    std::cout << "Type a command and then press enter.  "
      "Use 'z' to go to middle, '+' to move up, '-' to move down, "
      "'.' to exit.\n";

    while(nh_.ok()){

      std::cin.getline(cmd, 50);
      if(cmd[0]!='+' && cmd[0]!='-' && cmd[0]!='z' && cmd[0]!='.')
      {
        std::cout << "unknown command:" << cmd << "\n";
        continue;
      }
      
      //move forward
      if(cmd[0]=='+'){
        humanoid_cmd.position[0] += 5.0 * 3.14/180.;
      } 
      //turn left (yaw) and drive forward at the same time
      else if(cmd[0]=='-'){
        humanoid_cmd.position[0] -= 5 * 3.14/180.;
      } 
      //turn right (yaw) and drive forward at the same time
      else if(cmd[0]=='z'){
        humanoid_cmd.position[0] = 0 * 3.14/180.;
      } 
      //quit
      else if(cmd[0]=='.'){
        break;
      }
            
      humanoid_cmd.header.stamp = ros::Time::now();
      //publish the assembled command
      cmd_pub_.publish(humanoid_cmd);
    }
    return true;
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  HumanoidDriver driver(nh);
  driver.driveKeyboard();
}