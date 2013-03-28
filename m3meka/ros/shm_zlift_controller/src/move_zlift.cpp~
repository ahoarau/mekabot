#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <m3ctrl_msgs/M3JointCmd.h>
#include <m3/hardware/joint_mode_ros.pb.h>
#include <m3/hardware/smoothing_mode.pb.h>



class ZliftDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_pub_;

public:
  //! ROS node initialization
  ZliftDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_pub_ = nh_.advertise<m3ctrl_msgs::M3JointCmd>("/zlift_command", 1);
  }

  //! Loop forever while sending drive commands based on keyboard input
  bool driveKeyboard()
  {
    char cmd[50];
     m3ctrl_msgs::M3JointCmd zlift_cmd;
     
    zlift_cmd.chain.resize(1);
    zlift_cmd.stiffness.resize(1);
    zlift_cmd.position.resize(1);
    zlift_cmd.velocity.resize(1);
    zlift_cmd.control_mode.resize(1);
    zlift_cmd.smoothing_mode.resize(1);
    zlift_cmd.chain_idx.resize(1); 
     
    //  center robot first
    
    std::cout << "Press any key to center robot.\n";
    std::cin.getline(cmd, 50);
    
    
    zlift_cmd.control_mode[0] = (unsigned char)JOINT_MODE_ROS_THETA_GC;
    zlift_cmd.smoothing_mode[0] = (unsigned char)SMOOTHING_MODE_SLEW;
    zlift_cmd.velocity[0] = 2000.0;
    zlift_cmd.stiffness[0] = 1.0;
    zlift_cmd.header.stamp = ros::Time::now();
    zlift_cmd.header.frame_id = "zlift_cmd";
    zlift_cmd.position[0] = 600;
    cmd_pub_.publish(zlift_cmd);
  
    std::cout << "Type a command and then press enter.  "
      "Use 'm' to go to middle, '+' to move up, '-' to move down, "
      "'.' to exit.\n";

    

    
    while(nh_.ok()){

      std::cin.getline(cmd, 50);
      if(cmd[0]!='+' && cmd[0]!='-' && cmd[0]!='m' && cmd[0]!='.')
      {
        std::cout << "unknown command:" << cmd << "\n";
        continue;
      }
      
      //move forward
      if(cmd[0]=='+'){
        zlift_cmd.position[0] += 10;
      } 
      //turn left (yaw) and drive forward at the same time
      else if(cmd[0]=='-'){
        zlift_cmd.position[0] -= 10;
      } 
      //turn right (yaw) and drive forward at the same time
      else if(cmd[0]=='m'){
        zlift_cmd.position[0] = 600;
      } 
      //quit
      else if(cmd[0]=='.'){
        break;
      }
            
      zlift_cmd.header.stamp = ros::Time::now();
      //publish the assembled command
      cmd_pub_.publish(zlift_cmd);
    }
    
    // put in mode off b4 quitting
    zlift_cmd.control_mode[0] = (unsigned char)JOINT_MODE_ROS_OFF;
    cmd_pub_.publish(zlift_cmd);
    return true;
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  ZliftDriver driver(nh);
  driver.driveKeyboard();
}