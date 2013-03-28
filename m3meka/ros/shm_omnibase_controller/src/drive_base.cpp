#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub_;

public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("omnibase_command", 1);
  }

  //! Loop forever while sending drive commands based on keyboard input
  bool driveKeyboard()
  {
    char cmd[50];
    
    std::cout << "Lift E-Stop now to ensure safe startup!\n";
    std::cout << "Hit enter when ready\n";
    std::cin.getline(cmd, 50);
    std::cout << "Type a command and then press enter.  "
      "Use '+' to move forward, '-' to move backwards, 'l' to turn left, "
      "'r' to turn right, 's' to stop, '.' to exit.\n";

    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;

    
    while(nh_.ok()){

      std::cin.getline(cmd, 50);
      if(cmd[0]!='+' && cmd[0]!='-' && cmd[0]!='l' && cmd[0]!='r' && cmd[0]!='.' && cmd[0]!='s')
      {
        std::cout << "unknown command:" << cmd << "\n";
        continue;
      }

      base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;   
      //move forward
      if(cmd[0]=='+'){
        base_cmd.linear.x = 0.25;
	base_cmd.linear.y = 0.;
	base_cmd.angular.z = 0.;
      } 
      //move back
      if(cmd[0]=='-'){
        base_cmd.linear.x = -0.25;
	base_cmd.linear.y = 0.;
	base_cmd.angular.z = 0.;
      } 
      //turn left (yaw) and drive forward at the same time
      else if(cmd[0]=='l'){
	base_cmd.linear.x = 0.;
	base_cmd.linear.y = 0.;
        base_cmd.angular.z = 0.75;
//        base_cmd.linear.x = 0.25;
      } 
      //turn right (yaw) and drive forward at the same time
      else if(cmd[0]=='r'){
	base_cmd.linear.x = 0.;
	base_cmd.linear.y = 0.;
        base_cmd.angular.z = -0.75;
//        base_cmd.linear.x = 0.25;
      }
      else if(cmd[0]=='s'){
        base_cmd.linear.x = 0.;
	base_cmd.linear.y = 0.;
        base_cmd.angular.z = 0.;
      } 
      //quit
      else if(cmd[0]=='.'){
        break;
      }
            
      
      //publish the assembled command
      cmd_vel_pub_.publish(base_cmd);
    }
    
    base_cmd.linear.x = 0.;
	base_cmd.linear.y = 0.;
        base_cmd.angular.z = 0.;
	cmd_vel_pub_.publish(base_cmd);
	
    return true;
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "base_driver");
  ros::NodeHandle nh;

  RobotDriver driver(nh);
  driver.driveKeyboard();
}
