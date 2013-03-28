/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2010  University of Osnabrück
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * JointStatePublisher.cpp
 *
 *  Created on: 06.12.2010
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#include "meka_driver/JointStatePublisher.h"

namespace meka_driver
{

JointStatePublisher::JointStatePublisher(boost::shared_ptr<AbstractMeka> meka) :
  meka(meka)
{
  ros::NodeHandle nh;
  pub = nh.advertise<sensor_msgs::JointState> ("joint_states", 1000);
}

JointStatePublisher::~JointStatePublisher()
{
}

void JointStatePublisher::update()
{
  /* ************** Publish joint angles ************** */
  sensor_msgs::JointStatePtr msg = boost::make_shared<sensor_msgs::JointState>();
  std::vector<std::string> joint_names = meka->getJointNames();
  std::vector<double> angles = meka->getMotorAngles();
  std::vector<double> vels = meka->getMotorVelocities();

  for (size_t i = 0; i < NUM_JOINTS; i++)
  {
    msg->name.push_back(joint_names[i]);
    msg->position.push_back(angles[i]);
    msg->velocity.push_back(vels[i]);
    //msg->position.push_back(0.2);
    //msg->velocity.push_back(0.);
    
  }
#if 0  
  msg->name.push_back("floating_rot_w");
  msg->position.push_back(0.0);
  msg->velocity.push_back(0.0);
   
  msg->name.push_back("floating_rot_x");
  msg->position.push_back(0.0);
  msg->velocity.push_back(0.0);
  
  msg->name.push_back("floating_rot_y");
  msg->position.push_back(0.0);
  msg->velocity.push_back(0.0);
  
  msg->name.push_back("floating_rot_z");
  msg->position.push_back(0.0);
  msg->velocity.push_back(0.0);
  
  msg->name.push_back("floating_trans_x");
  msg->position.push_back(0.0);
  msg->velocity.push_back(0.0);
  
  msg->name.push_back("floating_trans_y");
  msg->position.push_back(0.0);
  msg->velocity.push_back(0.0);
  
  msg->name.push_back("floating_trans_z");
  msg->position.push_back(0.0);
  msg->velocity.push_back(0.0);

   msg->name.push_back("m3joint_mt3_j0");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
   
   msg->name.push_back("m3joint_mt3_j1");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
   
   msg->name.push_back("m3joint_mt3_j2");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
   
   
   msg->name.push_back("m3joint_ua_mh8_j0");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
   
   msg->name.push_back("m3joint_ua_mh8_j1");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
   
   msg->name.push_back("m3joint_ua_mh8_j2");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
   
   msg->name.push_back("m3joint_ua_mh8_j3");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
   
   msg->name.push_back("m3joint_ua_mh8_j4");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
   
   msg->name.push_back("m3joint_ua_mh8_j5");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
   
   msg->name.push_back("m3joint_ua_mh8_j6");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
   
   msg->name.push_back("m3joint_ua_mh8_j7");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
   
   msg->name.push_back("m3joint_ua_mh8_j8");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
   
   msg->name.push_back("m3joint_ua_mh8_j9");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
   
   msg->name.push_back("m3joint_ua_mh8_j10");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
      
   msg->name.push_back("m3joint_ua_mh8_j11");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
   
   msg->name.push_back("m3joint_ms2_j0");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
   
   msg->name.push_back("m3joint_ms2_j1");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
   
   msg->name.push_back("m3joint_ms2_j2");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
   
   msg->name.push_back("m3joint_ms2_j3");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
   
   msg->name.push_back("m3joint_ms2_j4");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
   
   msg->name.push_back("m3joint_ms2_j5");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
   
   msg->name.push_back("m3joint_ms2_j6");
   msg->position.push_back(0.0);
   msg->velocity.push_back(0.0);
#endif   
   



  /*msg->name.push_back(meka->getGripperJointNames()[0]);
  msg->position.push_back(angles[5]);
  msg->velocity.push_back(vels[5]);

  msg->name.push_back(meka->getGripperJointNames()[1]);
  msg->position.push_back(angles[5]); // both right and left finger are controlled by motor 6
  msg->velocity.push_back(vels[5]);*/

  msg->header.stamp = ros::Time::now();
  pub.publish(msg); // NOTE: msg must not be changed after publishing; use reset() if necessary (http://www.ros.org/wiki/roscpp/Internals)
}

}
