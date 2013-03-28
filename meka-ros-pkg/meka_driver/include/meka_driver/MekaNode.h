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
 * KatanaNode.h
 *
 *  Created on: 11.12.2010
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#ifndef MEKANODE_H_
#define MEKANODE_H_

#include <ros/ros.h>

#include <meka_driver/JointStatePublisher.h>
#include <meka_driver/joint_trajectory_action_controller.h>
#include <meka_driver/joint_movement_action_controller.h>
//#include <meka_driver/katana_gripper_grasp_controller.h>

#include <meka_driver/AbstractMeka.h>
#include "meka_driver/Meka.h"
//#include <meka_driver/Katana300.h>
#include <meka_driver/SimulatedMeka.h>

namespace meka_driver
{

/**
 * @brief This is the node providing all publishers/services/actions relating to the Katana arm.
 *
 * It actually calls several other classes to do the real work.
 */
class MekaNode
{
public:
  MekaNode();
  virtual ~MekaNode();
  int loop();

private:
  boost::shared_ptr<meka_driver::AbstractMeka> meka;
  ros::NodeHandle nh;  // just to make sure that there is at least one node handle which doesn't go out of scope

};

}

#endif /* KATANANODE_H_ */
