/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Yohei Kakiuchi (JSK lab.)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 Author: Yohei Kakiuchi
*/

#include "universal_robot_hw_shm.h"
#include <urdf/model.h>

// read from param
//   types
//   methods
//   limits(from URDF)
namespace universal_controller_shm
{

bool UniversalRobotHWShm::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)// add joint list
{
  { // wait launching raw controller
    shm_->frame = -1;
    while (shm_->frame < 0) {
      ROS_WARN("Waiting launching raw controller");
      ros::Duration tm(1);
      tm.sleep();
    }
    ROS_INFO("Detected running raw controller");
  }

  std::vector<std::string > names;
  if ( robot_hw_nh.hasParam("aero_joint_names") ) {
    ROS_INFO("Using joint names is configured from ros parameter %s/aero_joint_names",
             robot_hw_nh.getNamespace().c_str());
    robot_hw_nh.getParam("aero_joint_names", names);
  } else {
    ROS_ERROR("Parameter %s/aero_joint_names does not exist",
              robot_hw_nh.getNamespace().c_str());
    return false;
#if 0
    ROS_INFO("Using joint names is configured from shared memory");
    for(int i = 0; i < MAX_JOINT_NUM; i++) {
      char *nm = shm_->joint_names[i];
      if (nm[0] == '\0') {
        break;
      }
      names.push_back(std::string(nm));
    }
#endif
  }
  if (names.size() == 0) {
    ROS_ERROR("Parameter %s/aero_joint_names is empty", robot_hw_nh.getNamespace().c_str());
    return false;
  }

  std::string model_str;
  if (!robot_hw_nh.getParam("robot_description", model_str)) {
    ROS_ERROR("Failed to get model from robot_description");
    return false;
  }
  urdf::Model model;
  if (!model.initString(model_str)) {
    ROS_ERROR("Failed to parse robot_description");
    return false;
  }

  n_dof_ = names.size();

  joint_names_.resize(n_dof_);
  joint_types_.resize(n_dof_);
  joint_lower_limits_.resize(n_dof_);
  joint_upper_limits_.resize(n_dof_);
  joint_effort_limits_.resize(n_dof_);
  joint_control_methods_.resize(n_dof_);
  //pid_controllers_.resize(n_dof_);
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_effort_command_.resize(n_dof_);
  joint_position_command_.resize(n_dof_);
  joint_velocity_command_.resize(n_dof_);

  ROS_DEBUG("read %d joints", n_dof_);
  for (int i = 0; i < n_dof_; i++) {
    joint_names_[i] = names[i];
    ROS_DEBUG("  %d: %s", i, joint_names_[i].c_str());
  }
  for(unsigned int i = 0; i < n_dof_; i++) {
    if(!model.getJoint(joint_names_[i])) {
      ROS_ERROR("Joint %s does not exist in urdf model", joint_names_[i].c_str());
      return false;
    }
  }

  // Initialize values
  for(unsigned int j = 0; j < n_dof_; j++) {
    // Add data from transmission
    joint_position_[j] = shm_->act_angle[j];
    joint_position_command_[j] = shm_->act_angle[j];

    joint_velocity_[j] = 0.0;
    joint_velocity_command_[j] = 0.0;
    joint_effort_[j]   = 0.0;  // N/m for continuous joints
    joint_effort_command_[j]   = 0.0;
    std::string jointname = joint_names_[j];
    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
        jointname, &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

    joint_control_methods_[j] = POSITION;
    hardware_interface::JointHandle joint_handle =
      hardware_interface::JointHandle(js_interface_.getHandle(jointname),
                                      &joint_position_command_[j]);
    pj_interface_.registerHandle(joint_handle);

    joint_limits_interface::JointLimits limits;
    const bool urdf_limits_ok = joint_limits_interface::getJointLimits(model.getJoint(jointname), limits);
    if (!urdf_limits_ok) {
      ROS_WARN("urdf limits of joint %s is not defined", jointname.c_str());
    }
    // Register handle in joint limits interface
    joint_limits_interface::PositionJointSaturationHandle
      limits_handle(joint_handle, // We read the state and read/write the command
                    limits);       // Limits spec
    pj_sat_interface_.registerHandle(limits_handle);
  }

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&pj_interface_);
  //registerInterface(&vj_interface_);
  //registerInterface(&ej_interface_);

  return true;
}

void UniversalRobotHWShm::read(const ros::Time& time, const ros::Duration& period)
{
  double tm = period.toSec();
  for(unsigned int j=0; j < n_dof_; j++) {
    float position = shm_->act_angle[j];
    float velocity = shm_->act_velocity[j];

    if (joint_types_[j] == PRISMATIC) {
      joint_position_[j] = position;
    } else {
      joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
                                                              position);
    }
    joint_velocity_[j] = velocity; // read velocity from HW
    joint_effort_[j]   = 0; // read effort from HW
  }
}

void UniversalRobotHWShm::write(const ros::Time& time, const ros::Duration& period)
{
  pj_sat_interface_.enforceLimits(period);
#if 0
  ej_sat_interface_.enforceLimits(period);
  ej_limits_interface_.enforceLimits(period);
  pj_sat_interface_.enforceLimits(period);
  pj_limits_interface_.enforceLimits(period);
  vj_sat_interface_.enforceLimits(period);
  vj_limits_interface_.enforceLimits(period);
#endif
  for(unsigned int j=0; j < n_dof_; j++) {
    switch (joint_control_methods_[j]) {
    case POSITION:
      {
        // write position to hw
        shm_->ref_angle[j] = joint_position_command_[j];
      }
      break;
    case VELOCITY:
      {
      }
      break;
    case EFFORT:
      {
      }
      break;
    case POSITION_PID:
      {
      }
      break;
    case VELOCITY_PID:
      {
      }
      break;
    } // switch
  } // for
}

}

// PLUGINLIB_EXPORT_CLASS(universal_realtime_controller::UniversealRobotHWShm, gazebo_ros_control::RobotHWSim)
