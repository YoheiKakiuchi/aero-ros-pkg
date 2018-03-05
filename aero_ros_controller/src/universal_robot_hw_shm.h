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

#ifndef _UNIVERSAL_ROBOT_HW_SHM_H_
#define _UNIVERSAL_ROBOT_HW_SHM_H_

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>

// URDF
#include <urdf/model.h>

// shared memory
#include <aero_startup/controller_shm.h>

namespace universal_realtime_controller
{

class UniversalRobotHWShm : public hardware_interface::RobotHW
{
public:
  UniversalRobotHWShm() : shm_(NULL) {
    shm_ = (struct shm_struct *)set_shared_memory(5555, sizeof(struct shm_struct));
    if (shm_ == NULL) {
      ROS_ERROR( "shared memory initialize error" );
      exit(1);
    }
  }
  virtual ~UniversalRobotHWShm() {}

  /** \brief The init function is called to initialize the RobotHW from a
   * non-realtime thread.
   *
   * \param root_nh A NodeHandle in the root of the caller namespace.
   *
   * \param robot_hw_nh A NodeHandle in the namespace from which the RobotHW
   * should read its configuration.
   *
   * \returns True if initialization was successful
   */
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);


  /** \name Hardware Interface Switching
   *\{*/
  /**
   * Check (in non-realtime) if given controllers could be started and stopped from the current state of the RobotHW
   * with regard to necessary hardware interface switches and prepare the switching. Start and stop list are disjoint.
   * This handles the check and preparation, the actual switch is commited in doSwitch()
   */
  virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                             const std::list<hardware_interface::ControllerInfo>& stop_list) {
    // message
    return true;
  }

  /**
   * Perform (in realtime) all necessary hardware interface switches in order to start and stop the given controllers.
   * Start and stop list are disjoint. The feasability was checked in prepareSwitch() beforehand.
   */
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                        const std::list<hardware_interface::ControllerInfo>& stop_list) {
    // message
  }

  /**
   * Reads data from the robot HW
   *
   * \param time The current time
   * \param period The time passed since the last call to \ref read
   */
  virtual void read(const ros::Time& time, const ros::Duration& period);

  /**
   * Writes data to the robot HW
   *
   * \param time The current time
   * \param period The time passed since the last call to \ref write
   */
  virtual void write(const ros::Time& time, const ros::Duration& period);

protected:
  // Methods used to control a joint.
  enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};
  enum JointType {NONE, PRISMATIC, ROTATIONAL, CONTINUOUS, FIXED};

  struct shm_struct *shm_;

  unsigned int n_dof_;

  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  //hardware_interface::VelocityJointInterface vj_interface_;
  //hardware_interface::EffortJointInterface   ej_interface_;

#if 0
  joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
  joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;
  joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
#endif

  std::vector<std::string> joint_names_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_effort_limits_;
  std::vector<JointType> joint_types_;
  std::vector<ControlMethod> joint_control_methods_;
  // std::vector<control_toolbox::Pid> pid_controllers_;
  std::vector<double> joint_position_;
  std::vector<double> last_joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> last_joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;
};

typedef boost::shared_ptr<UniversalRobotHWShm> UniversalRobotHWShmPtr;
}

#endif // #ifndef _UNIVERSAL_ROBOT_HW_SHM_H_