#include <ros/ros.h>
#include <aero_ros_controller/RobotInterface.hh>

#include <Eigen/Eigen>

static std::ostream& operator<<(std::ostream& os, const Eigen::Affine3d &tr)
{
  Eigen::Vector3d tt = tr.translation();
  Eigen::Quaterniond qq(tr.linear());
  os << "(cons #f("
     << tt(0) << " "
     << tt(1) << " "
     << tt(2) << ")";
  os << " #f("
     << qq.w() << " "
     << qq.x() << " "
     << qq.y() << " "
     << qq.z() << "))";
  return os;
}

using namespace robot_interface;

//// TEST for aero TypeF
class AeroRobotInterface : public RobotInterface {

public:
  AeroRobotInterface(ros::NodeHandle &_nh) : RobotInterface(_nh) {
    // rarm
    rarm.reset(new TrajectoryClient(_nh,
                                    "rarm_controller/follow_joint_trajectory",
                                    "rarm_controller/state",
                                    { "r_shoulder_p_joint", "r_shoulder_r_joint", "r_shoulder_y_joint",
                                        "r_elbow_joint", "r_wrist_y_joint", "r_wrist_p_joint", "r_wrist_r_joint",
                                        "r_hand_y_joint" }
                                    ));
    this->add_controller("rarm", rarm);

    // larm
    larm.reset(new TrajectoryClient(_nh,
                                    "larm_controller/follow_joint_trajectory",
                                    "larm_controller/state",
                                    { "l_shoulder_p_joint", "l_shoulder_r_joint", "l_shoulder_y_joint",
                                        "l_elbow_joint", "l_wrist_y_joint", "l_wrist_p_joint", "l_wrist_r_joint",
                                        "l_hand_y_joint" }
                                    ));
    this->add_controller("larm", larm);

    // torso
    waist.reset(new TrajectoryClient(_nh,
                                     "waist_controller/follow_joint_trajectory",
                                     "waist_controller/state",
                                     { "waist_y_joint", "waist_p_joint", "waist_r_joint"}
                                     ));
    this->add_controller("waist",  waist);

    // lifter
    lifter.reset(new TrajectoryClient(_nh,
                                      "lifter_controller/follow_joint_trajectory",
                                      "lifter_controller/state",
                                      { "knee_joint", "ankle_joint" }
                                      ));
    this->add_controller("lifter", lifter);

    // head
    head.reset(new TrajectoryClient(_nh,
                                    "head_controller/follow_joint_trajectory",
                                    "head_controller/state",
                                    { "neck_y_joint", "neck_p_joint", "neck_r_joint"}
                                    ));
    this->add_controller("head",   head);

    // group settings
    controller_group_["both_arms"]  = {"rarm", "larm"};
    controller_group_["upper_body"] = {"rarm", "larm", "torso", "head"};
    controller_group_["torso"]      = {"waist", "lifter"};
  }
  using RobotInterface::wait_interpolation;
  virtual bool wait_interpolation(std::string &_name, double _tm = 0.0) {
    { // find interpolation group
      auto it = controller_group_.find(_name);
      if (it != controller_group_.end()) {
        return RobotInterface::wait_interpolation(it->second, _tm);
      }
    }
    return RobotInterface::wait_interpolation(_name, _tm);
  }

public:
  TrajectoryClient::Ptr larm;
  TrajectoryClient::Ptr rarm;
  TrajectoryClient::Ptr waist;
  TrajectoryClient::Ptr lifter;
  TrajectoryClient::Ptr head;
  // TrajectoryClient::Ptr l_hand;
  // TrajectoryClient::Ptr r_hand;
protected:
  std::map< std::string, std::vector<std::string > > controller_group_;
};

int main (int argc, char **argv) {
  ros::init(argc, argv, "robot_interface_test");
  ros::NodeHandle nh;

  ROS_INFO("Create Interface");
  AeroRobotInterface ari (nh);
  ROS_INFO("Create Interface: done");

  std::vector<double > rav = {0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0,
                             0.0};

  ROS_INFO("send rarm angle-vector");
  ari.rarm->send_angle_vector(rav, 3.0);

  std::vector<double > lav = {0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0,
                             0.0};

  ROS_INFO("send larm angle-vector");
  ari.larm->send_angle_vector(lav, 5.0);

  ////
  ROS_INFO("wait interpolation");
  bool ret = ari.wait_interpolation();

  ROS_INFO("ari.wait_interpolation() => %d", ret);

#if 0
  ros::Duration d(10);
  if( ari.rarm->waitForResult(d)) {
    ROS_INFO("success");
    actionlib::SimpleClientGoalState state = ari.rarm->getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  } else {
    ROS_INFO("fail");
  }
#endif
  ros::spin();
}
