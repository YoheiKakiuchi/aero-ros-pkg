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

  robot_interface::joint_angle_map a_map;
  robot_interface::joint_angle_map b_map;
  a_map["r_shoulder_p_joint"] = -0.18800;
  a_map["r_shoulder_r_joint"] = -0.39268;
  a_map["r_shoulder_y_joint"] = 0.98168;
  a_map["r_elbow_joint"]      = -0.61062;
  a_map["r_wrist_y_joint"]    = 0.78535;
  a_map["r_wrist_p_joint"]    = 0.07500;
  a_map["r_wrist_r_joint"]    = 0.80722;
  a_map["r_hand_y_joint"]     = 1.00000;
  a_map["l_shoulder_p_joint"] = -0.18800;
  a_map["l_shoulder_r_joint"] = 1.17803;
  a_map["l_shoulder_y_joint"] = 0.19637;
  a_map["l_elbow_joint"]      = -0.61062;
  a_map["l_wrist_y_joint"]    = 0.78535;
  a_map["l_wrist_p_joint"]    = 0.07500;
  a_map["l_wrist_r_joint"]    = 0.19635;
  a_map["l_hand_y_joint"]     = 1.00000;
  a_map["waist_y_joint"]      = 1.00000;
  a_map["waist_p_joint"]      = 0.37083;
  a_map["waist_r_joint"]      = 0.00850;
  a_map["ankle_joint"]        = 1.17803;
  a_map["knee_joint"]         = -0.39268;
  a_map["neck_y_joint"]       = 1.05000;
  a_map["neck_p_joint"]       = 0.45815;
  a_map["neck_r_joint"]       = 0.06109;

  b_map["r_shoulder_p_joint"] = -1.08600;
  b_map["r_shoulder_r_joint"] = -1.17803;
  b_map["r_shoulder_y_joint"] = -0.19637;
  b_map["r_elbow_joint"]      = -1.83185;
  b_map["r_wrist_y_joint"]    = -0.78535;
  b_map["r_wrist_p_joint"]    = -0.07500;
  b_map["r_wrist_r_joint"]    = -0.19635;
  b_map["r_hand_y_joint"]     = -1.00000;
  b_map["l_shoulder_p_joint"] = -1.08600;
  b_map["l_shoulder_r_joint"] = 0.39268;
  b_map["l_shoulder_y_joint"] = -0.98168;
  b_map["l_elbow_joint"]      = -1.83185;
  b_map["l_wrist_y_joint"]    = -0.78535;
  b_map["l_wrist_p_joint"]    = -0.07500;
  b_map["l_wrist_r_joint"]    = -0.80722;
  b_map["l_hand_y_joint"]     = -1.00000;
  b_map["waist_y_joint"]      = -1.00000;
  b_map["waist_p_joint"]      = 0.06548;
  b_map["waist_r_joint"]      = -0.00850;
  b_map["ankle_joint"]        = 0.39268;
  b_map["knee_joint"]         = -1.17803;
  b_map["neck_y_joint"]       = -1.05000;
  b_map["neck_p_joint"]       = 0.15272;
  b_map["neck_r_joint"]       = -0.06109;

  robot_interface::angle_vector rav =
    {0.1, 0.2, 0.3,
     0.4, 0.5, 0.6, 0.7,
     0.8};

  ROS_INFO("send rarm angle-vector");
  ari.rarm->send_angle_vector(rav, 3.0);

  std::vector<double > lav = {0.1, 0.2, 0.3,
                             0.4, 0.5, 0.6, 0.7,
                             0.8};

  ROS_INFO("send larm angle-vector");
  ari.larm->send_angle_vector(lav, 5.0);
  ari.larm->wait_interpolation();
  ari.larm->wait_interpolation(1.0);

  // stop_motion
  {
    robot_interface::joint_angle_map _map;
    ari.larm->getReferenceVector(_map);
    //ari.larm->getActualVector(_map);
    Ros::Time start_ = ros::Time::now() + ros::Time(0.0);
    ari.larm->sendAngles(_map, 0.1, start_);
  }
  ari.larm->wait_interpolation();

  ////
  ROS_INFO("wait interpolation");
  bool ret = ari.wait_interpolation();
  ari.wait_interpolation("larm");
  ari.wait_interpolation("larm", 1.0);
  std::vector < std::string> groups = {"larm", "rarm", "waist"};
  ari.wait_interpolation(groups, 2.0);

  ari.wait_interpolation("both_arms", 2.0);

  ROS_INFO("ari.wait_interpolation() => %d", ret);

  robot_interface::joint_angle_map ref_map, act_map;
  ari.getReferencePositions(ref_map);
  ari.getActualPositions(ref_map);
  robot_interface::angle_vector ref_av, act_av;
  ari.reference_vector(ref_av);
  ari.reference_vector(act_av);

  ari.send_angle_vector();

  ari.send_angle_vector_sequence();

  ari.larm.send_angle_vector();

  ari.larm.send_angle_vector_sequence();

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
  // ros::spin();
}
