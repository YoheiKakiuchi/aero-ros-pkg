#ifndef _IK_SETTINGS_H_
#define _IK_SETTINGS_H_

#define ENUM_TO_STRING(var) #var

#include <unordered_map>

namespace aero
{
  typedef Eigen::Affine3d      Transform;
  typedef Eigen::Vector3d      Vector3;
  typedef Eigen::Matrix3d      Matrix3;
  typedef Eigen::Translation3d Translation;
  typedef Eigen::Quaterniond   Quaternion;
  typedef Eigen::AngleAxisd    AngleAxis;
  typedef Eigen::Matrix<double, 6, 1> Vector6;

  enum struct arm : int {rarm, larm, either};

  enum struct ikrange : int {arm, torso, lifter};

  enum struct eef : int {hand, grasp, pick, index, thumb};

  enum struct joint : int {r_shoulder_p,
      r_shoulder_r,
      r_shoulder_y,
      r_elbow,
      r_wrist_y,
      r_wrist_p,
      r_wrist_r,
      r_hand_y,
      l_shoulder_p,
      l_shoulder_r,
      l_shoulder_y,
      l_elbow,
      l_wrist_y,
      l_wrist_p,
      l_wrist_r,
      l_hand_y,
      waist_y,
      waist_p,
      waist_r,
      lifter_x,
      lifter_z
      };

  const std::map<aero::joint, std::string> joint_map = {
    {aero::joint::r_shoulder_p,"r_shoulder_p_joint"},
    {aero::joint::r_shoulder_r,"r_shoulder_r_joint"},
    {aero::joint::r_shoulder_y,"r_shoulder_y_joint"},
    {aero::joint::r_elbow,"r_elbow_joint"},
    {aero::joint::r_wrist_y,"r_wrist_y_joint"},
    {aero::joint::r_wrist_p,"r_wrist_p_joint"},
    {aero::joint::r_wrist_r,"r_wrist_r_joint"},
    {aero::joint::r_hand_y,"r_hand_y_joint"},
    {aero::joint::l_shoulder_p,"l_shoulder_p_joint"},
    {aero::joint::l_shoulder_r,"l_shoulder_r_joint"},
    {aero::joint::l_shoulder_y,"l_shoulder_y_joint"},
    {aero::joint::l_elbow,"l_elbow_joint"},
    {aero::joint::l_wrist_y,"l_wrist_y_joint"},
    {aero::joint::l_wrist_p,"l_wrist_p_joint"},
    {aero::joint::l_wrist_r,"l_wrist_r_joint"},
    {aero::joint::l_hand_y,"l_hand_y_joint"},
    {aero::joint::waist_y,"waist_y_joint"},
    {aero::joint::waist_p,"waist_p_joint"},
    {aero::joint::waist_r,"waist_r_joint"},
    {aero::joint::lifter_x,"virtual_lifter_x_joint"},
    {aero::joint::lifter_z,"virtual_lifter_z_joint"}
  };

  const std::map<std::string, aero::joint> string_map = {
    {"r_shoulder_p_joint", aero::joint::r_shoulder_p},
    {"r_shoulder_r_joint", aero::joint::r_shoulder_r},
    {"r_shoulder_y_joint" ,aero::joint::r_shoulder_y},
    {"r_elbow_joint" ,aero::joint::r_elbow},
    {"r_wrist_y_joint" ,aero::joint::r_wrist_y},
    {"r_wrist_p_joint" ,aero::joint::r_wrist_p},
    {"r_wrist_r_joint" ,aero::joint::r_wrist_r},
    {"r_hand_y_joint" ,aero::joint::r_hand_y},
    {"l_shoulder_p_joint" ,aero::joint::l_shoulder_p},
    {"l_shoulder_r_joint" ,aero::joint::l_shoulder_r},
    {"l_shoulder_y_joint" ,aero::joint::l_shoulder_y},
    {"l_elbow_joint" ,aero::joint::l_elbow},
    {"l_wrist_y_joint" ,aero::joint::l_wrist_y},
    {"l_wrist_p_joint" ,aero::joint::l_wrist_p},
    {"l_wrist_r_joint" ,aero::joint::l_wrist_r},
    {"l_hand_y_joint" ,aero::joint::l_hand_y},
    {"waist_y_joint" ,aero::joint::waist_y},
    {"waist_p_joint" ,aero::joint::waist_p},
    {"waist_r_joint" ,aero::joint::waist_r},
    {"virtual_lifter_x_joint" ,aero::joint::lifter_x},
    {"virtual_lifter_z_joint" ,aero::joint::lifter_z}
  };

  inline std::string arm2LR(aero::arm _arm)
  {
    if (_arm == aero::arm::rarm) return "r";
    else if (_arm == aero::arm::larm) return "l";
    else return "";
  }

  inline std::string arm2LeftRight(aero::arm _arm)
  {
    if (_arm == aero::arm::rarm) return "right";
    else if (_arm == aero::arm::larm) return "left";
    else return "";
  }

  inline std::string arm2LarmRarm(aero::arm _arm)
  {
    if (_arm == aero::arm::rarm) return "rarm";
    else if (_arm == aero::arm::larm) return "larm";
    else return "";
  }

  inline std::string armAndRange2MoveGroup(aero::arm _arm, aero::ikrange _range)
  {
    std::string mg = arm2LarmRarm(_arm);
    if (_range == aero::ikrange::torso) mg = mg + "_with_torso";
    else if (_range == aero::ikrange::lifter) mg = mg + "_with_lifter";

    return mg;
  }

  inline std::string armAndEEF2LinkName(aero::arm _arm, aero::eef _eef)
  {
    std::string ln = arm2LR(_arm);
    if (_eef == aero::eef::hand) ln = ln + "_hand_link";
    else if (_eef == aero::eef::grasp) ln = ln + "_eef_grasp_link";
    else if (_eef == aero::eef::pick) ln = ln + "_eef_pick_link";
    else if (_eef == aero::eef::index) ln = ln + "_index_tip_link";
    else if (_eef == aero::eef::thumb) ln = ln + "_thumb_tip_link";
    return ln;
  }

  inline std::string joint2JointName(aero::joint _joint)
  {
    return aero::joint_map.at(_joint);
  }

  inline aero::joint jointName2Joint(std::string _joint_name)
  {
    return aero::string_map.at(_joint_name);
  }

  inline void jointMap2StringMap(std::map<aero::joint, double> &_j_map, std::map<std::string, double> &_s_map)
  {
    _s_map.clear();
    for(auto it = _j_map.begin(); it != _j_map.end(); ++it) {
      _s_map[joint2JointName(it->first)] = it->second;
    }
  }

  inline void stringMap2JointMap(std::map<std::string, double> &_s_map, std::map<aero::joint, double> &_j_map)
  {
    _j_map.clear();
    for(auto it = _s_map.begin(); it != _s_map.end(); ++it) {
      _j_map[jointName2Joint(it->first)] = it->second;
    }
  }

  struct fullarm {
    std::map<aero::joint, double> joints;
    double l_hand;
    double r_hand;
  };
}

#endif
