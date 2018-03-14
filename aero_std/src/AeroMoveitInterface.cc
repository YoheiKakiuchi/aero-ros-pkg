#include "aero_std/AeroMoveitInterface.hh"

//////////////////////////////////////////////////
aero::interface::AeroMoveitInterface::AeroMoveitInterface(ros::NodeHandle _nh, std::string _rd)
{
  // publishers

  //// which node, it connected to
  look_at_publisher_rpy_ = _nh.advertise<geometry_msgs::Point>
    ("/look_at/rpy", 10);

  look_at_publisher_base_ = _nh.advertise<geometry_msgs::Point>
    ("/look_at/target", 10);

  look_at_publisher_map_ = _nh.advertise<geometry_msgs::Point>
    ("/look_at/target/map", 10);

  look_at_publisher_base_static_ = _nh.advertise<geometry_msgs::Point>
    ("/look_at/target/static", 10);

  look_at_publisher_map_static_ = _nh.advertise<geometry_msgs::Point>
    ("/look_at/target/static/map", 10);

  lookat_target_publisher_ = _nh.advertise<std_msgs::String>
    ("/look_at/set_target_topic", 10);

  //// ???
  overwrite_speed_publisher_ = _nh.advertise<std_msgs::Float32>
    ("/aero_controller/speed_overwrite", 10);

  // subscribers
#if 0
  joint_states_subscriber_ = _nh.subscribe // obsolete
    ("/joint_states", 1, &aero::interface::AeroMoveitInterface::JointStateCallback_, this);

  waist_service_ = _nh.serviceClient<aero_startup::AeroTorsoController> // obsolete
    ("/aero_torso_controller");
#endif
  // service clients

#if 0
  joint_states_client_ = _nh.serviceClient<aero_startup::AeroSendJoints> // obsolete
    ("/aero_controller/get_joints");

  interpolation_client_ = _nh.serviceClient<aero_startup::AeroInterpolation> // obsolete
    ("/aero_controller/interpolation");

  lifter_ik_service_ = _nh.serviceClient<aero_startup::AeroTorsoController> // obsolete
    ("/aero_torso_kinematics");

  send_angle_service_ = _nh.serviceClient<aero_startup::AeroSendJoints> // obsolete
    ("/aero_controller/send_joints");
#endif

  // load robot model
  ROS_INFO("start loading robot model");
  robot_model_loader_ = robot_model_loader::RobotModelLoader(_rd);
  kinematic_model = robot_model_loader_.getModel();
  kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();// set all joints to 0.0

#if 0
  ROS_INFO("start loading robot model for height only");
  robot_model_loader_ho_ = robot_model_loader::RobotModelLoader(_rd + "_height_only");
  kinematic_model_ho = robot_model_loader_ho_.getModel();
  kinematic_state_ho = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model_ho));
  kinematic_state_ho->setToDefaultValues();// set all joints to 0.0

  ROS_INFO("start loading robot model for on plane");
  robot_model_loader_op_ = robot_model_loader::RobotModelLoader(_rd + "_on_plane");
  kinematic_model_op = robot_model_loader_op_.getModel();
  kinematic_state_op = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model_op));
  kinematic_state_op->setToDefaultValues();// set all joints to 0.0
#endif
  // JointModelGroup
  ROS_INFO("start loading joint model groups");
  jmg_larm = kinematic_model->getJointModelGroup("larm");
  jmg_larm_with_waist = kinematic_model->getJointModelGroup("larm_with_waist");
  jmg_larm_with_lifter = kinematic_model->getJointModelGroup("larm_with_lifter");
  jmg_rarm = kinematic_model->getJointModelGroup("rarm");
  jmg_rarm_with_waist = kinematic_model->getJointModelGroup("rarm_with_waist");
  jmg_rarm_with_lifter = kinematic_model->getJointModelGroup("rarm_with_lifter");
  jmg_lifter = kinematic_model->getJointModelGroup("lifter");

  jmg_waist = kinematic_model->getJointModelGroup("waist");
  jmg_torso = kinematic_model->getJointModelGroup("torso");
  jmg_both_arms = kinematic_model->getJointModelGroup("both_arms");
  jmg_upper_body = kinematic_model->getJointModelGroup("upper_body");
  jmg_whole_body = kinematic_model->getJointModelGroup("whole_body");
  jmg_head = kinematic_model->getJointModelGroup("head");

  //variables
  tracking_mode_flag_ = false;

  wait_ = true;
  saved_wait_settings_ = true;

  so_update_ = false;
  so_factor_ = 1.0f;
  so_retime_scale_ = 1.0f;

  ROS_INFO("----------------------------------------");
  ROS_INFO("  AERO MOVEIT INTERFACE is initialized");
  ROS_INFO("----------------------------------------");

  // ri.reset(new AeroRobotInterface(_nh));
}

//////////////////////////////////////////////////
aero::interface::AeroMoveitInterface::~AeroMoveitInterface()
{
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setRobotStateVariables(std::vector<double> &_av)
{ // convert angle-vector to moveit_vector
  kinematic_state->setVariablePositions(_av);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setRobotStateVariables(std::map<std::string, double> &_map)
{
  kinematic_state->setVariablePositions(_map);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setRobotStateVariables(std::map<aero::joint, double> &_map)
{
  std::map<std::string, double> map;
  aero::jointMap2StringMap(_map, map);
  setRobotStateVariables(map);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setRobotStateToCurrentState()
{
#if 0
  // get current joint states
  aero_startup::AeroSendJoints srv;
  if(!joint_states_client_.call(srv)) { // TODO
    ROS_WARN("getting joint states service failed");
    return;
  }
  srv.response;

  // update upper body
  std::map<std::string, double> map;
  for (auto it = aero::string_map.begin(); it != aero::string_map.end(); ++it) {
    auto itr = std::find(srv.response.joint_names.begin(), srv.response.joint_names.end(), it->first);
    if (itr == srv.response.joint_names.end()) continue;
    map[it->first] = srv.response.points.positions[static_cast<int>(itr - srv.response.joint_names.begin())];
  }

  // update hands
  std::vector<std::string> hand_joints = {"r_thumb_joint","r_indexbase_joint","r_indexmid_joint","r_indexend_joint", "l_thumb_joint","l_indexbase_joint","l_indexmid_joint","l_indexend_joint"};
  for(std::string jn: hand_joints) {
    auto hitr = std::find(srv.response.joint_names.begin(), srv.response.joint_names.end(), jn);
    if(hitr != srv.response.joint_names.end()) map[jn] = srv.response.points.positions[static_cast<int>(hitr - srv.response.joint_names.begin())];
  }

  kinematic_state->setVariablePositions(map);

  // update lifter
  auto hip_itr = std::find(srv.response.joint_names.begin(), srv.response.joint_names.end(), "hip_joint");
  auto knee_itr = std::find(srv.response.joint_names.begin(), srv.response.joint_names.end(), "knee_joint");

  double hip = srv.response.points.positions[static_cast<int>(hip_itr - srv.response.joint_names.begin())];
  double knee = srv.response.points.positions[static_cast<int>(knee_itr - srv.response.joint_names.begin())];
  double x = -lifter_foreleg_link_ * sin(knee - hip)
    + lifter_thigh_link_ * sin(hip);
  double z = lifter_foreleg_link_ * (cos(knee - hip) - 1.0)
    + lifter_thigh_link_ * (cos(hip) - 1.0);
  setLifter(x, z);

  // update necks
  std::vector<std::string> neck_joints = {"neck_r_joint", "neck_p_joint", "neck_y_joint"};
  for(std::string jn: neck_joints) {
    auto hitr = std::find(srv.response.joint_names.begin(), srv.response.joint_names.end(), jn);
    if(hitr != srv.response.joint_names.end()) map[jn] = srv.response.points.positions[static_cast<int>(hitr - srv.response.joint_names.begin())];
  }
#endif
  // TODO for hand ???
  robot_interface::joint_angle_map map;
  ri->getActualPositions(map);
  kinematic_state->setVariablePositions(map);

  updateLinkTransforms();
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setRobotStateToNamedTarget(std::string _move_group, std::string _target)
{
  //kinematic_state->setVariablePositions(getMoveGroup(_move_group).getNamedTargetValues(_target));
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::setFromIK(std::string _move_group, const aero::EigenTrans &_pose, std::string _eef_link, int _attempts) {
  const robot_state::JointModelGroup* jmg_tmp;
  bool lifter_ik = false;

  if (_move_group == "larm") {
    jmg_tmp = jmg_larm;
  } else if (_move_group == "larm_with_waist") {
    jmg_tmp = jmg_larm_with_waist;
  } else if (_move_group == "larm_with_lifter") {
    jmg_tmp = jmg_larm_with_lifter;
  } else if (_move_group == "rarm") {
    jmg_tmp = jmg_rarm;
  } else if (_move_group == "rarm_with_waist") {
    jmg_tmp = jmg_rarm_with_waist;
  } else if (_move_group == "rarm_with_lifter") {
    jmg_tmp = jmg_rarm_with_lifter;
  } else {
    ROS_WARN("IK error :: move_group [%s] doesn't exist", _move_group.c_str());
    return false;
  }

  bool found_ik;
  if (_eef_link == "") {
    found_ik = kinematic_state->setFromIK(jmg_tmp, _pose, _attempts, 0.1);
  } else {
    found_ik = kinematic_state->setFromIK(jmg_tmp, _pose, _eef_link, _attempts, 0.1);
  }
  //if (found_ik) getMoveGroup(_move_group).setJointValueTarget(*kinematic_state);
  return found_ik;
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::setFromIK(aero::arm _arm, aero::ikrange _range, const aero::EigenTrans &_pose, std::string _eef_link, int _attempts)
{
  return setFromIK(aero::armAndRange2MoveGroup(_arm, _range), _pose, _eef_link, _attempts);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::setFromIK(aero::arm _arm, aero::ikrange _range, const aero::EigenTrans &_pose, aero::eef _eef, int _attempts)
{
  return setFromIK(_arm, _range, _pose, armAndEEF2LinkName(_arm, _eef), _attempts);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::setFromIK(std::string _move_group, geometry_msgs::Pose _pose, std::string _eef_link, int _attempts)
{
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  aero::EigenTrans epose;
  tf::poseMsgToEigen(_pose, epose);
  return setFromIK(_move_group, epose, _eef_link, _attempts);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::setFromIK(aero::arm _arm, aero::ikrange _range, geometry_msgs::Pose _pose, std::string _eef_link, int _attempts)
{
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  return setFromIK(aero::armAndRange2MoveGroup(_arm, _range), _pose, _eef_link, _attempts);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::setFromIK(aero::arm _arm, aero::ikrange _range, geometry_msgs::Pose _pose, aero::eef _eef, int _attempts)
{
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  return setFromIK(_arm, _range, _pose, armAndEEF2LinkName(_arm, _eef), _attempts);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::setFromIK(std::string _move_group, Eigen::Vector3d _pos, Eigen::Quaterniond _qua, std::string _eef_link, int _attempts) {
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  aero::EigenTrans epose = EigenTranslation(_pos) * _qua;
  return setFromIK(_move_group, epose, _eef_link, _attempts);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::setFromIK(aero::arm _arm, aero::ikrange _range, Eigen::Vector3d _pos, Eigen::Quaterniond _qua, std::string _eef_link, int _attempts) {
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  return setFromIK(_arm, _range, _pos, _qua, _eef_link, _attempts);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::setFromIK(aero::arm _arm, aero::ikrange _range, Eigen::Vector3d _pos, Eigen::Quaterniond _qua, aero::eef _eef, int _attempts) {
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  return setFromIK(_arm, _range, _pos, _qua, _eef, _attempts);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::setLifter(double _x, double _z, bool _check_lifter_ik)
{
  std::vector<double> ans_xz;
#if 0
  if (!lifter_ik_(_x, _z, ans_xz)) {
    return false;
  }

  std::map<std::string, double> map = {
    {"virtual_lifter_x_joint", _x},
    {"virtual_lifter_z_joint", _z},
  };

  kinematic_state->setVariablePositions(map);
  return true;
#endif
  return lifter_ik_(_x, _z, ans_xz);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::lifter_ik_(double _x, double _z, std::vector<double>& _ans_xz)
{
  aero::EigenTrans initial_trans = EigenTranslation(0, 0, 0.725) * EigenQuaternion::Identity(); // typeB_lifter
  aero::EigenTrans diff_trans = EigenTranslation(_x, 0, - _z) * EigenQuaternion::Identity();
  aero::EigenTrans base2top = initial_trans * diff_trans;

  const aero::EigenTrans &base_trans = kinematic_state->getGlobalLinkTransform("lifter_base_link");
  aero::EigenTrans _pose = base_trans * base2top;

  int _attempts = 3;
  bool found_ik = kinematic_state->setFromIK(jmg_lifter, _pose, _attempts, 0.1);

  _ans_xz.resize(2);
  _ans_xz[0] = _x;
  _ans_xz[0] = _z;

  return found_ik;
#if 0
  aero_startup::AeroTorsoController srv;
  srv.request.x = static_cast<int>(_x * 1000);
  srv.request.z = static_cast<int>(_z * 1000);
  if (!lifter_ik_service_.call(srv)) {
    ROS_ERROR("lifter ik failed service call");
    return false;
  }

  if (srv.response.status == "success") {
    _ans_xz.reserve(2);
    _ans_xz[0] = srv.response.x;
    _ans_xz[1] = srv.response.z;
    return true;
  }
  return false;
#endif
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setLookAt(double _x, double _y, double _z, bool _map_coordinate, bool _tracking)
{
  ROS_INFO("setTrackingMode is %d in setLookAt, looking for %f %f %f in %s",
           static_cast<int>(tracking_mode_flag_), _x, _y, _z,
           (_map_coordinate ? "map" : "base"));

  if (tracking_mode_flag_) {
    geometry_msgs::Point msg;
    msg.x = _x;
    msg.y = _y;
    msg.z = _z;

    if (_map_coordinate) {
      if (_tracking) {
        previous_topic_ = "/look_at/target/map:"
          + std::to_string(_x) + "," + std::to_string(_y) + "," + std::to_string(_z);
        look_at_publisher_map_.publish(msg);
      } else {
        look_at_publisher_map_static_.publish(msg);
      }
    } else {
      if (_tracking) {
        previous_topic_ = "/look_at/target:"
          + std::to_string(_x) + "," + std::to_string(_y) + "," + std::to_string(_z);
        look_at_publisher_base_.publish(msg);
      } else {
        look_at_publisher_base_static_.publish(msg);
      }
    }
  } else {
    lookAt_(_x, _y, _z);
  }
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setLookAt(Eigen::Vector3d _target, bool _map_coordinate, bool _tracking)
{
  setLookAt(_target.x(), _target.y(), _target.z(), _map_coordinate, _tracking);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setLookAt(Eigen::Vector3f _target, bool _map_coordinate, bool _tracking)
{
  setLookAt(static_cast<double>(_target.x()), static_cast<double>(_target.y()), static_cast<double>(_target.z()), _map_coordinate, _tracking);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setLookAt(geometry_msgs::Pose _pose, bool _map_coordinate, bool _tracking)
{
  setLookAt(_pose.position.x, _pose.position.y, _pose.position.z, _map_coordinate, _tracking);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::resetLookAt()
{
  setNeck(0.0, 0.0, 0.0);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setNeck(double _r,double _p, double _y, bool _to_node)
{
  if (tracking_mode_flag_) {
    if (_to_node) {
      ROS_WARN("setNeck called in tracking mode! updating and sending model in node!");
      geometry_msgs::Point p;
      p.x = _r; p.y = _p; p.z = _y;
      look_at_publisher_rpy_.publish(p);
      return;
    } else {
      ROS_WARN("setNeck called in tracking mode! are you sure of what you are doing?");
    }
  }

  kinematic_state->setVariablePosition("neck_r_joint", _r);
  kinematic_state->setVariablePosition("neck_p_joint", _p);
  kinematic_state->setVariablePosition("neck_y_joint", _y);

  kinematic_state->enforceBounds( kinematic_model->getJointModelGroup("head"));
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendNeckAsync(int _time_ms)
{
  if (tracking_mode_flag_) {
    ROS_WARN("sendNeckAsync called in tracking mode! are you sure of what you are doing?");
  }

  trajectory_msgs::JointTrajectory msg;
  msg.points.resize(1);
  msg.joint_names = {"neck_r_joint", "neck_p_joint", "neck_y_joint"};
  msg.points[0].positions = {kinematic_state->getVariablePosition("neck_r_joint"), kinematic_state->getVariablePosition("neck_p_joint"), kinematic_state->getVariablePosition("neck_y_joint")};
  msg.points[0].time_from_start = ros::Duration(_time_ms * 0.001);
  // TODO
  //angle_vector_publisher_.publish(msg);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setLookAtTopic(std::string _topic, bool _record_topic)
{
  if (!tracking_mode_flag_) {
    ROS_WARN("must call setTrackingMode to true first for setLookAtTopic");
    return;
  }

  if (_record_topic) // usually false
    previous_topic_ = _topic;

  std_msgs::String msg;
  msg.data = _topic;

  if (_topic == "") {
    msg.data = "/look_at/manager_disabled";
    lookat_target_publisher_.publish(msg);
    lookat_topic_ = msg.data;
    previous_topic_ = "/look_at/manager_disabled";
    return;
  } else if (_topic == "/look_at/manager_disabled") {
    ROS_WARN("note, /look_at/manager_disabled only valid from prev");
    ROS_WARN("please make sure to call setTrackingMode false");
    aero_startup::AeroSendJoints srv;
    if (!get_saved_neck_positions_.call(srv)) {
      ROS_WARN("failed to get saved neck positions.");
    } else {
      // neck value set and send through node
      setNeck(srv.response.points.positions.at(0),
              srv.response.points.positions.at(1),
              srv.response.points.positions.at(2), true);
    }
    msg.data = "/look_at/manager_disabled";
    lookat_target_publisher_.publish(msg);
    lookat_topic_ = msg.data;
    previous_topic_ = "/look_at/manager_disabled";
    return;
  } else if (_topic == "/look_at/previous") {
    if (previous_topic_.find("/look_at/target") != std::string::npos) {
      auto pos = previous_topic_.find(":");
      std::string values = previous_topic_.substr(pos+1);
      auto posx = values.find(",");
      auto posy = values.find(",", posx+1);
      geometry_msgs::Point msg;
      msg.x = std::stof(values.substr(0, posx));
      msg.y = std::stof(values.substr(posx + 1, posy - posx -1));
      msg.z = std::stof(values.substr(posy + 1));
      if (previous_topic_.find("map") != std::string::npos)
        look_at_publisher_map_.publish(msg);
      else
        look_at_publisher_base_.publish(msg);
    } else {
      setLookAtTopic(previous_topic_);
    }
    return;
  }

  lookat_target_publisher_.publish(msg);
  lookat_topic_ = _topic;
}

//////////////////////////////////////////////////
std::string aero::interface::AeroMoveitInterface::getLookAtTopic()
{
  return lookat_topic_;
}

//////////////////////////////////////////////////
Eigen::Vector3d aero::interface::AeroMoveitInterface::volatileTransformToBase(double _x, double _y, double _z) {
  // TODO
  //geometry_msgs::Pose map2base = getCurrentPose();
  geometry_msgs::Pose map2base;
  Eigen::Vector3d map2base_p(map2base.position.x,
                             map2base.position.y,
                             map2base.position.z);
  Eigen::Quaterniond map2base_q(map2base.orientation.w,
                                map2base.orientation.x,
                                map2base.orientation.y,
                                map2base.orientation.z);
  // convert to map coordinates
  return map2base_q.inverse() * (Eigen::Vector3d(_x, _y, _z) - map2base_p);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::lookAt_(double _x ,double _y, double _z)
{
  Eigen::Vector3d obj;
  obj.x() = _x;
  obj.y() = _y;
  obj.z() = _z;
  double neck2eye = 0.2;
  double body2neck = 0.35;

  // get base position in robot coords
  updateLinkTransforms();
  std::string body_link = "body_link";
  Eigen::Vector3d base2body_p = kinematic_state->getGlobalLinkTransform(body_link).translation();
  Eigen::Matrix3d base2body_mat = kinematic_state->getGlobalLinkTransform(body_link).rotation();
  Eigen::Quaterniond base2body_q(base2body_mat);

  Eigen::Vector3d pos_obj_rel = base2body_q.inverse() * (obj - base2body_p) - Eigen::Vector3d(0.0, 0.0, body2neck);

  double yaw = atan2(pos_obj_rel.y(), pos_obj_rel.x());
  double dis_obj = sqrt(pos_obj_rel.x() * pos_obj_rel.x()
                        + pos_obj_rel.y() * pos_obj_rel.y()
                        + pos_obj_rel.z() * pos_obj_rel.z());
  double theta = acos(neck2eye / dis_obj);
  double pitch_obj = atan2(- pos_obj_rel.z(), pos_obj_rel.x());
  double pitch = 1.5708 + pitch_obj - theta;

  setNeck(0.0, pitch, yaw);
}

/////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::updateLinkTransforms()
{
  kinematic_state->updateLinkTransforms();
}

/////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::setInterpolation(int _i_type)
{
#if 0
  aero_startup::AeroInterpolation srv;
  srv.request.type.push_back(_i_type);

  if (!interpolation_client_.call(srv)) {
    ROS_WARN("interpolation service call failed");
    return false;
  }
#endif
  return true;
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setTrackingMode(bool _yes)
{
  // std_srvs::SetBool req;
  // req.request.data = _yes;
  // if (activate_tracking_client_.call(req)) tracking_mode_flag_ = _yes;
  if (!_yes) {
    ROS_WARN("disabling tracking mode!");
    wait_ = saved_wait_settings_;
    setLookAtTopic(""); // disable tracking
  } else {
    ROS_WARN("waitInterpolation disabled from setTrackingMode!");
    saved_wait_settings_ = wait_;
    wait_ = false;
  }
  tracking_mode_flag_ = _yes;
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::getRobotStateVariables(std::vector<double> &_av)
{
  // TODO: angle-vector
  double* tmp;
  int num = static_cast<int>(kinematic_model->getVariableCount());
  tmp = kinematic_state->getVariablePositions();
  _av.clear();
  _av.reserve(num);
  _av.assign(tmp, tmp + num);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::getRobotStateVariables(std::map<std::string, double> &_map)
{
  _map.clear();
  std::vector<std::string> upper_names, lifter_names;
  //upper_names = getMoveGroup("upper_body").getJointNames();
  //lifter_names = getMoveGroup("lifter").getJointNames();
  upper_names  = jmg_upper_body->getVariableNames();
  lifter_names = jmg_lifter    ->getVariableNames();

  std::vector<std::string> names;
  names.reserve(upper_names.size() + lifter_names.size());
  std::copy(upper_names.begin(),  upper_names.end(),  std::back_inserter(names));
  std::copy(lifter_names.begin(), lifter_names.end(), std::back_inserter(names));

  for (int i =0; i < names.size(); i++) {
    _map[names[i]] = kinematic_state->getVariablePosition(i);
  }
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::getRobotStateVariables(std::map<aero::joint, double> &_map)
{
  std::map<std::string, double> map_tmp;
  getRobotStateVariables(map_tmp);
  aero::stringMap2JointMap(map_tmp, _map);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::getRobotStateVariables(aero::fullarm &_map)
{
  std::map<std::string, double> map_tmp;
  getRobotStateVariables(map_tmp);
  aero::stringMap2JointMap(map_tmp, _map.joints);
  _map.l_hand = getHand(aero::arm::larm);
  _map.r_hand = getHand(aero::arm::rarm);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::getResetManipPose(std::map<aero::joint, double> &_map)
{
  std::map<aero::joint, double> save;
  getRobotStateVariables(save);
  setRobotStateToNamedTarget("upper_body", "reset-pose");
  getRobotStateVariables(_map);
  setRobotStateVariables(save);
}

//////////////////////////////////////////////////
aero::EigenVec3 aero::interface::AeroMoveitInterface::getWaistPosition()
{
  updateLinkTransforms();
  std::string link = "waist_link";
  aero::EigenVec3 vec = kinematic_state->getGlobalLinkTransform(link).translation();
  return vec;
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::getLifter(std::map<aero::joint, double>& _xz)
{
  // TODO ---
#if 0
  std::vector<double> tmp;
  kinematic_state->copyJointGroupPositions(jmg_lifter, tmp);
  _xz[aero::joint::lifter_x] = tmp[0];
  _xz[aero::joint::lifter_z] = tmp[1];
#endif
}

//////////////////////////////////////////////////
double aero::interface::AeroMoveitInterface::getHand(aero::arm _arm)
{
  std::string rl;
  if (_arm == aero::arm::rarm) rl = "r";
  else rl = "l";
  return kinematic_state->getVariablePosition(rl + "_thumb_joint");
}

/////////////////////////////////////////////////
Eigen::Vector3d aero::interface::AeroMoveitInterface::getEEFPosition(aero::arm _arm, aero::eef _eef)
{
  updateLinkTransforms();
  std::string link = aero::armAndEEF2LinkName(_arm, _eef);
  Eigen::Vector3d vec = kinematic_state->getGlobalLinkTransform(link).translation();
  return vec;
}

/////////////////////////////////////////////////
Eigen::Quaterniond aero::interface::AeroMoveitInterface::getEEFOrientation(aero::arm _arm, aero::eef _eef)
{
  updateLinkTransforms();
  std::string link = aero::armAndEEF2LinkName(_arm, _eef);
  Eigen::Matrix3d mat = kinematic_state->getGlobalLinkTransform(link).rotation();
  Eigen::Quaterniond vec(mat);
  return vec;
}

/////////////////////////////////////////////////
const robot_state::JointModelGroup* aero::interface::AeroMoveitInterface::getJointModelGroup(std::string _move_group){
  if (_move_group == "larm") {
    return this->jmg_larm;
  } else if (_move_group == "larm_with_waist") {
    return this->jmg_larm_with_waist;
  } else if (_move_group == "larm_with_lifter") {
    return this->jmg_larm_with_lifter;
  } else if (_move_group == "rarm") {
    return this->jmg_rarm;
  } else if (_move_group == "rarm_with_waist") {
    return this->jmg_rarm_with_waist;
  } else if (_move_group == "rarm_with_lifter") {
    return this->jmg_rarm_with_lifter;
  } else if (_move_group == "upper_body") {
    return this->jmg_upper_body;
  } else if (_move_group == "head") {
    return this->jmg_head;
  } else if(_move_group == "waist") {
    return this->jmg_waist;
  } else if(_move_group == "torso") {
    return this->jmg_torso;
  } else if (_move_group == "lifter") {
    return this->jmg_lifter;
  } else {
    ROS_WARN("error :: move_group [%s] doesn't exist", _move_group.c_str());
    ros::shutdown();
  }
}
/////////////////////////////////////////////////
const robot_state::JointModelGroup* aero::interface::AeroMoveitInterface::getJointModelGroup(aero::arm _arm, aero::ikrange _range)
{
  std::string gname =  aero::armAndRange2MoveGroup(_arm, _range);

  return getJointModelGroup(gname);
}
/////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendResetManipPose(int _time_ms)
{
  setRobotStateToNamedTarget("upper_body", "reset-pose");

  std::vector<std::string> j_names;
  //j_names = getMoveGroup("upper_body").getJointNames();
  j_names = jmg_upper_body->getVariableNames();

  std::vector<double> av_mg;
  kinematic_state->copyJointGroupPositions("upper_body", av_mg);

#if 0 // TODO
  aero_startup::AeroSendJoints srv;
  srv.request.joint_names = j_names;
  srv.request.points.positions.resize(j_names.size());
  srv.request.points.positions = av_mg;
  srv.request.points.time_from_start = ros::Duration(_time_ms * 0.001);

  if (!tracking_mode_flag_) {
    srv.request.points.positions.push_back(0.0);
    srv.request.joint_names.push_back("neck_r_joint");

    srv.request.points.positions.push_back(0.0);
    srv.request.joint_names.push_back("neck_p_joint");

    srv.request.points.positions.push_back(0.0);
    srv.request.joint_names.push_back("neck_y_joint");
  } else {
    ROS_WARN("tracking mode is on, not sending neck!");
  }

  if (!send_angle_service_.call(srv)) {
    ROS_ERROR("sendJoints failed service call");
    return;
  }

  usleep(_time_ms * 1000);
#endif
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendAngleVectorSync_(int _time_ms)
{ // TODO inline
  if (wait_) {
    usleep(static_cast<int>(_time_ms * 0.8) * 1000);// wait 80 percent
    waitInterpolation();
  } else {
    sleepInterpolation(_time_ms);
  }
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendAngleVector(aero::arm _arm, aero::ikrange _range, int _time_ms, bool async)
{
  sendAngleVectorAsync_(_arm, _range, _time_ms);
  if (!async) sendAngleVectorSync_(_time_ms);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendAngleVector(int _time_ms, aero::ikrange _move_waist, bool async)
{
  sendAngleVectorAsync_(_time_ms, _move_waist);
  if (!async) sendAngleVectorSync_(_time_ms);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendAngleVectorAsync_(aero::arm _arm, aero::ikrange _range, int _time_ms)
{
  std::string group = aero::armAndRange2MoveGroup(_arm, _range);
  std::vector<double> av_mg;
  kinematic_state->copyJointGroupPositions(group, av_mg);
  std::vector<std::string> j_names;
  j_names = getJointModelGroup(group)->getVariableNames();

  sendAngleVectorAsync_(av_mg, j_names, _time_ms);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendAngleVectorAsync_(int _time_ms, aero::ikrange _move_lifter)
{
  std::vector<double> av_mg;
  kinematic_state->copyJointGroupPositions("upper_body", av_mg);
  std::vector<std::string> j_names;
  j_names = jmg_upper_body->getVariableNames();

  if (_move_lifter == aero::ikrange::lifter) {
    std::vector<double> extra_av_mg;
    kinematic_state->copyJointGroupPositions("lifter", av_mg);
    std::vector<std::string> extra_j_names;
    j_names = jmg_lifter->getVariableNames();
    std::copy(extra_av_mg.begin(),   extra_av_mg.end(),   std::back_inserter(av_mg));
    std::copy(extra_j_names.begin(), extra_j_names.end(), std::back_inserter(j_names));
  }

  sendAngleVectorAsync_(av_mg, j_names, _time_ms);
}


//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendAngleVector(std::map<aero::joint, double> _av_map, int _time_ms, aero::ikrange _move_waist)
{
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  sendAngleVectorAsync(_av_map, _time_ms, _move_waist);
  sendAngleVectorSync_(_time_ms);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendAngleVector(aero::fullarm _av_map, int _time_ms, aero::ikrange _move_waist)
{
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  sendAngleVectorAsync(_av_map, _time_ms, _move_waist);
  sendAngleVectorSync_(_time_ms);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendAngleVectorAsync(aero::arm _arm, aero::ikrange _range, int _time_ms)
{
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  sendAngleVectorAsync_(_arm, _range, _time_ms);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendAngleVectorAsync(int _time_ms, aero::ikrange _move_lifter)
{
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  sendAngleVectorAsync_(_time_ms, _move_lifter);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendAngleVectorAsync(std::map<aero::joint, double> _av_map, int _time_ms, aero::ikrange _move_waist)
{
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  setRobotStateVariables(_av_map);
  sendAngleVectorAsync(_time_ms, _move_waist);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendAngleVectorAsync(aero::fullarm _av_map, int _time_ms, aero::ikrange _move_waist)
{
  // TODO
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  setRobotStateVariables(_av_map.joints);
  //setHand(aero::arm::larm, _av_map.l_hand);
  //setHand(aero::arm::rarm, _av_map.r_hand);
  // add upper body
  std::vector<double> av_mg;
  kinematic_state->copyJointGroupPositions("upper_body", av_mg);
  std::vector<std::string> j_names;
  //j_names = getMoveGroup("upper_body").getJointNames();
  j_names = jmg_upper_body->getVariableNames();

  // add hands
  av_mg.push_back(_av_map.l_hand);
  av_mg.push_back(_av_map.r_hand);
  j_names.push_back("l_thumb_joint");
  j_names.push_back("r_thumb_joint");
  // add lifter
  if (_move_waist == aero::ikrange::lifter) {
    std::map<aero::joint, double> av_lif;
    getLifter(av_lif);
    av_mg.push_back(av_lif[aero::joint::lifter_x]);
    av_mg.push_back(av_lif[aero::joint::lifter_z]);
    std::vector<std::string> j_lif{"virtual_lifter_x_joint", "virtual_lifter_z_joint"};
    j_names.push_back(j_lif[0]);
    j_names.push_back(j_lif[1]);
  }
  sendAngleVectorAsync_(av_mg, j_names, _time_ms);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendAngleVectorAsync_(std::vector<double> _av, std::vector<std::string> _joint_names, int _time_ms)
{
#if 0
  trajectory_msgs::JointTrajectory msg;
  msg.points.resize(1);
  msg.joint_names.resize(_av.size());
  msg.joint_names = _joint_names;
  msg.points[0].positions.resize(_av.size());
  msg.points[0].positions = _av;
  msg.points[0].time_from_start = ros::Duration(_time_ms * 0.001);


  // lifter
  auto itr_x = std::find(_joint_names.begin(), _joint_names.end(), "virtual_lifter_x_joint");
  auto itr_z = std::find(_joint_names.begin(), _joint_names.end(), "virtual_lifter_z_joint");

  size_t index_x = std::distance(_joint_names.begin(), itr_x);
  size_t index_z = std::distance(_joint_names.begin(), itr_z);

  if (index_x != _joint_names.size()) {
    // lifter ik check
    std::vector<double> res;
    bool result = lifter_ik_(_av[static_cast<int>(index_x)], _av[static_cast<int>(index_z)], res);
    if (!result) {
      ROS_WARN("lifter IK couldnt be solved  x:%f  z:%f",
               _av[static_cast<int>(index_x)], _av[static_cast<int>(index_z)]);
      return;
    }
    msg.joint_names[static_cast<int>(index_x)] = "hip_joint";
    msg.joint_names[static_cast<int>(index_z)] = "knee_joint";
    msg.points[0].positions[static_cast<int>(index_x)] = res[0];
    msg.points[0].positions[static_cast<int>(index_z)] = res[1];
  }

  if (!tracking_mode_flag_) {
    msg.points[0].positions.push_back(kinematic_state->getVariablePosition("neck_r_joint"));
    msg.joint_names.push_back("neck_r_joint");

    msg.points[0].positions.push_back(kinematic_state->getVariablePosition("neck_p_joint"));
    msg.joint_names.push_back("neck_p_joint");

    msg.points[0].positions.push_back(kinematic_state->getVariablePosition("neck_y_joint"));
    msg.joint_names.push_back("neck_y_joint");
  } else {
    ROS_WARN("tracking mode is on, not sending neck!");
  }

  angle_vector_publisher_.publish(msg);
#endif
  // TODO for tracking mode
  ros::Time start_time = ros::Time::now() + ros::Duration(0.05); // starting 0.05sec after now
  ri->sendAngles(_joint_names, _av, _time_ms*0.001, start_time);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendTrajectory(aero::trajectory _trajectory, std::vector<int> _times, aero::ikrange _move_lifter)
{
  if (!sendTrajectoryAsync(_trajectory, _times, _move_lifter)) return false;
  int time = std::accumulate(_times.begin(), _times.end(), 0);
  sendAngleVectorSync_(time);

  return true;
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendTrajectory(aero::trajectory _trajectory, int _time_ms, aero::ikrange _move_lifter)
{
  if (!sendTrajectoryAsync(_trajectory, _time_ms, _move_lifter)) return false;
  sendAngleVectorSync_(_time_ms);

  return true;
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendTrajectoryAsync(aero::trajectory _trajectory, std::vector<int> _times, aero::ikrange _move_lifter)
{
  if (_trajectory.size() != _times.size()) {
    ROS_WARN("trajectory length[%d] doesnt match with times length[%d]",
             static_cast<int>(_trajectory.size()), static_cast<int>(_times.size()));
    return false;
  }
  std::vector<std::vector<double>> xzs;
#if 0
  if (_move_lifter == aero::ikrange::lifter) {// lifter ik check
    xzs.reserve(static_cast<int>(_trajectory.size()));
    for (auto point : _trajectory) {
      setRobotStateVariables(point);
      std::map<aero::joint, double> lif;
      getLifter(lif);
      std::vector<double> xz;
      if (!lifter_ik_(lif[aero::joint::lifter_x], lif[aero::joint::lifter_z], xz)) {
        ROS_WARN("lifter_ik failed");
        return false;
      }
      xzs.push_back(std::vector<double>{xz[0], xz[1]});
    }
  }
#endif
  //get trajectory
  std::vector<std::vector<double>> tra;
  tra.reserve(_trajectory.size());
  for (auto point : _trajectory) {
    setRobotStateVariables(point);
    std::vector<double> av;
    kinematic_state->copyJointGroupPositions("upper_body", av);
    if (!tracking_mode_flag_) {
      av.push_back(kinematic_state->getVariablePosition("neck_r_joint"));
      av.push_back(kinematic_state->getVariablePosition("neck_p_joint"));
      av.push_back(kinematic_state->getVariablePosition("neck_y_joint"));
    }
    tra.push_back(av);
  }

  //get joint names
  std::vector<std::string> j_names;
  //j_names = getMoveGroup("upper_body").getJointNames();
  j_names = jmg_upper_body->getVariableNames();
  if (!tracking_mode_flag_) {
    j_names.push_back("neck_r_joint");
    j_names.push_back("neck_p_joint");
    j_names.push_back("neck_y_joint");
  } else {
    ROS_WARN("tracking mode is on, not sending neck!");
  }

  //add lifter to trajectory
  if (_move_lifter == aero::ikrange::lifter) {
    j_names.reserve(static_cast<int>(j_names.size()) + 2);
    j_names.push_back("hip_joint");
    j_names.push_back("knee_joint");
    for (int i = 0; i < static_cast<int>(tra.size()); ++i) {
      tra[i].reserve(static_cast<int>(tra[i].size()) + 2);
      tra[i].push_back(xzs[i][0]);
      tra[i].push_back(xzs[i][1]);
    }
  }
  trajectory_msgs::JointTrajectory msg;
  msg.points.resize(tra.size());
  msg.joint_names.resize(j_names.size());
  msg.joint_names = j_names;
  int ms_total = 0;
  for (int i = 0; i < static_cast<int>(tra.size()); ++i) {
    msg.points[i].positions.resize(static_cast<int>(tra[i].size()));
    msg.points[i].positions = tra[i];
    ms_total += _times[i];
    ROS_INFO("ms_total %d", ms_total);
    msg.points[i].time_from_start = ros::Duration(ms_total * 0.001);
  }
  //  angle_vector_publisher_.publish(msg);
  return true;
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendTrajectoryAsync(aero::trajectory _trajectory, int _time_ms, aero::ikrange _move_lifter)
{
  int num = static_cast<int>(_trajectory.size());
  std::vector<int> times(num, _time_ms/num);
  return sendTrajectoryAsync(_trajectory, times, _move_lifter);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::overwriteSpeed(float _speed_overwrite)
{
  std_msgs::Float32 msg;
  msg.data = _speed_overwrite;
  overwrite_speed_publisher_.publish(msg);
  so_mutex_.lock();
  if (_speed_overwrite < 0.1) {
    so_retime_scale_ = 0.0;
  } else {
    so_retime_scale_ = msg.data / so_factor_;
    so_factor_ = msg.data;
    so_update_ = true;
  }
  so_mutex_.unlock();
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendLifter(double _x, double _z, int _time_ms, bool _local, bool _async)
{
  // send lifter
  std::map<std::string, double> _map;
  getRobotStateVariables(_map);
  if(setLifter(_x, _z)) {
    std::vector<double> av;
    const std::vector<std::string> &names = jmg_lifter->getVariableNames();
    kinematic_state->copyJointGroupPositions("lifter", av);
    ros::Time start_time = ros::Time::now() + ros::Duration(0.05); // starting 0.05sec after now
    ri->sendAngles(names, av, _time_ms*0.001, start_time);
    setRobotStateVariables(_map); // revert state
    return true;
  }
  return true;
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendLifter(int _x, int _z, int _time_ms)
{
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  return sendLifter(_x * 0.001, _z * 0.001, _time_ms);
#if 0
  aero_startup::AeroTorsoController srv;
  srv.request.x = _x;
  srv.request.z = _z;
  if (_time_ms == 0) srv.request.coordinate = "world";
  else srv.request.coordinate = "world:" + std::to_string(_time_ms);
  if (!waist_service_.call(srv)) {
    ROS_ERROR("move waist failed service call");
    return false;
  }

  if (srv.response.status == "success") {
    setLifter(_x/1000.0, _z/1000.0);
    if (wait_) {
      if (_time_ms == 0) usleep(static_cast<int>(srv.response.time_sec * 1000 * 0.8) * 1000);
      else usleep(static_cast<int>(_time_ms * 0.8) * 1000);
      waitInterpolation();
    } else {
      if (_time_ms == 0) usleep(static_cast<int>(srv.response.time_sec * 1000) * 1000 + 1000);
      else usleep(static_cast<int>(_time_ms) * 1000 + 1000);
    }

    return true;
  }
#endif
  return false;
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendLifterLocal(double _x, double _z, int _time_ms)
{
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  return sendLifterLocal(static_cast<int>(_x * 1000), static_cast<int>(_z * 1000), _time_ms);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendLifterLocal(int _x, int _z, int _time_ms)
{
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  std::map<aero::joint,double> pos;
  getLifter(pos);
  return sendLifter(static_cast<int>(pos[aero::joint::lifter_x] * 1000) + _x, static_cast<int>(pos[aero::joint::lifter_z] * 1000) + _z, _time_ms);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendLifterAsync(double _x, double _z, int _time_ms)
{
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  return sendLifterAsync(static_cast<int>(_x * 1000), static_cast<int>(_z * 1000), _time_ms);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendLifterAsync(int _x, int _z, int _time_ms)
{
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  return sendLifter(_x, _z, _time_ms, false, true);
#if 0
  aero_startup::AeroTorsoController srv;
  srv.request.x = _x;
  srv.request.z = _z;
  if (_time_ms == 0) srv.request.coordinate = "world";
  else srv.request.coordinate = "world:" + std::to_string(_time_ms);

  if (!waist_service_.call(srv)) {
    ROS_ERROR("move waist failed service call");
    return false;
  }

  if (srv.response.status == "success") {
    setLifter(_x, _z);
    return true;
  }
  return false;
#endif
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::cancelLifter()
{
#if 0
  // send cancel joints
  // why not use AeroSendJoints? -> to safe exit trajectory
  // but actually, cancel joints is not supported with AeroSendJoints

  trajectory_msgs::JointTrajectory msg;
  msg.points.resize(1);
  msg.joint_names.resize(2);
  msg.joint_names = {"hip_joint", "knee_joint"};
  msg.points[0].positions.resize(2);
  msg.points[0].positions = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
  msg.points[0].time_from_start = ros::Duration(0.01); // duration is not used
  angle_vector_publisher_.publish(msg);
  if (wait_) {
    waitInterpolation();
  } else {
    usleep(200 * 1000);
  }

  // get current joint angles
  aero_startup::AeroSendJoints srv;
  if (!joint_states_client_.call(srv)) { // TODO
    ROS_ERROR("getJoints failed service call");
    return false;
  }

  // update lifter
  auto hip_itr = std::find(srv.response.joint_names.begin(), srv.response.joint_names.end(), "hip_joint");
  auto knee_itr = std::find(srv.response.joint_names.begin(), srv.response.joint_names.end(), "knee_joint");
  double hip = srv.response.points.positions[static_cast<int>(hip_itr - srv.response.joint_names.begin())];
  double knee = srv.response.points.positions[static_cast<int>(knee_itr - srv.response.joint_names.begin())];
  double x = -lifter_foreleg_link_ * sin(knee - hip)
    + lifter_thigh_link_ * sin(hip);
  double z = lifter_foreleg_link_ * (cos(knee - hip) - 1.0)
    + lifter_thigh_link_ * (cos(hip) - 1.0);
  setLifter(x, z);

  updateLinkTransforms();
#endif
  return true;
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendLifterLocalAsync(double _x, double _z, int _time_ms)
{
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  return sendLifterLocalAsync(static_cast<int>(_x * 1000), static_cast<int>(_z * 1000), _time_ms);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendLifterLocalAsync(int _x, int _z, int _time_ms)
{
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  std::map<aero::joint, double> pos;
  getLifter(pos);
  return sendLifterAsync(static_cast<int>(pos[aero::joint::lifter_x] * 1000) + _x, static_cast<int>(pos[aero::joint::lifter_z] * 1000) + _z, _time_ms);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendLifterTrajectory(std::vector<std::pair<double, double>>& _trajectory, std::vector<int> _times)
{
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  return false;
#if 0
  if(!sendLifterTrajectoryAsync(_trajectory, _times)) return false;
  else {
    int time = std::accumulate(_times.begin(), _times.end(), 0);
    if (wait_) {
      usleep(static_cast<int>(time * 0.8) * 1000);
      waitInterpolation();
    } else {
      sleepInterpolation(time);
    }
    return true;
  }
#endif
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendLifterTrajectory(std::vector<std::pair<double, double>>& _trajectory, int _time_ms)
{
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  return false;
#if 0
  if(!sendLifterTrajectoryAsync(_trajectory, _time_ms)) return false;
  else {
    if (wait_) {
      usleep(static_cast<int>(_time_ms * 0.8) * 1000);
      waitInterpolation();
    } else {
      sleepInterpolation(_time_ms);
    }
    return true;
  }
#endif
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendLifterTrajectoryAsync(std::vector<std::pair<double, double>>& _trajectory, std::vector<int> _times)
{
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  return false;
#if 0
  setInterpolation(aero::interpolation::i_linear);

  trajectory_msgs::JointTrajectory msg;
  float time_from_start = 0.0;

  msg.joint_names.push_back("hip_joint");
  msg.joint_names.push_back("knee_joint");

  std::vector<std::vector<double>> xzs;
  xzs.reserve(static_cast<int>(_trajectory.size()));
  for (auto point : _trajectory) {
    std::vector<double> xz;
    if (!lifter_ik_(point.first, point.second, xz)) {
      ROS_WARN("lifter_ik failed");
      return false;
    }
    xzs.push_back(std::vector<double>{xz[0], xz[1]});
  }

  for (int i=0; i < static_cast<int>(_trajectory.size()); ++i) {
    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.push_back(xzs[i][0]);
    p.positions.push_back(xzs[i][1]);
    time_from_start += _times[i] * 0.001; // interval
    p.time_from_start = ros::Duration(time_from_start);
    msg.points.push_back(p);
  }

  angle_vector_publisher_.publish(msg);

  setInterpolation(aero::interpolation::i_constant);
  return true;
#endif
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendLifterTrajectoryAsync(std::vector<std::pair<double, double>>& _trajectory, int _time_ms)
{
  ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " : this function is deprecated");
  int num = static_cast<int>(_trajectory.size());
  std::vector<int> times(num, _time_ms/num);
  return sendLifterTrajectoryAsync(_trajectory, times);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::waitInterpolation(int _timeout_ms) {
#if 0
  bool check_timeout = false;
  if (_timeout_ms > 0) check_timeout = true;
  ros::Duration timeout = ros::Duration(_timeout_ms * 0.001);
  ros::Time start = ros::Time::now();

  std_srvs::Trigger srv;
  while (ros::ok()) {
    usleep(50 * 1000);// 20Hz
    if (in_action_service_.call(srv)) {
      bool in_action = srv.response.success;
      if (!in_action) {
        ROS_INFO("%s: finished", __FUNCTION__);
        return true;
      }
    }

    if (check_timeout && start + timeout < ros::Time::now()) {
      ROS_WARN("%s: timeout! %d[ms]", __FUNCTION__, _timeout_ms);
      break;
    }
  }
  return false;
#endif
  return ri->wait_interpolation( _timeout_ms/1000.0 );
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sleepInterpolation(int _time_ms)
{
  so_mutex_.lock();
  so_factor_ = 1.0f;
  so_retime_scale_ = 1.0f;
  so_mutex_.unlock();
  int count = 0;
  int update_ms = 100;
  int update_count = _time_ms / update_ms;
  for (int i = 0; i < update_count; ) {
    so_mutex_.lock();
    if (so_update_) {
      update_count = i + static_cast<int>((update_count - i) / so_retime_scale_);
      so_update_ = false;
    }
    so_mutex_.unlock();
    usleep(update_ms * 1000);
    so_mutex_.lock();
    if (so_retime_scale_ > 0.00001) { // if not 0.0
      ++count;
      ++i;
    }
    so_mutex_.unlock();
  }
  usleep(std::max(0, _time_ms - count * update_ms) + 1000);
  so_mutex_.lock();
  so_factor_ = 1.0f;
  so_retime_scale_ = 1.0f;
  so_mutex_.unlock();
  // usleep(static_cast<int>(_time_ms) * 1000 + 1000);
}
