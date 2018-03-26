#include <aero_std/AeroLookatCommander.hh>

aero::lookat_commander::AeroLookatCommander::AeroLookatCommander(ros::NodeHandle &_nh,
                                                                 aero::interface::AeroMoveitInterface *_ami)
{
  ami_.reset(_ami);
  kinematic_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(ami_->kinematic_model));

  ros::TimerOptions tmopt(ros::Duration(0.1),
                          boost::bind(&aero::lookat_commander::AeroLookatCommander::timerCallback,
                                      this, _1),
                          &eventqueue_);
  _nh.createTimer(tmopt);
  //
}

bool aero::lookat_commander::AeroLookatCommander::setTrackingMode(aero::tracking _mode, const aero::Vector3 &_pos)
{
  boost::mutex::scoped_lock lk(callback_mtx_);
  tracking_mode_ = _mode;

  switch(tracking_mode_) {
  case aero::tracking::map:
  case aero::tracking::map_static:
    {
      aero::Vector3 vec_in_base;
      vec_in_base = ami_->volatileTransformToBase(_pos.x(), _pos.y(), _pos.z());
      ami_->setRobotStateToCurrentState(kinematic_state_);
      ami_->setLookAt_(vec_in_base.x(), vec_in_base.y(), vec_in_base.z(),
                       kinematic_state_);
      tracking_pos_ = _pos;
    }
    break;
  case aero::tracking::base:
  case aero::tracking::base_static:
    ami_->setRobotStateToCurrentState(kinematic_state_);
    ami_->setLookAt_(_pos.x(), _pos.y(), _pos.z(), kinematic_state_);
    tracking_pos_ = _pos;
    break;
  default:
    return false;
    break;
  }
  ami_->sendNeckAsync_(2000, kinematic_state_);

  return true;
}

bool aero::lookat_commander::AeroLookatCommander::setNeckRPY(double _r, double _p, double _y)
{
  ami_->setNeck_(_r, _p, _y, kinematic_state_);
  ami_->sendNeckAsync_(2000, kinematic_state_);
  return true;
}

bool aero::lookat_commander::AeroLookatCommander::setLookAtTopic(const std::string &_topic)
{
  // TODO:
  return true;
}

std::tuple<double, double, double> aero::lookat_commander::AeroLookatCommander::getNeck()
{
  return std::tuple<double, double, double>
    (ami_->kinematic_state->getVariablePosition("neck_r_joint"),
     ami_->kinematic_state->getVariablePosition("neck_p_joint"),
     ami_->kinematic_state->getVariablePosition("neck_y_joint"));
}

void aero::lookat_commander::AeroLookatCommander::timerCallback(const ros::TimerEvent& ev)
{
  boost::mutex::scoped_lock lk(callback_mtx_);
  //
  std::tuple<double, double, double> from;
  std::tuple<double, double, double> to;
  switch(tracking_mode_) {
  case aero::tracking::map:
    {
      aero::Vector3 vec_in_base;
      vec_in_base = ami_->volatileTransformToBase(tracking_pos_.x(), tracking_pos_.y(), tracking_pos_.z());
      ami_->setRobotStateToCurrentState(kinematic_state_);
      from = getNeck();
      ami_->setLookAt_(vec_in_base.x(), vec_in_base.y(), vec_in_base.z(),
                       kinematic_state_);
      to = getNeck();
    }
    break;
  case aero::tracking::base:
    ami_->setRobotStateToCurrentState(kinematic_state_);
    from = getNeck();
    ami_->setLookAt_(tracking_pos_.x(), tracking_pos_.y(), tracking_pos_.z(), kinematic_state_);
    to = getNeck();
    break;
  default:
    return;
    break;
  }
  double yaw_vel =
    ami_->kinematic_model->getVariableBounds("neck_y_joint").max_velocity_;
  double pitch_vel =
    ami_->kinematic_model->getVariableBounds("neck_p_joint").max_velocity_;
  double time = std::max((std::get<2>(from) - std::get<2>(to)) / yaw_vel,
                         (std::get<1>(from) - std::get<1>(to)) / pitch_vel);
  ami_->sendNeckAsync_(1000 * time + 100, kinematic_state_);
}
