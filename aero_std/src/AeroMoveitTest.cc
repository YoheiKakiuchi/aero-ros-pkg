#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

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

void testModel(std::string name,
               robot_model::RobotModelPtr kinematic_model,
               robot_state::RobotStatePtr kinematic_state)
{
  ROS_INFO("");
  ROS_INFO("------------ for group / %s ------------ ", name.c_str());
  ROS_INFO("");

  robot_state::JointModelGroup* jmg = kinematic_model->getJointModelGroup(name);
  //robot_state::JointModelGroup* jmg = kinematic_model->getJointModelGroup("larm");
  // jmg->attachEndEffector("left_eef");
  if (!!jmg) {
    const std::vector< std::string> nm_act = jmg->getActiveJointModelNames();
    ROS_INFO("getActiveJointModelNames() / %ld", nm_act.size());
    {
      std::stringstream sout;
      for(std::string nm: nm_act) sout << " " << nm.c_str();
      ROS_INFO_STREAM(sout.str());
    }

    const std::vector< std::string> nm_all = jmg->getJointModelNames();
    ROS_INFO("getJointModelNames() / %ld", nm_all.size());
    {
      std::stringstream sout;
      for(std::string nm: nm_all) sout << " " << nm.c_str();
      ROS_INFO_STREAM(sout.str());
    }

    int jm_vc = jmg->getVariableCount();
    const std::vector< std::string> nm_jm = jmg->getVariableNames();
    ROS_INFO("getVariableNames() / %ld, VariableCount = %d", nm_jm.size(), jm_vc);
    {
      std::stringstream sout;
      for(std::string nm: nm_jm) sout << " " << nm.c_str();
      ROS_INFO_STREAM(sout.str());
    }
    std::vector<double> tmp;
    kinematic_state->copyJointGroupPositions(jmg, tmp);
    ROS_INFO("copyJointGroupPositions() / %ld", tmp.size());


    std::vector <std::string> tips;
    jmg->getEndEffectorTips(tips);
    ROS_INFO("tips = %ld", tips.size());
    for(std::string nm: tips) ROS_INFO("tip: %s", nm.c_str());

    const std::vector <std::string> &links = jmg->getLinkModelNames();
    ROS_INFO("links = %ld", links.size());
    for(std::string nm: links) ROS_INFO("link: %s", nm.c_str());
  }
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "robot_interface_test");
  ros::NodeHandle nh;

  robot_model_loader::RobotModelLoader robot_model_loader_
    = robot_model_loader::RobotModelLoader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader_.getModel();
  robot_state::RobotStatePtr kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();// set all joints to 0.0

  ROS_INFO_STREAM("root_link: "  << kinematic_model->getRootLink()->getName());
  ROS_INFO_STREAM("joint_link: " << kinematic_model->getRootJoint()->getName());

  // check variables
  int km_vc = kinematic_model->getVariableCount();
  const std::vector< std::string> nm_km = kinematic_model->getVariableNames();
  ROS_INFO("kinematic_model: VariableCount = %d", km_vc);
  for(std::string nm: nm_km) ROS_INFO("vname: %s", nm.c_str());

  std::vector <std::string > groups = {
    "waist",
    "lifter",
    "larm_with_torso",
    "larm_torso_lifter",
    "both_arms",
    "head",
    "upper_body",
    "torso"
  };

  for(std::string nm: groups) {
    testModel(nm, kinematic_model, kinematic_state);
  }

  ROS_INFO("");
  ROS_INFO("------ pointer check ------");
  ROS_INFO("");
  // multiple ??
  robot_model::RobotModelPtr kinematic_model2 = robot_model_loader_.getModel();
  robot_model::RobotModelPtr kinematic_model3;
  {
    kinematic_model3 = robot_model_loader::RobotModelLoader("robot_description").getModel();
  }
  robot_state::RobotStatePtr kinematic_state2 = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
  robot_state::RobotStatePtr kinematic_state3 = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model2));
  robot_state::RobotStatePtr kinematic_state4 = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model3));
  ROS_INFO("kinematic model %lX, %lX, %lX", (void *)(kinematic_model.get()), (void *)(kinematic_model2.get()), (void *)(kinematic_model3.get()));
  ROS_INFO("kinematic state %lX, %lX, %lX, %lX", (void *)(kinematic_state.get()), (void *)(kinematic_state2.get()), (void *)(kinematic_state3.get()), (void *)(kinematic_state4.get()));
  robot_state::JointModelGroup* jmg1 = kinematic_model ->getJointModelGroup("larm");
  robot_state::JointModelGroup* jmg2 = kinematic_model2->getJointModelGroup("larm");
  robot_state::JointModelGroup* jmg3 = kinematic_model3->getJointModelGroup("larm");
  ROS_INFO("jmg1: %lX, jmg2 %lX, jmg3 %lX", (void *)jmg1, (void *)jmg2, (void *)jmg3);

  const robot_state::JointModelGroup* sjmg1 = kinematic_state ->getJointModelGroup("larm");
  const robot_state::JointModelGroup* sjmg2 = kinematic_state2->getJointModelGroup("larm");
  const robot_state::JointModelGroup* sjmg3 = kinematic_state3->getJointModelGroup("larm");
  const robot_state::JointModelGroup* sjmg4 = kinematic_state4->getJointModelGroup("larm");
  ROS_INFO("sjmg1 %lX, sjmg2 %lX, sjmg3 %lX, sjmg4 %lX", (void *)sjmg1, (void *)sjmg2, (void *)sjmg3, (void *)sjmg4);
  ROS_INFO("vp: %f %f %f %f",
           kinematic_state ->getVariablePosition("l_shoulder_p_joint"),
           kinematic_state2->getVariablePosition("l_shoulder_p_joint"),
           kinematic_state3->getVariablePosition("l_shoulder_p_joint"),
           kinematic_state4->getVariablePosition("l_shoulder_p_joint"));
  kinematic_state ->setVariablePosition("l_shoulder_p_joint", 0.1);
  kinematic_state2->setVariablePosition("l_shoulder_p_joint", 0.2);
  kinematic_state3->setVariablePosition("l_shoulder_p_joint", 0.3);
  kinematic_state4->setVariablePosition("l_shoulder_p_joint", 0.4);
  ROS_INFO("vp: %f %f %f %f",
           kinematic_state ->getVariablePosition("l_shoulder_p_joint"),
           kinematic_state2->getVariablePosition("l_shoulder_p_joint"),
           kinematic_state3->getVariablePosition("l_shoulder_p_joint"),
           kinematic_state4->getVariablePosition("l_shoulder_p_joint"));
  // IK test
  const robot_state::JointModelGroup* jmg_larm = kinematic_model->getJointModelGroup("lifter");
  //LinkModel *lbase = jmg_larm->getLinkModel();
  //LinkModel *ltip = jmg_larm->getLinkModel("lifter_top_link");
  {
    const Eigen::Affine3d &base_trans = kinematic_state->getGlobalLinkTransform("lifter_base_link");
    const Eigen::Affine3d &tip_trans =  kinematic_state->getGlobalLinkTransform("lifter_top_link");
    Eigen::Affine3d diff_trans = base_trans.inverse() * tip_trans;
    std::cout << "base: " << base_trans << std::endl;
    std::cout << "tip:  " << tip_trans << std::endl;
    std::cout << "diff: " << diff_trans << std::endl;
  }
  double x = 0;
  double z = 0.225;
  Eigen::Affine3d diff_t = Eigen::Translation3d(x, 0, 0.725 - z) * Eigen::Quaterniond::Identity();
  const Eigen::Affine3d &base_trans = kinematic_state->getGlobalLinkTransform("lifter_base_link");
  Eigen::Affine3d _pose = base_trans * diff_t;

  int _attempts = 3;
  bool found_ik = kinematic_state->setFromIK(jmg_larm, _pose, _attempts, 0.1);
  //bool found_ik = kinematic_state->setFromIK(jmg_larm, _pose, "waist_link", _attempts, 0.1);
  if (true) {
    ROS_INFO("found_ik: %d", found_ik);
    std::vector <double> jv;
    kinematic_state->copyJointGroupPositions(jmg_larm, jv);
    for(std::size_t i=0; i < jv.size(); ++i) {
      ROS_INFO("Joint : %f", jv[i]);
    }
  }

  {
    const Eigen::Affine3d &base_trans = kinematic_state->getGlobalLinkTransform("lifter_base_link");
    const Eigen::Affine3d &tip_trans =  kinematic_state->getGlobalLinkTransform("lifter_top_link");
    Eigen::Affine3d diff_trans = base_trans.inverse() * tip_trans;
    std::cout << "base: " << base_trans << std::endl;
    std::cout << "tip:  " << tip_trans << std::endl;
    std::cout << "diff: " << diff_trans << std::endl;
  }
  // ros::spin();
}
