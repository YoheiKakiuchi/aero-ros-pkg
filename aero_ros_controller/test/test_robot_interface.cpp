#include <ros/ros.h>
#include <gtest/gtest.h>

#include <aero_ros_controller/RobotInterface.hh>

class RobotInterfaceTest: public testing::Test
{
protected:
  virtual void SetUp()
  {
    nh.reset(new ros::NodeHandle());
    ROS_INFO("Set UP a");
    ROS_WARN("Set UP a");

    ri.reset(new robot_interface::RobotInterface(*nh));

    ros::Duration d(3);
    d.sleep();

    ROS_INFO("Set UP b");
  }

  virtual void TearDown() {
    // delete ri.get();
    ROS_INFO("Tear Down");
  }

  boost::shared_ptr< ros::NodeHandle> nh;
  robot_interface::RobotInterface::Ptr ri;
};

TEST_F(RobotInterfaceTest, testCreate)
{
  ASSERT_TRUE(ri.get() != NULL);
  ROS_WARN("test end");
  //EXPECT_EQ (a ,b);
  // EXPECT_FLAOT_EQ (a, b);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_robot_interface");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/*

ikinari
wait_interpolation

send/wait
wait_interpolation


redundant wait_interpolation


multiple/send
multiple/wait


multiple/send
single/wait


reference check

actual check


send angle-vector

send angle-vector-sequence


start-time-check


send small-joint-map

 */
