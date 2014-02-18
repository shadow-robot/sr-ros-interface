
// We are testing this
#include "sr_hand/hand_commander.hpp"

// ROS
#include "ros/ros.h"

// Gtest
#include <gtest/gtest.h>

using namespace std;
using namespace shadowrobot;

TEST(HandCommander, constructor)
{
    HandCommander handcmd = HandCommander();
    EXPECT_TRUE(true);
}

TEST(HandCommander, topic_state)
{
    HandCommander handcmd = HandCommander();

    // get_controller_state_topic returns fully resolved topics
    ros::NodeHandle nh;

    std::string topic = handcmd.get_controller_state_topic("ffj3");
    EXPECT_EQ(nh.resolveName("sh_ffj3_position_controller/state"), topic);

    topic = handcmd.get_controller_state_topic("ffj0");
    EXPECT_EQ(nh.resolveName("sh_ffj0_position_controller/state"), topic);

    topic = handcmd.get_controller_state_topic("unknown joint");
    EXPECT_EQ("", topic);
}

TEST(HandCommander, min_max)
{
    HandCommander handcmd = HandCommander();

    std::pair<double, double> min_max = handcmd.get_min_max("FFJ3");
    EXPECT_DOUBLE_EQ(min_max.first, 0.0);
    EXPECT_DOUBLE_EQ(min_max.second, 1.57079632679);

    //also works for lower case
    min_max = handcmd.get_min_max("ffj3");
    EXPECT_DOUBLE_EQ(min_max.first, 0.0);
    EXPECT_DOUBLE_EQ(min_max.second, 1.57079632679);

    //j0 should be 0, 180
    min_max = handcmd.get_min_max("ffj0");
    EXPECT_DOUBLE_EQ(min_max.first, 0.0);
    EXPECT_DOUBLE_EQ(min_max.second, 3.14159265358);

    //returns 0, 0 if joint not found
    min_max = handcmd.get_min_max("unknown joint");
    EXPECT_DOUBLE_EQ(min_max.first, 0.0);
    EXPECT_DOUBLE_EQ(min_max.second, 0.0);

    //Check that we can get the min and max for each joint
    std::vector<std::string> all_joints = handcmd.get_all_joints();
    for( size_t i=0; i < all_joints.size(); ++i)
    {
       min_max = handcmd.get_min_max(all_joints[i]);
       EXPECT_TRUE(min_max.first != min_max.second); //min = max if joint not found
    }
}

TEST(HandCommander, all_joints)
{
    HandCommander handcmd = HandCommander();

    EXPECT_EQ(handcmd.get_all_joints().size(), 20);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "hand_commander_test");
    ros::NodeHandle nh; // init the node

    //sleep until gazebo is ready
    sleep(10.0);

    return RUN_ALL_TESTS();
}
