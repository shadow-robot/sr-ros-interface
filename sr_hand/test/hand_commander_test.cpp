
// We are testing this
#include "sr_hand/hand_commander.hpp"

// ROS
#include "ros/ros.h"

// Gtest
#include <gtest/gtest.h>

using namespace shadowrobot;

TEST(HandCommander, constructor)
{
    HandCommander handcmd = HandCommander();
    EXPECT_TRUE(true);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "hand_commander_test");
    ros::NodeHandle nh; // init the node
    return RUN_ALL_TESTS();
}
