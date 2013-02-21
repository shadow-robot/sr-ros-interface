
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

    //returns 0, 0 if joint not found
    min_max = handcmd.get_min_max("stupid");
    EXPECT_DOUBLE_EQ(min_max.first, 0.0);
    EXPECT_DOUBLE_EQ(min_max.second, 0.0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "hand_commander_test");
    ros::NodeHandle nh; // init the node
    return RUN_ALL_TESTS();
}
