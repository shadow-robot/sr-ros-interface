
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

TEST(HandCommander, topic_state)
{
    HandCommander handcmd = HandCommander();

    std::string topic = handcmd.get_controller_state_topic("ffj3");
    EXPECT_EQ("/sh_ffj3_mixed_position_velocity_controller/state", topic);

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "hand_commander_test");
    ros::NodeHandle nh; // init the node
    return RUN_ALL_TESTS();
}
