
// We are testing this
#include "sr_hand/hand_commander.h"

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
    return RUN_ALL_TESTS();
}
