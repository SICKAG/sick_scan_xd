#include <gtest/gtest.h>

#ifndef _WIN32

#include <cstdlib>

TEST(SickScanXdStartTest, NodeStarts)
{
    int ret = system(
        "bash -c \"source /opt/ros/$ROS_DISTRO/setup.bash && "
        "timeout 3s ros2 run sick_scan_xd sick_generic_caller\""
    );

    // 0 = sauber beendet
    // 124 = timeout → Node lief → OK
    EXPECT_TRUE(ret == 0 || ret == 124);
}

#else

TEST(SickScanXdStartTest, SkipOnWindows)
{
    GTEST_SKIP() << "Start test skipped on Windows";
}

#endif