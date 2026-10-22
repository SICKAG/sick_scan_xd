#include <gtest/gtest.h>

/**
 * This is a minimal smoke test used for ROS 2 bloom and the build farm.
 *
 * Its purpose is not to verify functionality, but to ensure that:
 * - the package successfully compiles,
 * - the test infrastructure is correctly configured,
 * - and at least one test is discovered and executed by the ROS 2 build farm (colcon test).
 *
 * Bloom requires packages to provide tests so that the build farm can validate
 * the build and test pipeline. Even a trivial test like this helps detect
 * configuration issues (e.g., missing dependencies, broken test setup, or
 * misconfigured CMake/ament settings).
 *
 * In short, this test acts as a "smoke test" confirming that the package builds
 * and the testing framework runs without errors.
 */
TEST(SickScanXdMinimalGTest, BuildfarmSmokeTest)
{
  EXPECT_TRUE(true);
}