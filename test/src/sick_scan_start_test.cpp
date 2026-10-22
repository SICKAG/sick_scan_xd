#include <gtest/gtest.h>

#ifndef _WIN32

#include <chrono>
#include <csignal>
#include <cerrno>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>


TEST(SickScanXdStartTest, NodeStarts)
{
    const pid_t pid = fork();

    ASSERT_NE(pid, -1) << "fork() failed";

    if (pid == 0)
    {
        // Run ros2 and the started node in their own process group.
        // This allows the complete process tree to be terminated after
        // the startup test.
        if (setpgid(0, 0) != 0)
        {
            _exit(126);
        }

        // colcon test already provides the environment of the package
        // under test. Do NOT source /opt/ros/$ROS_DISTRO/setup.bash here,
        // because that would reset the environment to the ROS underlay.
        execlp(
            "ros2",
            "ros2",
            "run",
            "sick_scan_xd",
            "sick_generic_caller",
            static_cast<char *>(nullptr));

        // execlp() only returns if starting ros2 failed.
        _exit(127);
    }

    // Give the node enough time to start. If ros2 or the node terminates
    // during this period, the startup test has failed.
    constexpr auto startup_time = std::chrono::seconds(3);

    int status = 0;
    bool running = true;

    const auto deadline = std::chrono::steady_clock::now() + startup_time;

    while (std::chrono::steady_clock::now() < deadline)
    {
        const pid_t result = waitpid(pid, &status, WNOHANG);

        ASSERT_NE(result, -1) << "waitpid() failed";

        if (result == pid)
        {
            running = false;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!running)
    {
        if (WIFEXITED(status))
        {
            FAIL() << "ros2 run sick_scan_xd sick_generic_caller "
                   << "exited prematurely with code "
                   << WEXITSTATUS(status);
        }

        if (WIFSIGNALED(status))
        {
            FAIL() << "ros2 run sick_scan_xd sick_generic_caller "
                   << "terminated prematurely by signal "
                   << WTERMSIG(status);
        }

        FAIL() << "ros2 run sick_scan_xd sick_generic_caller "
               << "terminated prematurely";
    }

    // The process survived the startup period. Terminate the complete
    // process group created above.
    if (kill(-pid, SIGTERM) != 0 && errno != ESRCH)
    {
        ADD_FAILURE() << "Failed to send SIGTERM to process group";
    }

    // Give the process group a short grace period to terminate cleanly.
    constexpr auto shutdown_time = std::chrono::seconds(2);
    const auto shutdown_deadline =
        std::chrono::steady_clock::now() + shutdown_time;

    bool reaped = false;

    while (std::chrono::steady_clock::now() < shutdown_deadline)
    {
        const pid_t result = waitpid(pid, &status, WNOHANG);

        if (result == pid)
        {
            reaped = true;
            break;
        }

        if (result == -1)
        {
            if (errno == ECHILD)
            {
                reaped = true;
                break;
            }

            ADD_FAILURE() << "waitpid() failed while terminating process";
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Do not leave processes behind on the buildfarm.
    if (!reaped)
    {
        kill(-pid, SIGKILL);
        waitpid(pid, &status, 0);
    }

    SUCCEED();
}

#else

TEST(SickScanXdStartTest, SkipOnWindows)
{
    GTEST_SKIP() << "Start test skipped on Windows";
}

#endif
