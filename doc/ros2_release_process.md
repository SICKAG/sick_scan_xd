# Background Information of ROS 2 Release Process

## Introduction
This document summarizes the ROS 2 release process — in particular the use of *bloom*, local pre-tests, staging processes, release repository updates, and related best practices.

## Key Terms  
- **Bloom**: the ROS package release tool that automates the generation of Debian packages, tagging, and repository updates.  
- **Staging / Release Repository**: The GitHub repositories or `ros2-gbp` repositories where releases are pushed and indexed.  
- **Track / Distro**: ROS 2 distributions (e.g., Humble, Iron, Jazzy, Kilted) used as target platforms for release.  
- **APT propagation**: The time taken for built Debian binaries to become available via `apt install`.

## Workflow Overview  
1. **Clone and prepare source**  
   
   ```bash
   git clone -b master https://github.com/SICKAG/sick_scan_xd.git
   cd sick_scan_xd
   ```


2. **Local pre-test / prerelease build**

   *   Build the workspace (e.g., with `colcon`, `cmake` or `catkin` depending on the ROS version)
   *   Run tests in Docker or a native environment to ensure build and runtime correctness (see CONTRIBUTING.md for more details)

3. **Update version and changelog**
   *   Increment version in `package.xml`
   *   Use the changelog generator (e.g., `catkin_generate_changelog --all`)

4. **Local pretest of bloom-release**

    You can pretest the bloom release by emulation the bloom release on your local machine in a Docker environment by running the following steps:

    * * *

    **a. Set up a Python virtual environment**

    It’s good practice to isolate the ROS build farm tools inside a dedicated virtual      environment.

    ```bash
    cd /tmp
	  python3 -m venv .venv
	  source .venv/bin/activate
    ```

	  This creates and activates a clean environment named `.venv`.
	
	  **b. Install required Python packages**
	
	  Install the tools needed for ROS 2 prerelease script generation.

    ```bash
	  pip install -U pip setuptools
	  pip install ros_buildfarm rosdep rosdistro rosdistro-modules catkin_pkg
    ```

	  These packages provide:
	
	  * 	**ros\_buildfarm** → tools to generate and run prerelease tests
	  *   **rosdep / rosdistro** → dependency and distribution management
	  *   **catkin\_pkg** → workspace and package parsing support

	  **c. Generate the prerelease script**
	
	  Use the official ROS 2 build farm configuration to generate the test 	script for your package.  
	  Example:

    ```bash
	  generate_prerelease_script.py \
	  https://raw.githubusercontent.com/ros2/ros_buildfarm_config/ros2/index.yaml \
	  kilted default ubuntu noble amd64 \
	  --pkg sick_scan_xd \
	  --custom-repo 		sick_scan_xd:git:https://github.com/SICKAG/sick_scan_xd.git:feature/bloom_prerelease_test \
    --output-dir ./prerelease_kilted_feature_test
    ```

    **Explanation:**

    *   `index.yaml`: build farm configuration file for ROS 2
    *   `kilted`: target ROS 2 distribution (e.g., Jazzy, Iron, or a newer codename)
    *   `default ubuntu noble amd64`: defines the build target platform (Ubuntu 24.04 × 64-bit)
    *   `--pkg`: selects your package to test (`sick_scan_xd`)
    *   `--custom-repo`: overrides the default repository branch with your GitHub branch (excellent for pretesting a feature branch)
    *   `--output-dir`: defines where the generated test script will be placed


    **d. Run the generated prerelease test**

    The previous command creates a directory named `prerelease_kilted_feature_test/` containing a shell script — typically `prerelease.sh`.

    Change into that directory and execute the script:

    ```bash
    cd prerelease_kilted_feature_test
    ./prerelease.sh
    ```

    This script:

    *   Sets up a Docker environment
    *   Builds your ROS 2 workspace inside the container
    *   Runs unit and integration tests
    *   Reports the results in the terminal


    **e. Analyze the results**

    When the script finishes, review the console output.  
    You will see:

    *   ✅ successful builds and test passes
    *   ❌ failures (with detailed logs and stack traces)

    If the test runs correctly, the package is ready for release or further CI integration (i.e. run bloom-release)

​      

5. **Run bloom-release**
    ```bash
    bloom-release --rosdistro <distro> --track <track> sick_scan_xd
    ```

   * For a **new track**, use `--new-track`
   * Example for Kilted:

    ```bash
        bloom-release --new-track --rosdistro kilted --track kilted sick_scan_xd
    ```
6.  **Monitor Push to release repository**
    *   Ensure the tag and version appear in the repository (e.g., [https://github.com/ros2-gbp/sick\_scan\_xd-release](https://github.com/ros2-gbp/sick_scan_xd-release))
    *   Wait for CI/Jenkins builds to succeed (usually 0-3 days)
    *   Monitor Jenkins jobs for each distro

7.  **Monitor APT propagation**
    *   After merge, run:
    ```bash
        sudo apt update
        sudo apt show ros-<distro>-sick-scan-xd
    ```
    *   Expect the package to appear typically in 4–6 weeks

7.  **Post-release checks**
    *   Confirm Debian packages install correctly
    *   Monitor issue trackers for release-specific bugs
    *   Document any deviations or special notes in `CHANGELOG.rst`

Staging and Release Repositories
--------------------------------

*   Release repositories differ between ROS 1 and ROS 2:
    *   ROS 1: `https://github.com/SICKAG/sick_scan_xd-release.git`
    *   ROS 2: `https://github.com/ros2-gbp/sick_scan_xd-release.git`
*   After running `bloom-release`, verify:
    *   The version tag is present
    *   `tracks.yaml` points to the correct `devel_branch` (e.g., `develop` or `master`)
	*   Direct link: `https://github.com/ros2-gbp/sick_scan_xd-release/blob/master/tracks.yaml` 
*   If the release repo still points to an old branch, update the `devel_branch` and re-trigger a release
*   Overview over various version vs. ROS2-Releases: [https://index.ros.org/p/sick_scan_xd/](https://index.ros.org/p/sick_scan_xd/)

Best Practices and Tips
-----------------------

*   Always update `CHANGELOG.rst` when releasing
*   Use semantic versioning (major.minor.patch)
*   For new ROS 2 distros (like Kilted), explicitly run `--new-track`
*   Document any special hardware or dependency requirements in the release notes
*   Avoid releasing to EOL distributions (e.g., ROS 2 Foxy, now unsupported)
*   Integration into Ubuntu packages
    An entry point for verifying the integration of packages into Ubuntu’s package management system can be found [here](https://repo.ros2.org/ubuntu/)
    The Ubuntu integration follows a staged release process consisting of three main phases: `building → testing → main`. 
    Each stage represents a level of stability and readiness for general use:
    * building – Packages are initially built in this stage. The goal is to ensure that all dependencies are satisfied and that the package compiles 
	  successfully on supported Ubuntu distributions and architectures. Failures at this stage indicate build or dependency issues.
	  * Transition time: Packages usually remain in this stage for a few hours to several days, depending on build queue load and the complexity of dependencies.
    * testing – Successfully built packages move into the testing stage, where they undergo functional and integration testing. This phase ensures 
	  that the software installs cleanly, runs as expected, and does not conflict with existing packages. Continuous integration (CI) systems and automated test suites typically validate these criteria.
	  * Transition time: Testing generally takes from several hours up to a week. The duration depends on the breadth of the test suite and how quickly any detected issues are resolved.
    * main – Once a package has passed all tests and quality assurance checks, it is promoted to the main repository. Packages in this stage are considered stable, production-ready, and available to all users through standard Ubuntu package management tools such as apt.
      * Transition time: Promotion to the main stage typically occurs within a few days after successful testing, following manual review or automated approval processes.
      Overall, the time between building and main can vary from a single day to over a week, depending on factors such as build infrastructure load, the size of dependency chains, 
	  and the outcomes of automated and manual tests.
      This staged approach ensures that only verified and stable packages reach end users, maintaining the overall integrity and reliability of the Ubuntu repositories hosting ROS 2 packages
	
	* If the package is not released, you can also try to install the package by getting the deb package in the testing phase (or in the `building` phase - not recommend, too early).
	  Just search via `sick-scan-xd` in the following list:
	  * Building: https://repo.ros2.org/ubuntu/building/pool/main/r/
	  * Testing: https://repo.ros2.org/ubuntu/testing/pool/main/r/
	  * Main: https://repo.ros2.org/ubuntu/main/pool/main/r/
	* If the package has not yet been officially released, you can still try to install it manually by downloading the corresponding `.deb` file from the **testing** phase (or, if necessary, from the **building** phase — though this is not recommended, as it is often too early and may be unstable).  
	  To locate the package, search for `sick-scan-xd` in one of the following repositories:

	  * **Building:** https://repo.ros2.org/ubuntu/building/pool/main/r/  
	  * **Testing:** https://repo.ros2.org/ubuntu/testing/pool/main/r/  
	  * **Main:** https://repo.ros2.org/ubuntu/main/pool/main/r/  

	  A typical example entry looks like this:  
	  https://repo.ros2.org/ubuntu/main/pool/main/r/ros-humble-sick-scan-xd/  

	  Within this directory, look for a file ending with `_amd64.deb`.  
	  You can then install it using commands similar to the following:

	  ```bash
	  # Download the package
	  wget https://repo.ros2.org/ubuntu/testing/pool/main/r/ros-kilted-sick-scan-xd/ros-kilted-sick-scan-xd_3.8.0-1noble.20251103.175741_amd64.deb
	  
	  # Install the package (automatically resolving dependencies)
	  sudo apt install ./ros-kilted-sick-scan-xd_3.8.0-1noble.20251103.175741_amd64.deb
	  
	  # Verify the installation and version
	  apt show ros-kilted-sick-scan-xd | grep Version
	  ```
	  
	  This example is adapted from the discussion [here](https://github.com/SICKAG/sick_scan_xd/issues/528)

## Troubleshooting Common Issues

| Issue | Description | Solution |
| --- | --- | --- |
| Bloom builds old version | `devel_branch` unset or incorrect | Update `tracks.yaml` with correct branch and rerun bloom |
| apt installs older version | Debian sync delay | Wait 4–6 weeks or manually install from source or from the previous testing stage (see https://repo.ros2.org/ubuntu/testing/pool/main/r/ ) |

## References

*   ROS 2 Bloom Release Guide
*   [ROS Build Farm Prerelease Interface](https://prerelease.ros.org/)
## Summary

This document captures the core steps and background of the ROS2 release process using bloom. With proper local pre-testing, staging, and repository tracking, releases become more robust and less error-prone. For the `sick_scan_xd` project targeting ROS 2 distros including Humble, Iron, Jazzy and Kilted, following this guide ensures a consistent and repeatable process.

