# sick_scan_xd releases using bloom:

## First time installation of toolchain

1. Install on Linux:
    * Install bloom: 
        ```
        sudo apt-get update
        sudo apt-get install python3-bloom python3-catkin-pkg
        ```
    * Install docker:
        ```
        pushd /tmp
        curl -fsSL https://get.docker.com -o get-docker.sh
        sudo sh get-docker.sh
        sudo usermod -aG docker $USER
        popd
        shutdown -r now # reboot
        # short quicktest
        docker --version
        docker info
        docker run hello-world
        ```
    * Install ros-buildfarm:
        ```
        # sudo apt-get install python3-ros-buildfarm # not successfully, unable to locate
        pip3 install ros-buildfarm # installs ros-buildfarm 3.0 successfully
        ```

2. Build the prerelease:
    * Short version to build a prerelase:
        * Run the following commands:
            ```
            mkdir -p ./ws_sick_scan_xd_bloom/src
            cd ./ws_sick_scan_xd_bloom/src
            git clone -b master https://github.com/SICKAG/sick_scan_xd.git
            cd ./sick_scan_xd/test/scripts
            ./run_linux_ros1_bloom.bash
            ```
        * Fix any errors during the prerelease build and check in
        * Repeat `./run_linux_ros1_bloom.bash` until the the prerelease build finishes without errors
    * Alternative version:
        * Open http://prerelease.ros.org/noetic in the brower
        * Add a custom repository: `sick_scan_xd` , `https://github.com/SICKAG/sick_scan_xd` , `master` (or `feature/bloom_pretest` or any other branch to test)
        * Add a custom repository: `msgpack11` , `https://github.com/SICKAG/msgpack11` , `master`
        * Add a custom repository: `libsick_ldmrs` , `https://github.com/SICKAG/libsick_ldmrs` , `master`
        * Confirm next steps (i.e. URL of build farm: https://raw.githubusercontent.com/ros-infrastructure/ros_buildfarm_config/production/index.yaml, Ubuntu focal)
        * Click on `Generate command`
        * Run the generated command, i.e.:
            ```
            source /opt/ros/noetic/setup.bash
            mkdir -p /tmp/prerelease_job
            cd /tmp/prerelease_job
            generate_prerelease_script.py \
              https://raw.githubusercontent.com/ros-infrastructure/ros_buildfarm_config/production/index.yaml \
              noetic default ubuntu focal amd64 \
              --custom-repo \
                sick_scan_xd:git:https://github.com/SICKAG/sick_scan_xd:master \
                msgpack11:git:https://github.com/SICKAG/msgpack11:master \
                libsick_ldmrs:git:https://github.com/SICKAG/libsick_ldmrs:master \
              --level 1 \
              --output-dir ./        
            ```
        * Run `printf "\033c" ; rm -rf ~/.ccache ; mkdir -p ~/.ccache ; ./prerelease.sh` in folder `/tmp/prerelease_job`
        * In case of error message `/usr/lib/ccache/cc is not able to compile a simple test program`:
            * Remove folder `~/.ccache` before running `./prerelease.sh`
            * See https://answers.ros.org/question/347063/error-pre-release-melodic/
        * Fix any errors during the prerelease build and check in
        * Remove the temporary build folder by `rm -rf /tmp/prerelease_job`
        * Repeat until `prerelease.sh` finishes without errors.

3. Submit package sick_scan_xd for indexing (noetic)
    * Fork `https://github.com/ros/rosdistro` -> `https://github.com/<username>/rosdistro.git`
    * `git clone https://github.com/<username>/rosdistro.git`
    * Edit file `rosdistro/noetic/distribution.yaml` and add after `sick_scan`:
        ```
        sick_scan_xd:
          doc:
            type: git
            url: https://github.com/SICKAG/sick_scan_xd.git
            version: master
        ```
    * `cd rosdistro ; source /opt/ros/noetic/setup.bash ; rosdistro_reformat file://"$(pwd)"/index.yaml`
    * git commit: `git commit -m "Adding sick_scan_xd to documentation index for distro noetic" distribution.yaml`
    * git push: `git push origin master`
    * Submit a pull request on `https://github.com/<username>/rosdistro`

4. For ROS-2 humble: Follow instructions on https://docs.ros.org/en/humble/How-To-Guides/Releasing/Releasing-a-Package.html
    * Note: Bloom releases for ros2 foxy are not longer supported (Pull request failed, "This pull request changes files for a ROS distribution that is no longer supported (End Of Life)")
    * Submit package sick_scan_xd for indexing (ros2 humble)
        * Reset fork `https://github.com/<username>/rosdistro.git` to origin/master or delete the fork and create a new one -> `https://github.com/<username>/rosdistro.git`
        * `git clone https://github.com/<username>/rosdistro.git`
        * Edit file `rosdistro/humble/distribution.yaml` and add after `sick_safevisionary_ros2`:
            ```
            sick_scan_xd:
              doc:
                type: git
                url: https://github.com/SICKAG/sick_scan_xd.git
                version: develop
              status: developed
            ```
        * git commit and push ("Adding sick_scan_xd to documentation index for distro humble")
        * Submit a pull request on `https://github.com/<username>/rosdistro`
        * Do the same for any new ROS2 version, e.g. iron and jazzy (`rosdistro/iron/distribution.yaml`, `rosdistro/jazzy/distribution.yaml`)
    * [Start a new release team](https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=new_release_team.md&title=Add+release+team)
        * ROS-2 sick_scan_xd team: https://github.com/orgs/ros2-gbp/teams/sick_scan_xd
        * ROS-2 sick_scan_xd release repository: https://github.com/ros2-gbp/sick_scan_xd-release
    
## Release build for ROS-1 noetic

* Build a prerelease (dry run in a docker container):
    * Run the following commands:
        ```
        git clone -b master https://github.com/SICKAG/sick_scan_xd.git
        cd ./sick_scan_xd/test/scripts
        sudo dos2unix ./*.bash ; sudo chmod a+x ./*.bash
        ./run_linux_ros1_bloom.bash
        ```
    * Fix any errors during the prerelease build and check in
    * Repeat `./run_linux_ros1_bloom.bash` until the the prerelease build finishes without errors

* Build a binary release: follow https://wiki.ros.org/bloom/Tutorials/FirstTimeRelease
    * Update version number in package.xml, minor version number should be incremented at least
    * Create resp. update CHANGELOG.rst:
        ```
        source /opt/ros/noetic/setup.bash
        cd ./src/sick_scan_xd
        rm ./CHANGELOG.rst
        catkin_generate_changelog --all # create CHANGELOG.rst
        ```
    * Commit and pull all changes incl. CHANGELOG.rst and package.xml:
        ```
        git add CHANGELOG.rst package.xml
        git commit -m "Update CHANGELOG.rst and package version"
        git push    
        ```    
    * Run `catkin_prepare_release` and `bloom-release` in folder `src/sick_scan_xd`:
        ```
        source /opt/ros/noetic/setup.bash
        catkin_prepare_release -y
        bloom-release --rosdistro noetic --track noetic sick_scan_xd # at first time: call with option --edit for configuration
        ```
    * For the initial release (first time): Run `bloom-release` in folder `src/sick_scan_xd` with option `--edit`:
        ```
        source /opt/ros/noetic/setup.bash
        catkin_prepare_release -y
        bloom-release --rosdistro noetic --track noetic sick_scan_xd --edit
        Release repository url: https://github.com/SICKAG/sick_scan_xd-release.git
        upstream: <default, i.e. press ENTER>
        Upstream Repository URI: https://github.com/SICKAG/sick_scan_xd.git
        Upstream VCS Type: <default: git, i.e. press ENTER>
        Version: <default: auto, i.e. press ENTER>
        Release Tag: <default: version, i.e. press ENTER>
        Upstream Devel Branch: feature/bloom_pretest
        ROS Distro: noetic
        Patches Directory: <default: none, i.e. press ENTER>
        Release Repository Push URL:  <default: none, i.e. press ENTER>
        ```
    * Check status: https://index.ros.org/p/sick_scan_xd/#noetic
    * Install binary release: `sudo apt update ; sudo apt-get install ros-noetic-sick-scan-xd`. Note from https://wiki.ros.org/bloom/Tutorials/FirstTimeRelease: Packages built are periodically synchronized over to the shadow-fixed and public repositories, so it might take as long as a month before your package is available on the public ROS debian repositories (i.e. available via apt-get). 

## Release build for ROS-2

For ROS-2 follow the instructions on https://docs.ros.org/en/humble/How-To-Guides/Releasing/Releasing-a-Package.html:
* Checkout the sick_scan_xd version to be released and run:
    ```
    git clone -b master https://github.com/SICKAG/sick_scan_xd.git
    cd ./sick_scan_xd
    rm ./CHANGELOG.rst
    catkin_generate_changelog --all # create CHANGELOG.rst
    ```
* Commit CHANGELOG.rst and optional modifications:
    ```
    git add CHANGELOG.rst
    git commit -m "Update CHANGELOG.rst"
    git push    
    ```
* Run `catkin_prepare_release` and `bloom-release`:
    ```
    bloom-release --rosdistro humble --track humble sick_scan_xd # at first time: call with option --new-track
    ```
    For the initial release (i.e. at the first time): Run bloom-relase configuration with option --new-track:
    `bloom-release --new-track --rosdistro humble --track humble sick_scan_xd`
    * Release repository url: https://github.com/ros2-gbp/sick_scan_xd-release.git
    * Upstream: <default>
    * Upstream Repository URI: https://github.com/SICKAG/sick_scan_xd.git
    * Upstream Devel Branch: develop
    * ROS Distro: humble
    After the initial release has been approved: Run
    ```
    sudo rosdep init
    rosdep update
    ```
    
## Check status

* ROS-1 release repository: https://github.com/SICKAG/sick_scan_xd-release
* ROS-2 release repository: https://github.com/ros2-gbp/sick_scan_xd-release.git
* ROS-1 jenkins build status: https://build.ros.org/job/Ndev__sick_scan_xd__ubuntu_focal_amd64/lastBuild/
* ROS-2 jenkins build status: https://build.ros2.org/job/Hdev__sick_scan_xd__ubuntu_jammy_amd64/lastBuild/
* ROS-1 jenkins: https://build.ros.org/search/?q=sick_scan_xd
* ROS-2 jenkins: https://build.ros2.org/search/?q=sick_scan_xd
* Documentation: https://index.ros.org/p/sick_scan_xd/#noetic , http://wiki.ros.org/sick_scan_xd
   
## Troubleshooting, FAQ:

* Bloom builds an old sick_scan_xd version (ROS1):
    * Check `devel_branch` in https://github.com/SICKAG/sick_scan_xd-release/blob/master/tracks.yaml. If devel_branch is an old branch, replace it with e.g. `develop` or `master`, or update the `<devel_branch>` to a new version.

* Bloom builds an old sick_scan_xd version (ROS2):
    * Check `devel_branch` in https://github.com/ros2-gbp/sick_scan_xd-release/blob/master/tracks.yaml. If devel_branch is an old branch, replace it with e.g. `develop` or `master`, or update the `<devel_branch>` to a new version.

* Bloom builds a new sick_scan_xd version, but apt still installs an old version:
    * Check the sick_scan_xd version in the release repositories https://github.com/SICKAG/sick_scan_xd-release.git (ROS1) and https://github.com/ros2-gbp/sick_scan_xd-release.git (ROS2)
    * Install bloom (if not yet done) using `sudo apt-get install python-bloom` on Linux or `pip install -U bloom` on Windows
    * Run
        ```
        bloom-release --rosdistro noetic -d sick_scan_xd # release repository: https://github.com/SICKAG/sick_scan_xd-release.git, argument -d enables debug infos 
        bloom-release --rosdistro humble -d sick_scan_xd # release repository: https://github.com/ros2-gbp/sick_scan_xd-release.git, argument -d enables debug infos 
        bloom-release --rosdistro iron   -d sick_scan_xd # release repository: https://github.com/ros2-gbp/sick_scan_xd-release.git, argument -d enables debug infos 
        bloom-release --rosdistro jazzy  -d sick_scan_xd # release repository: https://github.com/ros2-gbp/sick_scan_xd-release.git, argument -d enables debug infos 
        ```
    * In case of github 2FA errors: Follow http://wiki.ros.org/bloom/Tutorials/GithubManualAuthorization to create a 2FA token and configure the token in file `~/.config/bloom`.


## Useful links and tutorials

* http://wiki.ros.org/bloom
* https://wiki.ros.org/bloom/Tutorials/FirstTimeRelease
* https://docs.ros.org/en/humble/How-To-Guides/Releasing/Releasing-a-Package.html
