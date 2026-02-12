## **Run a ROS 2 prerelease test locally (without Docker)**

### **1. Create and activate a Python virtual environment**
Keep the ROS build farm tools isolated to avoid polluting your system packages.

```bash
cd /tmp
python3 -m venv ros_prerelease_env
source ros_prerelease_env/bin/activate
```

### 2. Install required Python dependencies
Install the tools necessary for generating and running the prerelease script locally:

```bash
pip install -U pip setuptools
pip install ros_buildfarm rosdep rosdistro rosdistro-modules catkin_pkg colcon-common-extensions
```
These provide:

* ros_buildfarm → prerelease script generator

* rosdep / rosdistro → manage ROS dependencies

* colcon → build and test ROS 2 workspaces

* catkin_pkg → parse ROS package metadata

### 3. Generate the prerelease script
Use generate_prerelease_script.py from ros_buildfarm to create a local test script.
```
generate_prerelease_script.py \
  https://raw.githubusercontent.com/ros2/ros_buildfarm_config/ros2/index.yaml \
  kilted default ubuntu noble amd64 \
  --pkg sick_scan_xd \
  --custom-repo sick_scan_xd:git:https://github.com/SICKAG/sick_scan_xd.git:feature/bloom_prerelease_test \
  --output-dir ./prerelease_kilted_feature_test \
  --no-docker
```
The `--no-docker` flag tells the tool to produce a local build script instead of one that uses Docker.

### 4. Inspect and run the generated script
The command above creates a directory like:

```
./prerelease_kilted_feature_test/
```

Inside it, you’ll find:

* prerelease.sh — the main build/test script configuration files for the ROS 2 build farm environment

Run it directly on your host system:

```
cd prerelease_kilted_feature_test
./prerelease.sh
```

If the script is not executable:

```
chmod +x prerelease.sh
./prerelease.sh
```

### 5. What happens during execution
The script performs the following:

* Creates a temporary ROS 2 workspace.

* Downloads the specified repositories (using your --custom-repo branch).

* Runs rosdep install to resolve dependencies.

* Builds all packages using colcon build.

* Executes tests via colcon test and reports results.

### 6. Review results
After the script finishes:

Successful builds will appear as ✅ messages.

Test failures are reported with ❌ and logs stored in log/latest_test/ or build/.

You can inspect them, for example:

```
colcon test-result --verbose
```

### 7. (Optional) Clean up
When done, you can deactivate the environment and remove the temporary test directory:
```
deactivate
rm -rf /tmp/ros_prerelease_env ./prerelease_kilted_feature_test
```
