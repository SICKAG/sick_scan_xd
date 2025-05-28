#!/bin/bash

# 
# Run all sick_scan_xd dockertests on Linux (cpp API, python API, ROS1 and ROS2)
# 

# Just prompt for key input
function prompt()
{
  read -n 1 -p "Press any key to continue..."
}

# If a docker image is not already loaded, it is loaded from tar file
function load_docker_image()
{
  dockerimage=$1
  dockerfile=$2
  docker inspect ${dockerimage}  || { 
    docker image load -i ./docker_images/${dockerfile}
  }
}

# Prints a error resp. success message depending on docker_exit_status, prompts in case of error
function check_docker_exit_status()
{
  docker_exit_status=$1
  if [ $docker_exit_status -eq 0 ] ; then 
    echo -e "\n**\n** SUCCESS: all sick_scan_xd docker tests passed\n**\n"
  else 
    echo -e "\n##\n## ERROR: sick_scan_xd docker tests with status FAILED\n##\n"
    sleep 10 # prompt
  fi
}

# Build docker image and install sick_scan_xd for Linux x64, ROS1 and ROS2
function build_docker_image()
{
  dockerimage=$1
  dockerfile=$2
  echo -e "\n**\n** Build docker image ${dockerimage} from dockerfile ${dockerfile}\n**\n"

  # Default: Create a docker-image named "${dockerimage}" from dockerfile "${dockerfile}" with local sources in folder "./src/sick_scan_xd"
  docker build --progress=plain -t ${dockerimage} -f ${dockerfile} .
  # Alternative: Create a docker-image named "${dockerimage}" from dockerfile "./sick_scan_xd/test/docker/dockerfiles/dockerfile_linux_x64_sick_scan_xd" with github sources from https://github.com/SICKAG/sick_scan_xd
  # docker build --build-arg SICK_SCAN_XD_GIT_URL=https://github.com/SICKAG/sick_scan_xd --progress=plain -t ${dockerimage} -f ${dockerfile} .
  # Alternative: Create a docker-image named "${dockerimage}" from dockerfile "./sick_scan_xd/test/docker/dockerfiles/dockerfile_linux_x64_sick_scan_xd" with "apt-get install ros-humble-sick-scan-xd"
  # docker build --build-arg SICK_SCAN_XD_APT_PKG=ros-humble-sick-scan-xd --build-arg SICK_SCAN_XD_BUILD_FROM_SOURCES=0 --progress=plain -t ${dockerimage} -f ${dockerfile} .
  docker image ls ${dockerimage}
  # docker rm $(docker ps -a -q) # optional cleanup: remove all docker container
}

# Run sick_scan_xd docker test
function run_docker_test()
{
  dockerimage=$1
  export_folder=$2
  simu_args=$3
  cfg_files=$4
  docker_status_final=0 # success
  echo -e "\n**\n** Run docker test, dockerimage=\"${dockerimage}\", export_folder=\"./log/${export_folder}\", simu_args=\"${simu_args}\", cfg_files=\"${cfg_files}\"\n**\n"  
  # Run the docker-image named "sick_scan_xd/linux_x64" and execute a multiscan test within the docker container
  # docker run option --rm automatically removes the container when it exits
  for cfg in ${cfg_files} ; do 
    echo -e "\n**\n** Run docker test, dockerimage=\"${dockerimage}\", export_folder=\"./log/${export_folder}\", simu_args=\"${simu_args}\", cfg_file=\"${cfg}\"\n**\n"  
    container_name=sick_scan_xd_$cfg
    container_id=$(docker ps -aqf "name=$container_name") 
    docker rm $container_id # remove previous docker container
    docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY -w /workspace --name $container_name ${dockerimage} python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py ${simu_args} --cfg=./src/sick_scan_xd/test/docker/data/$cfg.json
    docker_exit_status=$?
    echo -e "docker_exit_status for $cfg: $docker_exit_status"
    if [ $docker_exit_status -eq 0 ] ; then 
        echo -e "\nSUCCESS: sick_scan_xd docker test passed for configuration $cfg\n"
    else 
        echo -e "\n## ERROR: sick_scan_xd docker test FAILED for configuration $cfg\n"
        docker_status_final=$docker_exit_status
        sleep 10 # prompt
    fi
    # docker ps -a # list all container
  done  
  # Export sick_scan_xd output folder from docker container to local folder ./log/${export_folder}
  docker ps -a # list all container
  if [ -d ./log/${export_folder}  ] ; then rm -rf ./log/${export_folder}  ; fi
  if [ -d ./log/sick_scan_xd_simu ] ; then rm -rf ./log/sick_scan_xd_simu ; fi
  mkdir -p ./log
  for cfg in  ${cfg_files} ; do 
    container_name=sick_scan_xd_$cfg
    container_id=$(docker ps -aqf "name=$container_name")
    # Export logfiles from container workspace to local folder ./log/sick_scan_xd_simu
    docker cp $container_id:/workspace/log/sick_scan_xd_simu ./log
  done
  # Rename local folder ./log/sick_scan_xd_simu to ./log/${export_folder}
  mv ./log/sick_scan_xd_simu ./log/${export_folder}
  ls -al ./log/${export_folder}
  # Return 0 for success or error code otherwise  
  check_docker_exit_status $docker_status_final
  return $docker_status_final
}

# Create a summary and convert reports to htlm
function create_report()
{
  export_folder=$1
  docker_exit_status=$2
  echo -e "\n**\n** Create docker test report, export_folder=\"./log/${export_folder}\"\n**\n"  
  pushd ./log/${export_folder}
  echo -e "# sick_scan_xd test report summary\n" > sick_scan_xd_testreport.md
  if [ $docker_exit_status -eq 0 ] ; then 
    echo -e "**SUCCESS: all sick_scan_xd docker tests passed**\n" >> sick_scan_xd_testreport.md
  else 
    echo -e "**ERROR: sick_scan_xd docker tests with status FAILED**\n" >> sick_scan_xd_testreport.md
  fi
  for summary in ./*/sick_scan_xd_summary.md ; do (cat $summary >> sick_scan_xd_testreport.md) ; done
  for mdfile in ./*.md ; do pandoc -f markdown -t html -s $mdfile -o $mdfile.html ; done
  for mdfile in ./*/*.md ; do pandoc -f markdown -t html -s $mdfile -o $mdfile.html ; done
  popd
  echo -e "\nlogfiles exported from docker container to local folder ./log/${export_folder}:"
  ls -al ./log/${export_folder}/*
  echo -e "\n"
  cat ./log/${export_folder}/sick_scan_xd_testreport.md
  # firefox ./log/${export_folder}/sick_scan_xd_testreport.md.html &
}

# 
# Init docker tests
# 

printf "\033c"
pushd ../../../..
cp -f ./src/sick_scan_xd/.dockerignore .
if [ -d ./log  ] ; then rm -rf ./log ; fi
xhost +local:docker # allow X11-gui-applications in docker like rviz to display on host, see https://medium.com/intro-to-artificial-intelligence/rviz-on-docker-bdf4d0fca5b

# 
# Build sick_scan_xd for Linux x64, ROS1 and ROS2 in Docker
# 

if [ ! -f ./docker_images/linux_x64_develop.tar         ] ; then docker build --progress=plain -t linux_x64_develop         -f ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_linux_x64_develop .         ; docker image save -o ./docker_images/linux_x64_develop.tar linux_x64_develop                 ; fi
if [ ! -f ./docker_images/linux_ros1_noetic_develop.tar ] ; then docker build --progress=plain -t linux_ros1_noetic_develop -f ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_linux_ros1_noetic_develop . ; docker image save -o ./docker_images/linux_ros1_noetic_develop.tar linux_ros1_noetic_develop ; fi
if [ ! -f ./docker_images/linux_ros2_humble_develop.tar ] ; then docker build --progress=plain -t linux_ros2_humble_develop -f ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_linux_ros2_humble_develop . ; docker image save -o ./docker_images/linux_ros2_humble_develop.tar linux_ros2_humble_develop ; fi
load_docker_image linux_x64_develop         linux_x64_develop.tar
load_docker_image linux_ros1_noetic_develop linux_ros1_noetic_develop.tar
load_docker_image linux_ros2_humble_develop linux_ros2_humble_develop.tar
build_docker_image sick_scan_xd/linux_x64   ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_linux_x64_sick_scan_xd
build_docker_image sick_scan_xd/ros1_noetic ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_linux_ros1_noetic_sick_scan_xd
build_docker_image sick_scan_xd/ros2_humble ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_linux_ros2_humble_sick_scan_xd
docker images -a # list all docker images
docker ps -a # list all container

#
# Run all sick_scan_xd docker tests (cpp API, python API, ROS1, ROS2)
#

run_docker_test sick_scan_xd/linux_x64 sick_scan_xd_simu_cpp_api  "--ros=none --api=cpp"    "multiscan_compact_test01_cfg picoscan_compact_test01_cfg lms1xx_test01_cfg lms1xxx_test01_cfg lms5xx_test01_cfg mrs1xxx_test01_cfg mrs6xxx_test01_cfg nav350_test01_cfg tim240_test01_cfg tim7xx_test01_cfg tim7xxs_test01_cfg lms4xxx_test01_cfg lrs36x0_test01_cfg lrs36x1_test01_cfg lrs4xxx_test01_cfg oem15xx_test01_cfg tim4xx_test01_cfg tim5xx_test01_cfg"
dockertest_cpp_api_exit_status=$?

run_docker_test sick_scan_xd/linux_x64 sick_scan_xd_simu_py_api   "--ros=none --api=python" "multiscan_compact_test01_cfg picoscan_compact_test01_cfg lms1xx_test01_cfg lms1xxx_test01_cfg lms5xx_test01_cfg mrs1xxx_test01_cfg mrs6xxx_test01_cfg nav350_test01_cfg tim240_test01_cfg tim7xx_test01_cfg tim7xxs_test01_cfg lms4xxx_test01_cfg lrs36x0_test01_cfg lrs36x1_test01_cfg lrs4xxx_test01_cfg oem15xx_test01_cfg tim4xx_test01_cfg tim5xx_test01_cfg"
dockertest_py_api_exit_status=$?

run_docker_test sick_scan_xd/ros1_noetic sick_scan_xd_simu_noetic "--ros=noetic --api=none" "multiscan_compact_test01_cfg picoscan_compact_test01_cfg lms1xx_test01_cfg lms1xxx_test01_cfg lms5xx_test01_cfg mrs1xxx_test01_cfg mrs6xxx_test01_cfg nav350_test01_cfg rmsxxxx_test01_cfg tim240_test01_cfg tim7xx_test01_cfg tim7xxs_test01_cfg lms4xxx_test01_cfg lrs36x0_test01_cfg lrs36x1_test01_cfg lrs4xxx_test01_cfg oem15xx_test01_cfg tim4xx_test01_cfg tim5xx_test01_cfg"
dockertest_noetic_exit_status=$?

run_docker_test sick_scan_xd/ros2_humble sick_scan_xd_simu_humble "--ros=humble --api=none" "multiscan_compact_test01_cfg picoscan_compact_test01_cfg lms1xx_test01_cfg lms1xxx_test01_cfg lms5xx_test01_cfg mrs6xxx_test01_cfg nav350_test01_cfg rmsxxxx_test01_cfg tim240_test01_cfg tim7xx_test01_cfg tim7xxs_test01_cfg lms4xxx_test01_cfg lrs36x0_test01_cfg lrs36x1_test01_cfg lrs4xxx_test01_cfg oem15xx_test01_cfg tim4xx_test01_cfg tim5xx_test01_cfg"
dockertest_humble_exit_status=$?

#
# Create summary and convert reports to htlm
#

create_report sick_scan_xd_simu_cpp_api $dockertest_cpp_api_exit_status
create_report sick_scan_xd_simu_py_api  $dockertest_py_api_exit_status
create_report sick_scan_xd_simu_noetic  $dockertest_noetic_exit_status
create_report sick_scan_xd_simu_humble  $dockertest_humble_exit_status

echo -e "\n# Summary for all dockertests\n" > ./log/sick_scan_xd_testreport_summary.md
for export_folder in sick_scan_xd_simu_cpp_api sick_scan_xd_simu_py_api sick_scan_xd_simu_noetic sick_scan_xd_simu_humble ; do 
  cat ./log/${export_folder}/sick_scan_xd_testreport.md >> ./log/sick_scan_xd_testreport_summary.md
done
cat ./log/sick_scan_xd_testreport_summary.md
# firefox ./log/sick_scan_xd_simu_cpp_api/sick_scan_xd_testreport.md.html ./log/sick_scan_xd_simu_py_api/sick_scan_xd_testreport.md.html ./log/sick_scan_xd_simu_noetic/sick_scan_xd_testreport.md.html ./log/sick_scan_xd_simu_humble/sick_scan_xd_testreport.md.html &
gedit ./log/sick_scan_xd_testreport_summary.md ./log/sick_scan_xd_simu_cpp_api/sick_scan_xd_testreport.md ./log/sick_scan_xd_simu_py_api/sick_scan_xd_testreport.md ./log/sick_scan_xd_simu_noetic/sick_scan_xd_testreport.md ./log/sick_scan_xd_simu_humble/sick_scan_xd_testreport.md &

#
# Return overall dockertest status (0=success, otherwise error)
#

popd
docker_status_final=0 # success
if [ $dockertest_cpp_api_exit_status -eq 0 ] && [ $dockertest_py_api_exit_status -eq 0 ] && [ $dockertest_noetic_exit_status -eq 0 ] && [ $dockertest_humble_exit_status -eq 0 ] ; then 
  docker_status_final=0 # success
else
  docker_status_final=1 # error
fi
check_docker_exit_status $docker_status_final
exit $docker_status_final
