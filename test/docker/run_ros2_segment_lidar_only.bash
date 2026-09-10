#!/bin/bash

#
# Run sick_scan_xd ROS2 Humble docker tests
# Tests:
#   - multiscan_compact_test01_cfg
#   - picoscan_compact_test01_cfg
#

function load_docker_image()
{
  dockerimage=$1
  dockerfile=$2

  docker inspect ${dockerimage} || {
    docker image load -i ./docker_images/${dockerfile}
  }
}


function check_docker_exit_status()
{
  docker_exit_status=$1

  if [ $docker_exit_status -eq 0 ] ; then
    echo -e "\n**\n** SUCCESS: all sick_scan_xd ROS2 docker tests passed\n**\n"
  else
    echo -e "\n##\n## ERROR: sick_scan_xd ROS2 docker tests FAILED\n##\n"
    sleep 10
  fi
}


function build_docker_image()
{
  dockerimage=$1
  dockerfile=$2

  echo -e "\n**\n** Build docker image ${dockerimage} from dockerfile ${dockerfile}\n**\n"

  docker build \
    --progress=plain \
    -t ${dockerimage} \
    -f ${dockerfile} .

  docker image ls ${dockerimage}
}


function run_docker_test()
{
  dockerimage=$1
  export_folder=$2
  simu_args=$3
  cfg_files=$4

  docker_status_final=0

  echo -e "\n**\n** Run ROS2 docker tests"
  echo -e "** dockerimage=\"${dockerimage}\""
  echo -e "** export_folder=\"./log/${export_folder}\""
  echo -e "** simu_args=\"${simu_args}\""
  echo -e "** cfg_files=\"${cfg_files}\"\n**\n"

  for cfg in ${cfg_files} ; do

    echo -e "\n**\n** Run docker test: ${cfg}\n**\n"

    container_name=sick_scan_xd_$cfg
    container_id=$(docker ps -aqf "name=$container_name")

    # Remove previous container if it exists
    if [ -n "$container_id" ]; then
      docker rm $container_id
    fi

    docker run -it \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      --env=DISPLAY \
      -w /workspace \
      --name $container_name \
      ${dockerimage} \
      python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py \
      ${simu_args} \
      --cfg=./src/sick_scan_xd/test/docker/data/$cfg.json

    docker_exit_status=$?

    echo -e "docker_exit_status for $cfg: $docker_exit_status"

    if [ $docker_exit_status -eq 0 ] ; then
      echo -e "\nSUCCESS: sick_scan_xd ROS2 docker test passed for $cfg\n"
    else
      echo -e "\n## ERROR: sick_scan_xd ROS2 docker test FAILED for $cfg\n"
      docker_status_final=$docker_exit_status
      sleep 10
    fi

  done


  #
  # Export log files
  #

  docker ps -a

  if [ -d ./log/${export_folder} ]; then
    rm -rf ./log/${export_folder}
  fi

  if [ -d ./log/sick_scan_xd_simu ]; then
    rm -rf ./log/sick_scan_xd_simu
  fi

  mkdir -p ./log

  for cfg in ${cfg_files} ; do

    container_name=sick_scan_xd_$cfg
    container_id=$(docker ps -aqf "name=$container_name")

    if [ -n "$container_id" ]; then
      docker cp \
        $container_id:/workspace/log/sick_scan_xd_simu \
        ./log
    fi

  done

  mv ./log/sick_scan_xd_simu ./log/${export_folder}

  ls -al ./log/${export_folder}

  check_docker_exit_status $docker_status_final

  return $docker_status_final
}


function create_report()
{
  export_folder=$1
  docker_exit_status=$2

  echo -e "\n**\n** Create ROS2 docker test report"
  echo -e "** export_folder=\"./log/${export_folder}\"\n**\n"

  pushd ./log/${export_folder}

  echo -e "# sick_scan_xd ROS2 test report summary\n" \
    > sick_scan_xd_testreport.md

  if [ $docker_exit_status -eq 0 ] ; then

    echo -e "**SUCCESS: all sick_scan_xd ROS2 docker tests passed**\n" \
      >> sick_scan_xd_testreport.md

  else

    echo -e "**ERROR: sick_scan_xd ROS2 docker tests FAILED**\n" \
      >> sick_scan_xd_testreport.md

  fi

  for summary in ./*/sick_scan_xd_summary.md ; do
    cat $summary >> sick_scan_xd_testreport.md
  done

  for mdfile in ./*.md ; do
    pandoc -f markdown -t html -s $mdfile -o $mdfile.html
  done

  for mdfile in ./*/*.md ; do
    pandoc -f markdown -t html -s $mdfile -o $mdfile.html
  done

  popd

  echo -e "\nLogfiles exported to:"
  echo -e "./log/${export_folder}\n"

  ls -al ./log/${export_folder}/*

  echo -e "\n"

  cat ./log/${export_folder}/sick_scan_xd_testreport.md
}


#
# Init
#

printf "\033c"

pushd ../../../..

cp -f ./src/sick_scan_xd/.dockerignore .

if [ -d ./log ]; then
  rm -rf ./log
fi

# Allow X11 applications inside docker
xhost +local:docker


#
# Build/load ROS2 Humble base docker image
#

if [ ! -f ./docker_images/linux_ros2_humble_develop.tar ]; then

  docker build \
    --progress=plain \
    -t linux_ros2_humble_develop \
    -f ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_linux_ros2_humble_develop .

  docker image save \
    -o ./docker_images/linux_ros2_humble_develop.tar \
    linux_ros2_humble_develop

fi


load_docker_image \
  linux_ros2_humble_develop \
  linux_ros2_humble_develop.tar


#
# Build sick_scan_xd ROS2 Humble docker image
#

build_docker_image \
  sick_scan_xd/ros2_humble \
  ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_linux_ros2_humble_sick_scan_xd


docker images -a
docker ps -a


#
# Run ROS2 Humble docker tests
#
# Only:
#   multiscan_compact_test01_cfg
#   picoscan_compact_test01_cfg
#

run_docker_test \
  sick_scan_xd/ros2_humble \
  sick_scan_xd_simu_humble \
  "--ros=humble --api=none" \
  "multiscan_compact_test01_cfg picoscan_compact_test01_cfg"

dockertest_humble_exit_status=$?


#
# Create report
#

create_report \
  sick_scan_xd_simu_humble \
  $dockertest_humble_exit_status


#
# Return docker test status
#

popd

check_docker_exit_status $dockertest_humble_exit_status

exit $dockertest_humble_exit_status
