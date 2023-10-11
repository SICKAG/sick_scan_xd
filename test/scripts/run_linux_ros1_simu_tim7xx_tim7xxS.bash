#!/bin/bash
printf "\033c"
pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash     ] ; then source /opt/ros/melodic/setup.bash   ; fi
if [ -f /opt/ros/noetic/setup.bash      ] ; then source /opt/ros/noetic/setup.bash    ; fi
if [ -f ./devel_isolated/setup.bash     ] ; then source ./devel_isolated/setup.bash   ; fi
# if [ -f ./install_isolated/setup.bash ] ; then source ./install_isolated/setup.bash ; fi
# if [ -f ./install/setup.bash          ] ; then source ./install/setup.bash          ; fi

for sick_scan_launch_file in sick_tim_7xx.launch sick_tim_7xxS.launch ; do
  echo -e "run_simu_tim7xx_tim7xxS.bash: starting TiM7xx/TiM7xxS emulation with  $sick_scan_launch_file\n"
  
  # Start roscore if not yet running
  roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
  if [ $roscore_running -lt 1 ] ; then 
    roscore &
    sleep 3
  fi
  
  for emulator_launch_cfg in emulator_01_default.launch emulator_03_2nd_fieldset.launch emulator_02_toggle_fieldsets.launch ; do # emulator_04_fieldset_test.launch
    echo -e "Starting TiM7xx/TiM7xxS emulation $emulator_launch_cfg\n"
    
    # Start sick_scan_xd emulator
    roslaunch sick_scan_xd $emulator_launch_cfg &
    sleep 1
    
    # Start rviz
    # Note: Due to a bug in opengl 3 in combination with rviz and VMware, opengl 2 should be used by rviz option --opengl 210
    # See https://github.com/ros-visualization/rviz/issues/1444 and https://github.com/ros-visualization/rviz/issues/1508 for further details
    rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg.rviz --opengl 210 &
    sleep 1
    
    # Start sick_scan_xd driver for TiM7xx/TiM7xxS
    echo -e "Launching sick_scan_xd $sick_scan_launch_file\n"
    roslaunch sick_scan_xd $sick_scan_launch_file hostname:=127.0.0.1 &
    sleep 1
    
    # rosservice call /sick_tim_7xx/SCdevicestate "{}" # query device state
    # rosservice call /sick_tim_7xx/ColaMsg "{request: 'sEN LIDoutputstate 1'}"
    # rosservice call /sick_tim_7xx/ColaMsg "{request: 'sRN LIDoutputstate'}"
    # rosservice call /sick_tim_7xx/ColaMsg "{request: 'sEN LFErec 1'}"
    # rosservice call /sick_tim_7xx/ColaMsg "{request: 'sRN LFErec'}"
    # rosservice call /sick_tim_7xx/ColaMsg "{request: 'sEN field000 1'}"
    # rosservice call /sick_tim_7xx/ColaMsg "{request: 'sRN field000'}"
    # rosservice call /sick_tim_7xx/ColaMsg "{request: 'sEN LIDinputstate 1'}"
    # rosservice call /sick_tim_7xx/ColaMsg "{request: 'sRN LIDinputstate'}" # response: "sRA LIDinputstate \\x00\\x00\\x00\\x00\\x00\\x00\\x01\\x00\\x00\\x00\\x00\\x00"
    # rostopic echo "/sick_tim_7xxS/lferec" &
    # rostopic echo "/sick_tim_7xxS/lidoutputstate" &
    
    # Wait for 'q' or 'Q' to exit or until rviz is closed
    for i in {1..60}; do # while true ; do  
      echo -e "TiM7xx/TiM7xxS emulation running ($sick_scan_launch_file, $emulator_launch_cfg). Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
      if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
      rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
      if [ $rviz_running -lt 1 ] ; then break ; fi
    done
    
    # Shutdown
    echo -e "Finishing TiM7xx/TiM7xxS emulation $emulator_launch_cfg, shutdown ros nodes\n"
    pkill rviz
    rosnode kill -a ; sleep 1
    killall sick_generic_caller ; sleep 1
    killall sick_scan_emulator ; sleep 1

  done
  killall rosmaster ; sleep 1
  echo -e "run_simu_tim7xx_tim7xxS.bash: TiM7xx/TiM7xxS emulation finished.\n\n"

done
echo -e "run_simu_tim7xx_tim7xxS.bash: TiM7xx/TiM7xxS emulation finished.\n\n"

popd

