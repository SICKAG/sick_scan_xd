#!/bin/bash

# Shutdown simulation, kill all nodes and processes
function kill_simu()
{
    echo -e "Finishing field monitoring test, shutdown ros nodes\n"
    rosnode kill -a ; sleep 1
    killall sick_generic_caller ; sleep 1
    killall sick_scan_emulator ; sleep 1
}

# Wait for max 30 seconds, or until 'q' or 'Q' pressed, or until rviz is closed
function waitUntilRvizClosed()
{
    duration_sec=$1
    sleep 1
    for ((cnt=1;cnt<=$duration_sec;cnt++)) ; do  
        echo -e "sick_scan_xd running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
        if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
        rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
        if [ $rviz_running -lt 1 ] ; then break ; fi
    done
    kill_simu
}

printf "\033c"
pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash   ] ; then source /opt/ros/melodic/setup.bash   ; fi
if [ -f /opt/ros/noetic/setup.bash    ] ; then source /opt/ros/noetic/setup.bash    ; fi
if [ -f ./devel_isolated/setup.bash   ] ; then source ./devel_isolated/setup.bash   ; fi

# Start roscore if not yet running
roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
if [ $roscore_running -lt 1 ] ; then 
  roscore &
  sleep 3
fi

#
# Run LMS_1xx, LMS_5xx and TIM_7xx with field monitoring
#
kill_simu
echo -e "\nrun_linux_ros1_simu_field_monitoring.bash: starting field monitoring test\n"

# Run sick_lms_5xx with 20220803_lms511.pcapng.json (lms511 reference file, see sopas screenshots 20220803_lms511_fields*.png)
roslaunch sick_scan_xd emulator_lms5xx.launch scanner_type:=sick_lms_511 scandatafiles:=`(pwd)`/src/sick_scan_xd/test/emulator/scandata/20220803_lms511.pcapng.json &
sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_lms5xx.rviz --opengl 210 &
sleep 1 ; roslaunch sick_scan_xd sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
waitUntilRvizClosed 30

# Run sick_lms_1xx with 20210302_lms111.pcapng_full.json
roslaunch sick_scan_xd emulator_lms1xx.launch scanner_type:=sick_lms_1xx scandatafiles:=`(pwd)`/src/sick_scan_xd/test/emulator/scandata/20210302_lms111.pcapng_full.json &
sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_lms1xx.rviz --opengl 210 &
sleep 1 ; roslaunch sick_scan_xd sick_lms_1xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
waitUntilRvizClosed 30

# Run sick_lms_1xx with 20220802_lms111.pcapng.json
roslaunch sick_scan_xd emulator_lms111.launch scanner_type:=sick_lms_111 scandatafiles:=`(pwd)`/src/sick_scan_xd/test/emulator/scandata/20220802_lms111.pcapng.json &
sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_lms1xx.rviz --opengl 210 &
sleep 1 ; roslaunch sick_scan_xd sick_lms_1xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
waitUntilRvizClosed 40

# Run sick_lms_5xx with 20210301_lms511.pcapng_full.json
roslaunch sick_scan_xd emulator_lms5xx.launch scanner_type:=sick_lms_5xx scandatafiles:=`(pwd)`/src/sick_scan_xd/test/emulator/scandata/20210301_lms511.pcapng_full.json &
sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_lms5xx.rviz --opengl 210 &
sleep 1 ; roslaunch sick_scan_xd sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
waitUntilRvizClosed 30

# Run sick_lms_5xx with 20210302_lms511.pcapng_full.json
roslaunch sick_scan_xd emulator_lms5xx.launch scanner_type:=sick_lms_5xx scandatafiles:=`(pwd)`/src/sick_scan_xd/test/emulator/scandata/20210302_lms511.pcapng_full.json &
sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_lms5xx.rviz --opengl 210 &
sleep 1 ; roslaunch sick_scan_xd sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
waitUntilRvizClosed 30

# Run sick_tim_7xx
# for jsonfile in sopas_et_field_test_1_2_both_010.pcapng_full.json 20210113_tim871s_elephant_full.pcapng.json fieldset_trial_0001.pcapng.json 20210125-tim781s-scandata.pcapng.json 20210126-tim781s-test-fieldsets.pcapng.json 20210104_tim781s_sopas_et_binary_monitoring.pcapng.json 20210106_tim781s_scandata_elephant.pcapng.json 20210722_102600_tim_781_sick_scan_xd.pcapng.json 20210722_103100_tim_781_sick_scan.pcapng.json 20210722_145212_ros2_tim7xxx_sick_scan_xd.pcapng.json ; do
for jsonfile in sopas_et_field_test_1_2_both_010.pcapng_full.json 20210113_tim871s_elephant_full.pcapng.json ; do
  roslaunch sick_scan_xd emulator_01_default.launch scanner_type:=sick_tim_7xx scandatafiles:=`(pwd)`/src/sick_scan_xd/test/emulator/scandata/$jsonfile &
  sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg.rviz --opengl 210 &
  sleep 1 ; roslaunch sick_scan_xd sick_tim_7xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
  waitUntilRvizClosed 30
done

echo -e "\nrun_linux_ros1_simu_field_monitoring.bash finished.\n"
popd

