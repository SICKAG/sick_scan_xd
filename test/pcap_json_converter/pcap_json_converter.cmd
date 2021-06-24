@if exist "c:/dev/ros2_foxy/local_setup.bat" ( call C:/dev/ros2_foxy/local_setup.bat )
@if exist %ProgramFiles(x86)%/Microsoft Visual Studio/Shared/Python36_64 (
  set PYTHON_DIR=%ProgramFiles(x86)%/Microsoft Visual Studio/Shared/Python36_64
  set PATH=%PYTHON_DIR%;%PYTHON_DIR%/Scripts;%PATH%
)
@if exist %ProgramFiles(x86)%/Microsoft Visual Studio/Shared/Python37_64 (
  set PYTHON_DIR=%ProgramFiles(x86)%/Microsoft Visual Studio/Shared/Python37_64
  set PATH=%PYTHON_DIR%;%PYTHON_DIR%/Scripts;%PATH%
)
@echo.
echo PATH=%PATH%
echo PYTHON_DIR=%PYTHON_DIR%
python --version
@echo.

python pcap_json_converter.py --pcap_filename=q:/tmp/pa0350_sick_field_monitoring/10_info_material/LMS511_fields/20210302_lms111.pcapng
python pcap_json_converter.py --pcap_filename=q:/tmp/pa0350_sick_field_monitoring/10_info_material/LMS511_fields/20210302_lms511.pcapng
python pcap_json_converter.py --pcap_filename=q:/tmp/pa0350_sick_field_monitoring/10_info_material/LMS511_fields/ascii_sopaslog_fields_lms511.pcapng
python pcap_json_converter.py --pcap_filename=q:/tmp/pa0350_sick_field_monitoring/10_info_material/40_inputstate_debugging/20210126-test-fieldsets.pcapng
python pcap_json_converter.py --pcap_filename=q:/tmp/pa0350_sick_field_monitoring/10_info_material/40_inputstate_debugging/20210125-tim781s-scandata.pcapng
python pcap_json_converter.py --pcap_filename=q:/tmp/pa0350_sick_field_monitoring/10_info_material/40_inputstate_debugging/fieldset_trial_0000.pcapng
python pcap_json_converter.py --pcap_filename=q:/tmp/pa0350_sick_field_monitoring/10_info_material/40_inputstate_debugging/fieldset_trial_0001.pcapng
python pcap_json_converter.py --pcap_filename=q:/tmp/pa0350_sick_field_monitoring/10_info_material/20_linux_ros1_tim781s/20210113_tim871s_elephant.pcapng
python pcap_json_converter.py --pcap_filename=q:/tmp/pa0350_sick_field_monitoring/10_info_material/20_linux_ros1_tim781s/20210111_sick_tim781s_mon_elephant.pcapng
python pcap_json_converter.py --pcap_filename=q:/tmp/pa0350_sick_field_monitoring/10_info_material/20_linux_ros1_tim781s/20210111_sick_tim781s_srn_elephant.pcapng
python pcap_json_converter.py --pcap_filename=q:/tmp/pa0350_sick_field_monitoring/10_info_material/20_linux_ros1_tim781s/20210111_sick_tim781s_lferec_elephant.pcapng
python pcap_json_converter.py --pcap_filename=q:/tmp/pa0350_sick_field_monitoring/10_info_material/20_linux_ros1_tim781s/20210106_tim781s_scandata_elephant_1.pcapng
python pcap_json_converter.py --pcap_filename=q:/tmp/pa0350_sick_field_monitoring/10_info_material/20_linux_ros1_tim781s/20210106_tim781s_scandata_elephant_2.pcapng
python pcap_json_converter.py --pcap_filename=q:/tmp/pa0350_sick_field_monitoring/10_info_material/20_linux_ros1_tim781s/000_linux_sick_scan_tim781s_startup.pcapng
python pcap_json_converter.py --pcap_filename=q:/tmp/pa0350_sick_field_monitoring/10_info_material/10_sopas_et/001_sopas_et_binary_startup.pcapng
python pcap_json_converter.py --pcap_filename=q:/tmp/pa0350_sick_field_monitoring/10_info_material/10_sopas_et/002_sopas_et_binary_login.pcapng
python pcap_json_converter.py --pcap_filename=q:/tmp/pa0350_sick_field_monitoring/10_info_material/10_sopas_et/002_sopas_et_binary_read_parameter.pcapng
python pcap_json_converter.py --pcap_filename=q:/tmp/pa0350_sick_field_monitoring/10_info_material/10_sopas_et/004_sopas_et_binary_monitoring.pcapng
python pcap_json_converter.py --pcap_filename=q:/tmp/pa0350_sick_field_monitoring/10_info_material/10_sopas_et/sopas_et_binary_startup_005.pcapng
python pcap_json_converter.py --pcap_filename=q:/tmp/pa0350_sick_field_monitoring/10_info_material/10_sopas_et/sopas_et_field_test_1_2_both_010.pcapng

@pause
