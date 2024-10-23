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

python pcap_json_converter.py --pcap_filename=../emulator/scandata/20241022_lms4000_encoder.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20241021_lms4000_encoder.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20241010-tim781-lidinputstate-toggle-1-9.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20241008_tim781_lidinputstate_telegram.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20240909-rms2xxx-field-evaluation.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20240909-rms2xxx-field-evaluation-with-rotating-fan.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20240527-LRS36x0.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20240527-OEM15xx.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20240307-MRS1xxx-default-settings-rssiflag3-angres0.2500-scanfreq50.0.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20240307-MRS1xxx-default-settings-rssiflag3-angres0.1250-scanfreq25.0.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20240307-MRS1xxx-default-settings-rssiflag3-angres0.0625-scanfreq12.5.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20240304-MRS1xxx-default-settings-rssiflag-1.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20240304-MRS1xxx-default-settings-rssiflag-3.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230719_mrs1104_infringement/20230719_mrs1104_ros1_field_eval_activated.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230719_mrs1104_infringement/20230719_mrs1104_infringement_sopaset_activate_deactivate_activate_again.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230719_mrs1104_infringement/20230719_mrs1104_infringement_sopaset_default_run.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230719_mrs1104_infringement/20230719_mrs1104_ros1_LIDinputstate_LIDoutputstate_activated.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230510_tim240.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230112_01_mrs1000_layer_1111_50hz_0.25deg.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230112_02_mrs1000_layer_1000_50hz_0.25deg.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230112_03_mrs1000_layer_0100_50hz_0.25deg.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230112_04_mrs1000_layer_0010_50hz_0.25deg.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230112_05_mrs1000_layer_0001_50hz_0.25deg.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230112_06_mrs1000_layer_0101_50hz_0.25deg.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230112_07_mrs1000_layer_1010_50hz_0.25deg.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230112_08_mrs1000_layer_1111_25hz_0.125deg.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230112_09_mrs1000_layer_0101_25hz_0.125deg.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230112_10_mrs1000_layer_1010_25hz_0.125deg.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230112_11_mrs1000_layer_1111_12.5hz_0.0625deg.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230112_12_mrs1000_layer_0101_12.5hz_0.0625deg.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230112_13_mrs1000_layer_1010_12.5hz_0.0625deg.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230112_14_mrs1000_layer_1000_12.5hz_0.0625deg.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230112_15_mrs1000_layer_0100_12.5hz_0.0625deg.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230112_16_mrs1000_layer_0010_12.5hz_0.0625deg.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20230112_17_mrs1000_layer_0001_12.5hz_0.0625deg.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20221110-LMS1xxx-150hz-0.75deg.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20221110-LMS1xxx-75hz-0.375deg.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20221110-LMS1xxx-37.5hz-0.1875deg.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20221018_rms_1xxx_ascii_rawtarget_object.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20220803_lms511.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20220802_lms111.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20220505_lms511_wireshark_issue49.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20220323_nav350_binary.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20220316-rms1000-ascii.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20220316-rms1000-binary.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20211201_MRS_1xxx_IMU_with_movement.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20211201_RMS_1xxx_start_up.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20210722_102600_tim_781_sick_scan_xd.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20210722_103100_tim_781_sick_scan.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20210722_112100_ldmrs_sick_scan_xd.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20210722_143600_ros2_mrs1104_sick_scan_xd.pcapng
python pcap_json_converter.py --pcap_filename=../emulator/scandata/20210722_145212_ros2_tim7xxx_sick_scan_xd.pcapng
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
