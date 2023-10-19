del /f/q *.png
python ../sickag/imu_delay_tester.py --csv_filename=20231009_multiscan_timestamp_azimuth_imuacceleration.csv
python ../sickag/imu_delay_tester.py --csv_filename=20231011a_multiscan_timestamp_azimuth_imuacceleration.csv
python ../sickag/imu_delay_tester.py --csv_filename=20231011b_multiscan_timestamp_azimuth_imuacceleration.csv
python ../sickag/imu_delay_tester.py --csv_filename=20231011c_multiscan_timestamp_azimuth_imuacceleration.csv
@pause
