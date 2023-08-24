REM 
REM Example call for multiscan_perftest_player.py
REM 

python --version
python multiscan_perftest_player.py --dst_ip=192.168.1.27 --udp_port=2115 --repeat=100 --send_rate=0 --force_delay=5.0e-4 --verbose=0 --prompt=0
@pause
