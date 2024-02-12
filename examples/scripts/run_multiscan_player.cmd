REM 
REM Run pcapng player with multiscan pcapng-file in an endless loop
REM debug option: --send_rate=10
REM 

for /l %%n in (1,1,999) do (
    @echo Running multiscan_pcap_player for %%n. time ...
    python ../../test/python/multiscan_pcap_player.py --pcap_filename=../../test/emulator/scandata/20231009-multiscan-compact-imu-01.pcapng --udp_port=-1 --filter=pcap_filter_multiscan_hildesheim
)
