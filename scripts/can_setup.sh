echo Starting CAN...

sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000