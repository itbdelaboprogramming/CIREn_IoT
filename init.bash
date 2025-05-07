
# Check for HAT initialization
is_CanHAT_init=false
if dmesg | grep -q "MCP2515 successfully initialized"; then
    is_CanHAT_init=true
else
    is_CanHAT_init=false
fi





sudo ip link set can0 up type can bitrate 1000000
sudo ifconfig can0 txqueuelen 65536
sudo ifconfig can0 up
