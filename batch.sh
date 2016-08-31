#!/bin/sh
# Batch of instructions to be sent to Dynamixel devices
DXL_PORTNAME=/dev/ttyUSB0
DXL_BAUDRATE=1000000
DXL_STARTID=1
DXL_NBID=1
DXL_PROTO=2

# Initialization sequence

# Loop
while [ 1 ]
do
	./dxl write $DXL_PORTNAME $DXL_BAUDRATE $DXL_STARTID $DXL_NBID $DXL_PROTO 65 1 0
	sleep 0.1
	./dxl write $DXL_PORTNAME $DXL_BAUDRATE $DXL_STARTID $DXL_NBID $DXL_PROTO 65 1 1
	sleep 0.1
	./dxl read $DXL_PORTNAME $DXL_BAUDRATE $DXL_STARTID $DXL_NBID $DXL_PROTO 120 2 0
done
