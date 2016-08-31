#!/bin/sh
# Example script for XM430

## Script initialization
# Non blocking stdin
if [ -t 0 ]; then stty -echo -icanon -icrnl time 0 min 0; fi
# Batch of instructions to be sent to Dynamixel devices
DXL_PORTNAME=/dev/ttyUSB0
DXL_BAUDRATE=1000000
DXL_STARTID=1
DXL_NBID=1
DXL_PROTO=2
DXL_POS1=-511
DXL_POS2=511

## Dynamixel initialization sequence
# Operating mode 4: extende position control
./dxl write $DXL_PORTNAME $DXL_BAUDRATE $DXL_STARTID $DXL_NBID $DXL_PROTO 11 1 4
# Position D gain
./dxl write $DXL_PORTNAME $DXL_BAUDRATE $DXL_STARTID $DXL_NBID $DXL_PROTO 80 2 0
# Position I gain
./dxl write $DXL_PORTNAME $DXL_BAUDRATE $DXL_STARTID $DXL_NBID $DXL_PROTO 82 2 0
# Position P gain
./dxl write $DXL_PORTNAME $DXL_BAUDRATE $DXL_STARTID $DXL_NBID $DXL_PROTO 84 2 800
# Position FF2 gain
./dxl write $DXL_PORTNAME $DXL_BAUDRATE $DXL_STARTID $DXL_NBID $DXL_PROTO 88 2 0
# Position FF1 gain
./dxl write $DXL_PORTNAME $DXL_BAUDRATE $DXL_STARTID $DXL_NBID $DXL_PROTO 90 2 0
# Torque enable
./dxl write $DXL_PORTNAME $DXL_BAUDRATE $DXL_STARTID $DXL_NBID $DXL_PROTO 64 1 1

## Loop
echo "Press any key to exit..."
keypress=''
while [ "x$keypress" = "x" ];
do
	# Go to position 1
	./dxl write $DXL_PORTNAME $DXL_BAUDRATE $DXL_STARTID $DXL_NBID $DXL_PROTO 116 4 $DXL_POS1
	sleep 0.5
	# Read position
	./dxl read $DXL_PORTNAME $DXL_BAUDRATE $DXL_STARTID $DXL_NBID $DXL_PROTO 132 4 1
	# Go to position 2
	./dxl write $DXL_PORTNAME $DXL_BAUDRATE $DXL_STARTID $DXL_NBID $DXL_PROTO 116 4 $DXL_POS2
	sleep 0.5
	# Read position
	./dxl read $DXL_PORTNAME $DXL_BAUDRATE $DXL_STARTID $DXL_NBID $DXL_PROTO 132 4 1
	keypress="`cat -v`"
done

## Cleanup
# Torque disable
./dxl write $DXL_PORTNAME $DXL_BAUDRATE $DXL_STARTID $DXL_NBID $DXL_PROTO 64 1 0
# Restore tty behavior
if [ -t 0 ]; then stty sane; fi
