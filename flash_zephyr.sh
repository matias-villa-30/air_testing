#!/bin/bash

set -e 

LABEL="XIAO-SENSE"
MOUNT_POINT="/mnt/xiao_ble"
UF2_FILE="build/zephyr/zephyr.uf2"

DEVICE=$(lsblk -f -o NAME,LABEL -n | awk -v label="$LABEL" '$2==label {print "/dev/"$1}')

if [ -z "$DEVICE" ]; then
	echo "Device with '$LABEL' not found."
	exit 1
fi

sudo mount "$DEVICE" "$MOUNT_POINT"
sudo cp "$UF2_FILE" "$MOUNT_POINT"
sync

echo "Flashed succesfully!"
