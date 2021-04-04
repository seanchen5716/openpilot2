#!/usr/bin/bash

echo =================================================================
echo comma boot logo and bootanimation change by op3t
echo =================================================================

dd if=./logo.bin of=/dev/block/bootdevice/by-name/LOGO
mount -o rw,remount /system
cp ./bootanimation.zip /system/media/

echo =================================================================
echo logo and bootanimation change complete !
echo now reboot eon
reboot
