#!/usr/bin/bash

echo =================================================================
echo comma boot logo and bootanimation change by lepro3
echo =================================================================
dd if=./splash.img of=/dev/block/bootdevice/by-name/splash
mount -o rw,remount /system
cp ./bootanimation.zip /system/media/

echo =================================================================
echo logo and bootanimation change complete !
echo now reboot eon
reboot
