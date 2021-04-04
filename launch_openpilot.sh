#!/usr/bin/bash

if [ ! -f "/system/fonts/NanumGothic.ttf" ]; then
    sleep 3
    mount -o remount,rw /system

    cp -rf /data/openpilot/installer/fonts/NanumGothic* /system/fonts/
    cp -rf /data/openpilot/installer/fonts/fonts.xml /system/etc/fonts.xml
    chmod 644 /system/etc/fonts.xml
    chmod 644 /system/fonts/NanumGothic*

    cp /data/openpilot/installer/fonts/bootanimation.zip /system/media/

    mount -o remount,r /system

    setprop persist.sys.locale ko-KR
    setprop persist.sys.local ko-KR
    setprop persist.sys.timezone Asia/Seoul

        if [ ! -f "/data/BOOTLOGO" ]; then

        dd if=/data/openpilot/installer/fonts/splash.img of=/dev/block/bootdevice/by-name/splash
        dd if=/data/openpilot/installer/fonts/logo.bin of=/dev/block/bootdevice/by-name/LOGO

        /usr/bin/touch /data/BOOTLOGO

        cp /data/openpilot/installer/fonts/LateralControlLqr /data/params/d
        cp /data/openpilot/installer/fonts/MadModeEnabled /data/params/d

        fi

    echo =================================================================
    echo Ko-KR NanumGothic font install complete
    echo Bootanimation change complete
    echo Ko-KR locale change complete
    echo Comma boot logo change complete
    echo Reboot Now..!!
    echo =================================================================

    reboot
fi

export PASSIVE="0"
exec ./launch_chffrplus.sh
