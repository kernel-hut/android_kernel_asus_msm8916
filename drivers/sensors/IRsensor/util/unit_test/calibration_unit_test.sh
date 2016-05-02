
#Calibration Test
echo "psensor low calibration test"
original_low_cal=$(adb shell "cat /factory/psensor_low.nv"|tr -d '\r' )
low_cal=23
adb shell "echo $low_cal > /sys/class/sensors/psensor/low_cal"
low_cal_ret=$(adb shell "cat /factory/psensor_low.nv"|tr -d '\r' ) 
if [ $low_cal_ret = $low_cal ];
then
	echo "pass"
else
	echo "fail"
	exit 0
fi
adb shell "echo $original_low_cal > /sys/class/sensors/psensor/low_cal"
sleep 1

echo "psensor hi calibration test"
original_hi_cal=$(adb shell "cat /factory/psensor_hi.nv"|tr -d '\r' )
hi_cal=53
adb shell "echo $hi_cal > /sys/class/sensors/psensor/hi_cal"
hi_cal_ret=$(adb shell "cat /factory/psensor_hi.nv"|tr -d '\r' ) 
if [ $hi_cal_ret = $hi_cal ];
then
	echo "pass"
else
	echo "fail"
	exit 0
fi
adb shell "echo $original_hi_cal > /sys/class/sensors/psensor/hi_cal"
sleep 1

echo "lsensor 200 lux calibration test"
original_200_cal=$(adb shell "cat /factory/lsensor_200lux.nv"|tr -d '\r' )
lux_200_cal=203
adb shell "echo $lux_200_cal > /sys/class/sensors/lsensor/200lux_cal"
lux_200_cal_ret=$(adb shell "cat /factory/lsensor_200lux.nv"|tr -d '\r' ) 
if [ $lux_200_cal_ret = $lux_200_cal ];
then
	echo "pass"
else
	echo "fail"
	exit 0
fi
adb shell "echo $original_200_cal > /sys/class/sensors/lsensor/200lux_cal"
sleep 1

echo "lsensor 1000 lux calibration test"
original_1000_cal=$(adb shell "cat /factory/lsensor_1000lux.nv"|tr -d '\r' )
lux_1000_cal=1003
adb shell "echo $lux_1000_cal > /sys/class/sensors/lsensor/1000lux_cal"
lux_1000_cal_ret=$(adb shell "cat /factory/lsensor_1000lux.nv"|tr -d '\r' ) 
if [ $lux_1000_cal_ret = $lux_1000_cal ];
then
	echo "pass"
else
	echo "fail"
	exit 0
fi
adb shell "echo $original_1000_cal > /sys/class/sensors/lsensor/1000lux_cal"
sleep 1
