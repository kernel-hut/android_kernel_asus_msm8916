
#i2c test
echo "psensor i2c test"
psensor_i2c_result=$(adb shell "cat /data/data/proximity_status"|tr -d '\r')
if [ $psensor_i2c_result = "1" ];
then
	echo "pass"
else
	echo "fail"
	exit 0
fi
sleep 1

echo "lsensor i2c test"
lsensor_i2c_result=$(adb shell "cat /data/data/lightsensor_status"|tr -d '\r')
if [ $lsensor_i2c_result = "1" ];
then
	echo "pass"
else
	echo "fail"
	exit 0
fi
sleep 1
