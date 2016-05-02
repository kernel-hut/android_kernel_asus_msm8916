#!/system/bin/sh
MAX_RUN=$1

lsensor_turn_on()
{
	if [ $1 = "on" ];
	then
		#echo "psensor switch on"
		adb shell "echo on > /sys/class/sensors/lsensor/switch"
	elif  [ $1 = "off" ];
	then
		#echo "psensor switch off"
		adb shell "echo off > /sys/class/sensors/lsensor/switch"
	else
		echo "lsensor_turn_on() invalid argument."
	fi
}

#check the framework status
framework_switch=$(adb shell "cat /sys/class/sensors/lsensor/switch"|tr -d '\r' )
echo "The framework status : $framework_switch"

#switch on/off stress test
for run in $(seq 1 $MAX_RUN)
do	
	echo "lsensor Switch on/off Stress Test : $run"
	lsensor_turn_on "on"
	sleep 1
	adc=$(adb shell "cat /data/data/lightsensor_adc")
	if [ $adc -gt 0 ];
	then
		echo "adc : $adc"
	else
		echo "adc : ERROR"
		exit 0
	fi
	lsensor_turn_on "off"
	sleep 1
done

#restore framework status
if [ $framework_switch = "on" ];
then
	echo "Restore Framework Status : Turning on"
	lsensor_turn_on "on"
elif [ $framework_switch = "off" ];
then
	echo "Restore Framework Status : Turning off"
	lsensor_turn_on "off"
else
	echo "can not recognize Framework Status : $framework_switch"
fi

exit 0
