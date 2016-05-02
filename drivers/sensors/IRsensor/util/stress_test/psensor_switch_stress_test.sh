#!/system/bin/sh
MAX_RUN=$1

psensor_turn_on()
{
	if [ $1 = "on" ];
	then
		#echo "psensor switch on"
		adb shell "echo on > /sys/class/sensors/psensor/switch"
	elif  [ $1 = "off" ];
	then
		#echo "psensor switch off"
		adb shell "echo off > /sys/class/sensors/psensor/switch"
	else
		echo "psensor_turn_on() invalid argument."
	fi
}

#check the framework status
framework_switch=$(adb shell "cat /sys/class/sensors/psensor/switch"|tr -d '\r' )
echo "The framework status : $framework_switch"

#switch on/off stress test
for run in $(seq 1 $MAX_RUN)
do	
	echo "psensor Switch on/off Stress Test : $run"
	psensor_turn_on "on"
	sleep 1
	proxm=$(adb shell "cat /data/data/proximity_proxm")
	if [ $proxm -gt 0 ];
	then
		echo "Proxm : $proxm"
	else
		echo "Proxm : ERROR"
		exit 0
	fi
	psensor_turn_on "off"
	sleep 1
done

#restore framework status
if [ $framework_switch = "on" ];
then
	echo "Restore Framework Status : Turning on"
	psensor_turn_on "on"
elif [ $framework_switch = "off" ];
then
	echo "Restore Framework Status : Turning off"
	psensor_turn_on "off"
else
	echo "can not recognize Framework Status : $framework_switch"
fi

exit 0
