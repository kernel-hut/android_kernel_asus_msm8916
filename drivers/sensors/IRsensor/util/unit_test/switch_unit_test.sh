
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

#Turn on/off Test
echo "psensor switch test"
framework_switch=$(adb shell "cat /sys/class/sensors/psensor/switch"|tr -d '\r' )
psensor_turn_on "on"
proxm=$(adb shell "cat /data/data/proximity_proxm")
if [ $proxm -gt 0 ];
then
	echo "Proxm : $proxm"
else
	echo "Proxm : ERROR"
	exit 0
fi
if [ $framework_switch = "on" ];
then
	psensor_turn_on "on"
elif [ $framework_switch = "off" ];
then
	psensor_turn_on "off"
else
	echo "can not recognize Framework Status : $framework_switch"
fi
sleep 1

echo "lsensor switch test"
framework_switch=$(adb shell "cat /sys/class/sensors/lsensor/switch"|tr -d '\r' )
psensor_turn_on "on"
adc=$(adb shell "cat /data/data/lightsensor_adc")
if [ $adc -gt 0 ];
then
	echo "adc : $adc"
else
	echo "adc : ERROR"
	exit 0
fi
if [ $framework_switch = "on" ];
then
	lsensor_turn_on "on"
elif [ $framework_switch = "off" ];
then
	lsensor_turn_on "off"
else
	echo "can not recognize Framework Status : $framework_switch"
fi
sleep 1
