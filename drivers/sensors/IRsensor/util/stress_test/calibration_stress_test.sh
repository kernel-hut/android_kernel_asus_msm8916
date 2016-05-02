#!/system/bin/sh
MAX_RUN=$1

for run in $(seq 1 $MAX_RUN)
do
	echo "IR sensor Calibration Stress Test : $run"
	sh ../unit_test/calibration_unit_test.sh
done
