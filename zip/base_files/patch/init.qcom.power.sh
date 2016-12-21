#!/system/bin/sh
#
# Copyright (C) 2016 The CyanogenMod Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

export PATH=/system/bin

if [ -f /sys/devices/soc0/soc_id ]; then
    soc_id=`cat /sys/devices/soc0/soc_id`
else
    soc_id=`cat /sys/devices/system/soc/soc0/id`
fi

# Enable adaptive LMK and set vmpressure_file_min
echo 1 > /sys/module/lowmemorykiller/parameters/enable_adaptive_lmk
echo 81250 > /sys/module/lowmemorykiller/parameters/vmpressure_file_min

# HMP scheduler settings for 8916, 8939
echo 3 > /proc/sys/kernel/sched_window_stats_policy
echo 3 > /proc/sys/kernel/sched_ravg_hist_size

case "$soc_id" in
    "206" | "247" | "248" | "249" | "250")
        # Apply MSM8916 specific Sched & Governor settings

        # HMP scheduler load tracking settings
        echo 3 > /proc/sys/kernel/sched_ravg_hist_size

        # HMP Task packing settings for 8916
        echo 20 > /proc/sys/kernel/sched_small_task
        echo 30 > /proc/sys/kernel/sched_mostly_idle_load
        echo 3 > /proc/sys/kernel/sched_mostly_idle_nr_run

        # Disable thermal core_control to update scaling_min_freq
        echo 0 > /sys/module/msm_thermal/core_control/enabled
	echo Y > /sys/module/msm_thermal/parameters/enabled
	echo Y > /sys/module/workqueue/parameters/power_efficient

	# Enable Reaper Darkness Features	
	echo performance > /sys/devices/soc.0/1c00000.qcom,kgsl-3d0/devfreq/1c00000.qcom,kgsl-3d0/governor
	echo 258 > /sys/devices/platform/kcal_ctrl.0/kcal_val
	echo 265 > /sys/devices/platform/kcal_ctrl.0/kcal_cont
	echo 280 > /sys/devices/platform/kcal_ctrl.0/kcal_sat
	echo 1 > /proc/sys/vm/laptop_mod
	echo 5 5 > /sys/kernel/sound_control_3/gpl_speaker_gain
	echo 5 5 245 > /sys/kernel/sound_control_3/gpl_speaker_gain
	swapoff /dev/block/zram0 > /dev/null 2>&1
	echo 1 > /dev/block/zram0/reset
	echo 534773760 > /sys/block/zram0/disksize
	mkswap /dev/block/zram0 > /dev/null 2>&1
	swapon /dev/block/zram0 > /dev/null 2>&1

        # Enable governor
        echo 1 > /sys/devices/system/cpu/cpu0/online
        echo "interactive" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

        echo "25000 1209600:50000" > /sys/devices/system/cpu/cpufreq/interactive/above_hispeed_delay
        echo 90 > /sys/devices/system/cpu/cpufreq/interactive/go_hispeed_load
        echo 30000 > /sys/devices/system/cpu/cpufreq/interactive/timer_rate
        echo 1152000 > /sys/devices/system/cpu/cpufreq/interactive/hispeed_freq
        echo 0 > /sys/devices/system/cpu/cpufreq/interactive/io_is_busy
        echo "1 200000:85 1094400:90 800000:80" > /sys/devices/system/cpu/cpufreq/interactive/target_loads
        echo 50000 > /sys/devices/system/cpu/cpufreq/interactive/min_sample_time
    ;;
    "239" | "241" | "263" | "268" | "269" | "270" | "271")
        # Apply MSM8939 specific Sched & Governor settings

        # Sched Boost
        echo 1 > /proc/sys/kernel/sched_boost

        # HMP scheduler load tracking settings
        echo 5 > /proc/sys/kernel/sched_ravg_hist_size

        # HMP Task packing settings for 8939, 8929
        echo 20 > /proc/sys/kernel/sched_small_task
        echo 30 > /proc/sys/kernel/sched_mostly_idle_load
        echo 3 > /proc/sys/kernel/sched_mostly_idle_nr_run

        echo bw_hwmon > /sys/class/devfreq/cpubw/governor
        echo 20 > /sys/class/devfreq/cpubw/bw_hwmon/io_percent
        echo 40 >/sys/class/devfreq/gpubw/bw_hwmon/io_percent
        echo cpufreq > /sys/class/devfreq/mincpubw/governor

        # Disable thermal core_control to update interactive gov settings
        echo 0 > /sys/module/msm_thermal/core_control/enabled

        # Enable governor for perf cluster
        echo 1 > /sys/devices/system/cpu/cpu0/online
        echo "interactive" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
        echo "20000 1113600:50000" > /sys/devices/system/cpu/cpu0/cpufreq/interactive/above_hispeed_delay
        echo 85 > /sys/devices/system/cpu/cpu0/cpufreq/interactive/go_hispeed_load
        echo 20000 > /sys/devices/system/cpu/cpu0/cpufreq/interactive/timer_rate
        echo 1113600 > /sys/devices/system/cpu/cpu0/cpufreq/interactive/hispeed_freq
        echo 0 > /sys/devices/system/cpu/cpu0/cpufreq/interactive/io_is_busy
        echo "1 960000:85 1113600:90 1344000:80" > /sys/devices/system/cpu/cpu0/cpufreq/interactive/target_loads
        echo 50000 > /sys/devices/system/cpu/cpu0/cpufreq/interactive/min_sample_time
        echo 960000 > /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq

        # Enable governor for power cluster
        echo 1 > /sys/devices/system/cpu/cpu4/online
        echo "interactive" > /sys/devices/system/cpu/cpu4/cpufreq/scaling_governor
        echo "25000 800000:50000" > /sys/devices/system/cpu/cpu4/cpufreq/interactive/above_hispeed_delay
        echo 90 > /sys/devices/system/cpu/cpu4/cpufreq/interactive/go_hispeed_load
        echo 40000 > /sys/devices/system/cpu/cpu4/cpufreq/interactive/timer_rate
        echo 998400 > /sys/devices/system/cpu/cpu4/cpufreq/interactive/hispeed_freq
        echo 0 > /sys/devices/system/cpu/cpu4/cpufreq/interactive/io_is_busy
        echo "1 800000:90" > /sys/devices/system/cpu/cpu4/cpufreq/interactive/target_loads
        echo 40000 > /sys/devices/system/cpu/cpu4/cpufreq/interactive/min_sample_time
        echo 800000 > /sys/devices/system/cpu/cpu4/cpufreq/scaling_min_freq

        # Enable thermal core_control now
        echo 1 > /sys/module/msm_thermal/core_control/enabled

        # Bring up all cores online
        echo 1 > /sys/devices/system/cpu/cpu1/online
        echo 1 > /sys/devices/system/cpu/cpu2/online
        echo 1 > /sys/devices/system/cpu/cpu3/online
        echo 1 > /sys/devices/system/cpu/cpu4/online
        echo 1 > /sys/devices/system/cpu/cpu5/online
        echo 1 > /sys/devices/system/cpu/cpu6/online
        echo 1 > /sys/devices/system/cpu/cpu7/online

        # Enable low power modes
        echo 0 > /sys/module/lpm_levels/parameters/sleep_disabled

        # Enable core control and set userspace permission
        echo 4 > /sys/devices/system/cpu/cpu4/core_ctl/max_cpus
        echo 0 > /sys/devices/system/cpu/cpu4/core_ctl/min_cpus
        echo 60 > /sys/devices/system/cpu/cpu4/core_ctl/busy_up_thres
        echo 50 > /sys/devices/system/cpu/cpu4/core_ctl/busy_down_thres
        echo 100 > /sys/devices/system/cpu/cpu4/core_ctl/offline_delay_ms
        echo 4 > /sys/devices/system/cpu/cpu4/core_ctl/task_thres
        chown system:system /sys/devices/system/cpu/cpu0/core_ctl/max_cpus
        chown system:system /sys/devices/system/cpu/cpu0/core_ctl/min_cpus
        chown system:system /sys/devices/system/cpu/cpu4/core_ctl/max_cpus
        chown system:system /sys/devices/system/cpu/cpu4/core_ctl/min_cpus

        # HMP scheduler (big.Little cluster related) settings
        echo 75 > /proc/sys/kernel/sched_upmigrate
        echo 60 > /proc/sys/kernel/sched_downmigrate
    ;;
esac

case "$soc_id" in
    "206" | "247" | "248" | "249" | "250")
        echo 0 > /sys/module/lpm_levels/parameters/sleep_disabled
        echo 1 > /sys/devices/system/cpu/cpu1/online
        echo 1 > /sys/devices/system/cpu/cpu2/online
        echo 1 > /sys/devices/system/cpu/cpu3/online
    ;;
    "239" | "241" | "263"| "268" | "269" | "270" | "271")
        echo 10 > /sys/class/net/rmnet0/queues/rx-0/rps_cpus
    ;;
esac

case $soc_id in
    "206" | "247" | "248" | "249" | "250" | "233" | "240" | "242")
        setprop ro.min_freq_0 800000
    ;;
    "239" | "241" | "263" | "268" | "269" | "270" | "271")
        setprop ro.min_freq_0 960000
        setprop ro.min_freq_4 800000
    ;;
esac
