#!/bin/bash
# Set GPU frequency to maximum
sudo echo "***** Set GPU frequency to maximum *****"
sudo echo "- Previous frequency of GPU"
sudo cat /sys/devices/17000000.ga10b/devfreq/17000000.ga10b/cur_freq
sudo echo "- The list of available frequencies"
sudo cat /sys/devices/17000000.ga10b/devfreq/17000000.ga10b/available_frequencies
sudo echo 1109250000 > /sys/devices/17000000.ga10b/devfreq/17000000.ga10b/max_freq
sudo echo 1109250000 > /sys/devices/17000000.ga10b/devfreq/17000000.ga10b/min_freq
sudo echo "- Changed frequency of GPU"
sudo cat /sys/devices/17000000.ga10b/devfreq/17000000.ga10b/cur_freq

