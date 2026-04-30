sudo nvpmodel -m 8 & sleep 1;
sudo ./scrips/max_cpu_freq.sh;
sudo ./scrips/max_emc_freq.sh;
sudo ./scrips/max_gpu_freq.sh;

roslaunch faster_lio mapping_with_driver.launch & sleep 6;
roslaunch px4ctrl run_ctrl.launch & sleep 2;
roslaunch ego_planner run_in_exp.launch & sleep 2;
#roslaunch usb_cam usb_cam.launch & sleep 2;
wait;
