sudo nvpmodel -m 8 & sleep 1;
sudo ./src/scrips/max_cpu_freq.sh;
sudo ./src/scrips/max_emc_freq.sh;
sudo ./src/scrips/max_gpu_freq.sh;

source ../devel/setup.bash
sudo chmod 777 /dev/ttyTHS0

roslaunch realsense2_camera rs_camera.launch & sleep 1;
sh ./src/px4.sh & sleep 10;
#roslaunch mavros px4.launch & sleep 10;
roslaunch vins vins.launch & sleep 10;

# roslaunch px4ctrl run_ctrl_vins.launch & sleep 1;

#roslaunch ego_planner run_in_exp_vins.launch & sleep 1;

wait;
