sudo chmod 777 /dev/tty* & sleep 1;
roslaunch mavros px4.launch & sleep 6;
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 2;
roslaunch faster_lio mapping_with_driver.launch & sleep 6;
roslaunch px4ctrl run_ctrl.launch & sleep 2;
roslaunch ego_planner run_in_exp_interactive.launch & sleep 2;
wait;