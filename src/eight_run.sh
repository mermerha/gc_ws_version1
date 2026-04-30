roslaunch fast_lio mapping_with_driver.launch & sleep 6;
roslaunch px4ctrl run_ctrl.launch & sleep 2;
roslaunch poly_planner poly_planner.launch & sleep 2;
wait;
