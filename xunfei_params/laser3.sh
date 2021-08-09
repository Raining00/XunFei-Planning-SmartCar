echo "continuous gear3"
sudo cp laser3/laser.prototxt /home/ucar/ucar_ws/src/roborts_planning/local_planner/timed_elastic_band/config/timed_elastic_band.prototxt
sudo cp laser3/amcl.launch /home/ucar/ucar_ws/src/ucar_nav/launch/amcl.launch
sudo cp laser3/inflation_layer_config.prototxt /home/ucar/ucar_ws/src/roborts_costmap/config
sudo cp laser3/global_planner_config.prototxt /home/ucar/ucar_ws/src/roborts_planning/global_planner/config/global_planner_config.prototxt
sudo cp laser3/local_planner.prototxt /home/ucar/ucar_ws/src/roborts_planning/local_planner/config/local_planner.prototxt


sleep 1
roslaunch ucar_nav Obviously_laser.launch

