sudo cp 0.1/1.prototxt /home/ucar/ucar_ws/src/roborts_planning/local_planner/timed_elastic_band/config/timed_elastic_band.prototxt
sudo cp 0.1/amcl.launch /home/ucar/ucar_ws/src/ucar_nav/launch/amcl.launch
sudo cp 0.1/inflation_layer_config.prototxt /home/ucar/ucar_ws/src/roborts_costmap/config
sudo cp 0.1/global_planner_config.prototxt /home/ucar/ucar_ws/src/roborts_planning/global_planner/config/global_planner_config.prototxt
sudo cp 0.1/local_planner.prototxt /home/ucar/ucar_ws/src/roborts_planning/local_planner/config/local_planner.prototxt


sleep 1
roslaunch ucar_nav Obviously_gear1.launch

