sudo cp photo/photo.prototxt /home/ucar/ucar_ws/src/roborts_planning/local_planner/timed_elastic_band/config/timed_elastic_band.prototxt
sudo cp photo/amcl.launch /home/ucar/ucar_ws/src/ucar_nav/launch/amcl.launch
sudo cp photo/inflation_layer_config.prototxt /home/ucar/ucar_ws/src/roborts_costmap/config/inflation_layer_config.prototxt
sudo cp photo/global_planner_config.prototxt /home/ucar/ucar_ws/src/roborts_planning/global_planner/config/global_planner_config.prototxt
sudo cp photo/local_planner.prototxt /home/ucar/ucar_ws/src/roborts_planning/local_planner/config/local_planner.prototxt

sleep 1
roslaunch ucar_nav Obviously_gear1.launch

