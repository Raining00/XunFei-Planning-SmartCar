sudo cp 2/2.prototxt /home/ucar/522ucar/src/roborts_planning/local_planner/timed_elastic_band/config/timed_elastic_band.prototxt
sudo cp 2/amcl.launch /home/ucar/522ucar/src/ucar_nav/launch/amcl.launch
sudo cp 2/inflation_layer_config.prototxt /home/ucar/522ucar/src/roborts_costmap/config
sudo cp 2/global_planner_config.prototxt /home/ucar/522ucar/src/roborts_planning/global_planner/config/global_planner_config.prototxt
sudo cp 2/local_planner.prototxt /home/ucar/522ucar/src/roborts_planning/local_planner/config/local_planner.prototxt


sleep 1
roslaunch ucar_nav Obviously.launch 

