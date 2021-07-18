sudo cp 5/5.prototxt /home/ucar/522ucar/src/roborts_planning/local_planner/timed_elastic_band/config/timed_elastic_band.prototxt
sudo cp 5/amcl.launch /home/ucar/522ucar/src/ucar_nav/launch/amcl.launch
sudo cp 5/inflation_layer_config.prototxt /home/ucar/522ucar/src/roborts_costmap/config/inflation_layer_config.prototxt
sudo cp 5/global_planner_config.prototxt /home/ucar/522ucar/src/roborts_planning/global_planner/config/global_planner_config.prototxt
sudo cp 5/local_planner.prototxt /home/ucar/522ucar/src/roborts_planning/local_planner/config/local_planner.prototxt

sleep 1
roslaunch ucar_nav Obviously.launch 

