sudo cp C6H5C4H9/C6H5C4H9.prototxt /home/ucar/ucar_ws/src/roborts_planning/local_planner/timed_elastic_band/config/timed_elastic_band.prototxt
sudo cp C6H5C4H9/amcl.launch /home/ucar/ucar_ws/src/ucar_nav/launch/amcl.launch
sudo cp C6H5C4H9/inflation_layer_config.prototxt /home/ucar/ucar_ws/src/roborts_costmap/config

sleep 1
roslaunch ucar_nav Obviously.launch 

