sudo cp C6H5C3H7/C6H5C3H7.prototxt /home/ucar/522ucar/src/roborts_planning/local_planner/timed_elastic_band/config/timed_elastic_band.prototxt
sudo cp C6H5C3H7/amcl.launch /home/ucar/522ucar/src/ucar_nav/launch/amcl.launch
sudo cp C6H5C3H7/inflation_layer_config.prototxt /home/ucar/522ucar/src/roborts_costmap/config

sleep 1
roslaunch ucar_nav Obviously.launch 

