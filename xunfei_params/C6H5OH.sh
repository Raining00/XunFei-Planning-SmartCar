sudo cp C6H5OH/C6H5OH.prototxt /home/ucar/522ucar/src/roborts_planning/local_planner/timed_elastic_band/config/timed_elastic_band.prototxt
sudo cp C6H5OH/amcl.launch /home/ucar/522ucar/src/ucar_nav/launch/amcl.launch
sudo cp C6H5OH/inflation_layer_config.prototxt /home/ucar/522ucar/src/roborts_costmap/config/inflation_layer_config.prototxt

sleep 1
roslaunch ucar_nav Obviously.launch 

