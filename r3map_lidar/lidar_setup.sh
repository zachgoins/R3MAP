sudo ifconfig eth0 192.168.3.100
sudo route add 192.168.1.201 eth0
rosrun velodyne_pointcloud gen_calibration.py calibration.xml

