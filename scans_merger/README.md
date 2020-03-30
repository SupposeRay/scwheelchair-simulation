# scans_merger

## Brief description

This package is used to merge two individual LaserScans (e.g. front and rear) to a single LaserScan message. 
It uses the tf-package to transform the individual sensor data into a single reference frame (base_link). 

## Node description

### Subscribers
/front_scan, /rear_scan [sensor_msgs/LaserScan]

### Publishers

/scan [sensor_msgs/LaserScan]
/pcl [sensor_msgs/PointCloud]
