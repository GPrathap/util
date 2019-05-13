# geodetic_utils

## Description
Simple library for converting coordinates to/from several geodetic frames and managing incoming GPS information.



## Nodes
### geodetic_to_local_conversion_node
Publishes position information based on GPS measurements and initialised reference frame
#### Parameters
* `frame_id` - string in header field of output messages ("world" is the default one if it's not specified)

#### Subscribed Topics:
* `gps` ([sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html)) - GPS sensor information
* 'imu'
* 'external_altitude'

#### Published Topics:
* `gps_pose` ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)) - local ENU co-ordinates in initialised reference frame
* 'odometry'
* 'tf'

## Dependencies:
* ros
* geometry_msgs
* sensor_msgs

## Notes:
Based on much changed https://github.com/ethz-asl/geodetic_utils from ETHZ ASL

## Credits:
Amber Garage
Roman Fedorenko 
