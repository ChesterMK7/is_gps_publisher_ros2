# is_gps_publisher_ros2

A GPS and IMU data parser and publisher ROS2 package. Takes in [NMEA](https://github.com/inertialsense/docs.inertialsense.com/blob/1.11.0/docs/user-manual/com-protocol/nmea.md) messages from a specified network port using UDP packets, then parses them and outputs the parsed GPS and IMU data in the sensor_messages [Imu](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Imu.msg) and [NavSatFix](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/NavSatFix.msg) formats as ROS2 topics.

## Requirements

Ubuntu & ROS2    
GPS that outputs NMEA messages in the PIMU and GGA formats    
Above messages sent as UDP packets to known network port    

### Tested Using

Ubuntu 22.04 ARM64    
ROS2 Humble    
Nvidia Jetson AGX Orin    
Inertial Sense uINS    
[ISRoverNetworkNMEA](https://github.com/arcater/ISRoverNetworkNMEA)    

## Setup

Create a ROS2 Workspace

``` bash
mkdir -p gpsWS/src
cd gpsWS/src
```

Clone the respository in the src folder

``` bash
git clone https://github.com/ChesterMK7/is_gps_publisher_ros2
```

Make sure to configure the network port and frame id you are using

``` bash
nano is_gps_publisher_ros2/src/gps_parser.hpp
```

Specifically modify these lines in gps_parser.hpp

``` cpp
#define GPS_FRAME "gps_link" // GPS frame name
#define IMU_FRAME "imu_link" // IMU frame name
#define PORT 25565 // Port used
#define MAXLINE 256 // Maximum line length
```

Return to the workspace directory and build using colcon

``` bash
cd ..
source /opt/ros/<ros2-version>/setup.bash
colcon build
```

Source the workspace

``` bash
source install/setup.bash
```

## Usage

Run the parser/publisher node

``` bash
ros2 run is_gps_publisher_ros2 gps_output_publisher
```

### Viewing output data

Open a new terminal tab/window

IMU

``` bash
source /opt/ros/<ros2-version>/setup.bash
ros2 topic echo /imu
```

GPS

``` bash
source /opt/ros/<ros2-version>/setup.bash
ros2 topic echo /nav_sat_fix
```
