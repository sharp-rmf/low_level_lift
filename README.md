# Lift2
Dry Contact Lift Sensor

# Dry Contact Lift Sensor-ROS2 wrapper (Meyer Lift Sensor)

The AGV calls up the lift to the current floor using this ROS2 wrapper and sends the destination floor to lift controller via the ROS2 wrapper. The wrapper reads the command from the AGV and controls the signals of the GPIO pins on the Raspberry Pi accordingly.

### Branches

`bcm2835_lift_controller` branch runs on ROS crystal
`master` branch runs on ROS Dashing

### Getting Started

The following is the activity diagram for the dry contact lift sensor

![alt text](https://github.com/RMFHOPE/Lift2/blob/master/documentation/Meyer%20Lift%20Sensor_updated.png)

### Prerequisites

This package is a ROS2 bouncy package running on Ubuntu. To run this package, you will need the following:
1. Raspberry Pi 3B+
2. Ubuntu 18.04/Raspbian installed on Raspberry Pi
3. ROS2 Dashing Version
4. BCM2835 library for Raspberry Pi 3B+
5. Electrical components for testing

### Installing

BCM2835 library need to be installed to manipulate GPIO pins on the Raspberry Pi.
To install BCM2835 do the following https://www.airspayce.com/mikem/bcm2835/ :
```
tar zxvf bcm2835-1.xx.tar.gz
cd bcm2835-1.xx
./configure
make
sudo make check
sudo make install
```

Download rmf_lift_msgs [https://github.com/osrf/rmf_core/tree/master/rmf_lift_msgs] from rmf_core repo [https://github.com/osrf/rmf_core] into the same workspace

After this, clone `dry_contact_lift_sensor` repository to your ROS2 workspace
```
cd my_ros2_ws/src
git clone https://github.com/RMFHOPE/Lift2.git
cp -r my_ros2_ws/src/Lift2/dry_contact_lift_sensor .
```

## Running the code

There is a startup script running at reboot. 
In case, the script needs to be modified, compile and run using the following code: 

Compile the `rmf_lift_msgs` using the command:

```
colcon build --symlink-install --packages-select rmf_lift_msgs
```


For compiling the lift controller code, use the following command:

```
colcon build --cmake-clean-first --symlink-install --packages-select dry_contact_lift_sensor --cmake-clean-cache
```


Source the setup files using the command,
```

source install/setup.bash

```


To run the sensor package node (dry_contact_node):

```
ros2 run dry_contact_lift_sensor DryContactLiftWrap __params:=/home/.../rmf/build/ros2/src/dry_contact_lift_sensor/params/parameters.yaml
```

Command to run on the Pi

```
ros2 run dry_contact_lift_sensor DryContactLiftWrap __params:=<path_to>/ros2_ws/src/dry_contact_lift_sensor/params/parameters.yaml
```

To send commands to control the lift:


```
ros2 topic pub rmf_lift_msgs/LiftRequest rmf_lift_msgs/LiftRequest "{lift_name: "LF001", session_id: "ABCD", request_type: 1, destination_floor: "L1", door_state: 2}" --once
```



To check on the messages being published, echo the following topics:


```
ros2 topic echo rmf_lift_msgs/LiftRequest
```

```
ros2 topic echo rmf_lift_msgs/LiftState
```
