## Multicast Parser

WORK IN PROGRESS - DO NOT USE YET

This is a ROS Noetic node that listens to a multicast server from Motive Tracker 2.0.2 and parses the data into a ROS topic.

## Usage

Clone the repository in your ROS Noetic workspace and build the package.

```
cd ~/catkin_ws/src
git clone https://github.com/kyavuzkurt/multicast_parser.git
cd ~/catkin_ws
catkin_make
```

Configure the config.yaml file with the parameters of your system.
Parameters:
- multicast_group: The multicast group to listen to.
- multicast_port: The multicast port to listen to.
- server_address: The server address to listen to. (In my case it is the IP address of the machine running the Motive Tracker 2.0.2)
- buffer_size: The buffer size for the multicast listener.
- rigid_body_names: The names of the rigid bodies to parse the data for.

You can run the node with the following command:

```
roslaunch multicast_parser multicast_parser.launch
```

The node will publish the data to the following topics:

- `/multicast_parser/Robot_1/pose`
- `/multicast_parser/Robot_1/ground_pose`
- `/multicast_parser/Robot_1/odom`
