## Multicast Parser


This is a ROS Noetic node that listens to a multicast server from Motive Tracker 2.0.2 and parses the data into a ROS topic. In my tests, Motive Tracker 2.0.2 is running on a Windows machine and the ROS nodes are running on a Ubuntu 20.04 Docker container. 

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
- log_level: The log level for the node. (DEBUG, INFO)

You can run the node with the following command:

```
roslaunch multicast_parser multicast_parser.launch
```

The node will publish the data to the following topics:

- `/multicast_parser/<rigid_body_name>/pose` (geometry_msgs/PoseStamped)
- `/multicast_parser/<rigid_body_name>/ground_pose` (geometry_msgs/PoseStamped)
- `/multicast_parser/<rigid_body_name>/odom` (nav_msgs/Odometry)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
