#!/usr/bin/env python

import rospy
import json
import yaml
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Pose2D
from nav_msgs.msg import Odometry

class MulticastDataParser:
    def __init__(self, config_path):
        rospy.init_node('multicast_data_parser', anonymous=True)

        # Load configuration
        self.config = self.load_config(config_path)
        if not self.config:
            rospy.signal_shutdown("Failed to load configuration.")

        # Subscribers
        rospy.Subscriber('multicast_data', String, self.callback)

        # Publishers dictionary
        self.publishers = {}
        rigid_bodies = self.config.get('rigid_body_names', [])
        for rb in rigid_bodies:
            self.publishers[rb] = {
                'pose': rospy.Publisher(f'/multicast_parser/{rb}/pose', Pose, queue_size=10),
                'ground_pose': rospy.Publisher(f'/multicast_parser/{rb}/ground_pose', Pose2D, queue_size=10),
                'odom': rospy.Publisher(f'/multicast_parser/{rb}/Odom', Odometry, queue_size=10)
            }

        rospy.loginfo("Multicast Data Parser Node Initialized.")

    def load_config(self, config_path):
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
                rospy.loginfo(f"Configuration loaded from {config_path}")
                return config
        except Exception as e:
            rospy.logerr(f"Failed to load configuration file: {e}")
            return None

    def callback(self, msg):
        try:
            data = json.loads(msg.data)
            rospy.loginfo("Received multicast data.")

            # Iterate through each key-value pair in the received data
            for path, payload in data.items():
                # Extract rigid body name from the path
                parts = path.split('/')
                if len(parts) < 4:
                    rospy.logwarn(f"Invalid path format: {path}")
                    continue
                rb_name = parts[3]

                if rb_name not in self.publishers:
                    rospy.logwarn(f"Unknown rigid body: {rb_name}")
                    continue

                if path == f'/mocap_node/{rb_name}/pose':
                    pose_msg = self.parse_pose(payload)
                    if pose_msg:
                        self.publishers[rb_name]['pose'].publish(pose_msg)
                        rospy.loginfo(f"Published Pose data for {rb_name}.")
                
                elif path == f'/mocap_node/{rb_name}/ground_pose':
                    pose2d_msg = self.parse_pose2d(payload)
                    if pose2d_msg:
                        self.publishers[rb_name]['ground_pose'].publish(pose2d_msg)
                        rospy.loginfo(f"Published Pose2D data for {rb_name}.")

                elif path == f'/mocap_node/{rb_name}/Odom':
                    odom_msg = self.parse_odometry(payload)
                    if odom_msg:
                        self.publishers[rb_name]['odom'].publish(odom_msg)
                        rospy.loginfo(f"Published Odometry data for {rb_name}.")
                else:
                    rospy.logwarn(f"Received unknown path: {path}")

        except json.JSONDecodeError as e:
            rospy.logerr(f"JSON Decode Error: {e}")
        except Exception as e:
            rospy.logerr(f"Error parsing multicast data: {e}")

    def parse_pose(self, payload):
        try:
           #TODO
           pass
        except KeyError as e:
            rospy.logerr(f"Missing key in Pose data: {e}")
            return None

    def parse_pose2d(self, payload):
        try:
           #TODO
           pass
        except KeyError as e:
            rospy.logerr(f"Missing key in Pose2D data: {e}")
            return None

    def parse_odometry(self, payload):
        try:
           #TODO
           pass
        except KeyError as e:
            rospy.logerr(f"Missing key in Odometry data: {e}")
            return None

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    config_path = rospy.get_param('~config', 'config/config.yaml')
    parser = MulticastDataParser(config_path)
    parser.run()
