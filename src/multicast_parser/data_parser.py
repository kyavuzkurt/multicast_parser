#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Pose2D
from nav_msgs.msg import Odometry

class MulticastDataParser:
    def __init__(self):
        rospy.init_node('multicast_data_parser', anonymous=True)

        # Subscribers
        rospy.Subscriber('multicast_data', String, self.callback)

        # Publishers
        self.pose_pub = rospy.Publisher('/multicast_parser/Robot_1/pose', Pose, queue_size=10)
        self.ground_pose_pub = rospy.Publisher('/multicast_parser/Robot_1/ground_pose', Pose2D, queue_size=10)
        self.odom_pub = rospy.Publisher('/multicast_parser/Robot_1/Odom', Odometry, queue_size=10)

        rospy.loginfo("Multicast Data Parser Node Initialized.")

    def callback(self, msg):
        try:
            data = json.loads(msg.data)
            rospy.loginfo("Received multicast data.")

            # Iterate through each key-value pair in the received data
            for path, payload in data.items():
                if path == '/mocap_node/Robot_1/pose':
                    pose_msg = self.parse_pose(payload)
                    if pose_msg:
                        self.pose_pub.publish(pose_msg)
                        rospy.loginfo("Published Pose data.")
                
                elif path == '/mocap_node/Robot_1/ground_pose':
                    pose2d_msg = self.parse_pose2d(payload)
                    if pose2d_msg:
                        self.ground_pose_pub.publish(pose2d_msg)
                        rospy.loginfo("Published Pose2D data.")

                elif path == '/mocap_node/Robot_1/Odom':
                    odom_msg = self.parse_odometry(payload)
                    if odom_msg:
                        self.odom_pub.publish(odom_msg)
                        rospy.loginfo("Published Odometry data.")
                else:
                    rospy.logwarn(f"Received unknown path: {path}")

        except json.JSONDecodeError as e:
            rospy.logerr(f"JSON Decode Error: {e}")
        except Exception as e:
            rospy.logerr(f"Error parsing multicast data: {e}")

    def parse_pose(self, payload):
        try:
            pose_msg = Pose()
            pose_msg.position.x = payload['position']['x']
            pose_msg.position.y = payload['position']['y']
            pose_msg.position.z = payload['position']['z']
            pose_msg.orientation.x = payload['orientation']['x']
            pose_msg.orientation.y = payload['orientation']['y']
            pose_msg.orientation.z = payload['orientation']['z']
            pose_msg.orientation.w = payload['orientation']['w']
            return pose_msg
        except KeyError as e:
            rospy.logerr(f"Missing key in Pose data: {e}")
            return None

    def parse_pose2d(self, payload):
        try:
            pose2d_msg = Pose2D()
            pose2d_msg.x = payload['x']
            pose2d_msg.y = payload['y']
            pose2d_msg.theta = payload['theta']
            return pose2d_msg
        except KeyError as e:
            rospy.logerr(f"Missing key in Pose2D data: {e}")
            return None

    def parse_odometry(self, payload):
        try:
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = payload.get('frame_id', 'odom')

            # Position
            odom_msg.pose.pose.position.x = payload['pose']['position']['x']
            odom_msg.pose.pose.position.y = payload['pose']['position']['y']
            odom_msg.pose.pose.position.z = payload['pose']['position']['z']

            # Orientation
            odom_msg.pose.pose.orientation.x = payload['pose']['orientation']['x']
            odom_msg.pose.pose.orientation.y = payload['pose']['orientation']['y']
            odom_msg.pose.pose.orientation.z = payload['pose']['orientation']['z']
            odom_msg.pose.pose.orientation.w = payload['pose']['orientation']['w']

            # Velocity
            odom_msg.twist.twist.linear.x = payload['twist']['linear']['x']
            odom_msg.twist.twist.linear.y = payload['twist']['linear']['y']
            odom_msg.twist.twist.linear.z = payload['twist']['linear']['z']
            odom_msg.twist.twist.angular.x = payload['twist']['angular']['x']
            odom_msg.twist.twist.angular.y = payload['twist']['angular']['y']
            odom_msg.twist.twist.angular.z = payload['twist']['angular']['z']

            return odom_msg
        except KeyError as e:
            rospy.logerr(f"Missing key in Odometry data: {e}")
            return None

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    parser = MulticastDataParser()
    parser.run()
