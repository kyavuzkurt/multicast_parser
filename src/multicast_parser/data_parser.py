#!/usr/bin/env python

import rospy
import yaml
import struct
from std_msgs.msg import UInt8MultiArray
from geometry_msgs.msg import Pose, Pose2D
from nav_msgs.msg import Odometry
import os
import rospkg
import math

VECTOR3_FORMAT = '<fff'        # x, y, z as floats
QUATERNION_FORMAT = '<ffff'    # x, y, z, w as floats

class MulticastDataParser:
    NAT_FRAMEOFDATA = 7  # Corrected Message ID for Frame Data
    NAT_MODELDEF = 5     


    def __init__(self, config_path):
        rospy.init_node('multicast_data_parser', anonymous=True)

        # Load configuration
        self.config = self.load_config(config_path)
        if not self.config:
            rospy.signal_shutdown("Failed to load configuration.")

        # Subscribers
        rospy.Subscriber('multicast_data', UInt8MultiArray, self.callback)

        # Publishers dictionary
        self.publishers = {}
        rigid_bodies = self.config.get('rigid_body_names', [])
        for rb in rigid_bodies:
            self.publishers[rb] = {
                'pose': rospy.Publisher(f'/multicast_parser/{rb}/pose', Pose, queue_size=10),
                'ground_pose': rospy.Publisher(f'/multicast_parser/{rb}/ground_pose', Pose2D, queue_size=10),
                'odom': rospy.Publisher(f'/multicast_parser/{rb}/odom', Odometry, queue_size=10)
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
            # Convert the list of integers back to bytes
            data = bytes(msg.data)
            rospy.logdebug(f"Received multicast data: {data}")

            # Process the binary data
            self.process_binary_data(data)

        except Exception as e:
            rospy.logerr(f"Error processing multicast data: {e}")

    def process_binary_data(self, data: bytes):
        """
        Parses the binary data and publishes the corresponding ROS messages.
        """
        try:
            if len(data) < 4:
                rospy.logwarn("Received data is too short to contain message ID and packet size.")
                return

            # Extract Message ID and Packet Size
            message_id = int.from_bytes(data[0:2], byteorder='little', signed=True)
            packet_size = int.from_bytes(data[2:4], byteorder='little', signed=True)
            rospy.logdebug(f"Message ID: {message_id}, Packet Size: {packet_size} bytes")

            # Check if the received data matches the expected packet size
            actual_payload_size = len(data[4:])
            if actual_payload_size < packet_size:
                rospy.logwarn(f"Insufficient data: expected {packet_size} bytes, received {actual_payload_size} bytes. Skipping this packet.")
                return
            elif actual_payload_size > packet_size:
                rospy.logwarn(f"Received more data than expected: expected {packet_size} bytes, received {actual_payload_size} bytes. Truncating extra bytes.")

            # Slice the data to the expected packet size
            packet_data = data[4:4+packet_size]

            if message_id == self.NAT_FRAMEOFDATA:
                frame_data, _ = self.unpack_frame_data(packet_data)
                self.publish_frame_data(frame_data)
            else:
                rospy.logwarn(f"Unrecognized or unhandled Message ID: {message_id}")

        except Exception as e:
            rospy.logerr(f"Failed to process binary data: {e}")

    def unpack_frame_data(self, data: bytes):
        """
        Unpacks frame data from the binary stream.
        Returns a dictionary with parsed data and the new offset.
        """
        offset = 0
        frame_data = {}

        try:
            # Ensure there's enough data for frame number and timestamp
            if len(data) < offset + 4 + 8 + 4:
                rospy.logwarn("Insufficient data for frame_number, timestamp, and rigid_body_count.")
                return frame_data, offset

            # Unpack frame number (4 bytes)
            frame_number = int.from_bytes(data[offset:offset+4], byteorder='little', signed=True)
            frame_data['frame_number'] = frame_number
            offset += 4

            # Unpack timestamp (8 bytes)
            timestamp, = struct.unpack_from('<d', data, offset)
            frame_data['timestamp'] = timestamp
            offset += 8

            # Unpack rigid body count (4 bytes)
            rigid_body_count = int.from_bytes(data[offset:offset+4], byteorder='little', signed=True)
            frame_data['rigid_body_count'] = rigid_body_count
            offset += 4

            rospy.logdebug(f"Frame Number: {frame_number}, Timestamp: {timestamp}, Rigid Body Count: {rigid_body_count}")

            # Iterate through each rigid body
            rigid_bodies = []
            for i in range(rigid_body_count):
                expected_length = offset + 4 + 12 + 16
                if len(data) < expected_length:
                    rospy.logwarn(f"Insufficient data for rigid body {i+1}. Expected {expected_length} bytes, got {len(data)} bytes.")
                    break

                rigid_body = {}

                # Unpack rigid body ID (4 bytes)
                rb_id = int.from_bytes(data[offset:offset+4], byteorder='little', signed=True)
                rigid_body['id'] = rb_id
                offset += 4

                # Unpack position (12 bytes)
                pos = struct.unpack_from(VECTOR3_FORMAT, data, offset)
                rigid_body['position_x'] = pos[0]
                rigid_body['position_y'] = pos[1]
                rigid_body['position_z'] = pos[2]
                offset += 12

                # Unpack orientation (16 bytes)
                rot = struct.unpack_from(QUATERNION_FORMAT, data, offset)
                rigid_body['orientation_x'] = rot[0]
                rigid_body['orientation_y'] = rot[1]
                rigid_body['orientation_z'] = rot[2]
                rigid_body['orientation_w'] = rot[3]
                offset += 16

                rospy.logdebug(f"Rigid Body {i+1}: ID={rb_id}, Position={pos}, Orientation={rot}")

                rigid_bodies.append(rigid_body)

            frame_data['rigid_bodies'] = rigid_bodies

            return frame_data, offset

        except struct.error as e:
            rospy.logerr(f"Struct unpacking error: {e}")
            return frame_data, offset
        except Exception as e:
            rospy.logerr(f"Unexpected error during frame data unpacking: {e}")
            return frame_data, offset

    def publish_frame_data(self, frame_data: dict):
        """
        Publishes the parsed frame data to respective ROS topics.
        """
        rigid_bodies = frame_data.get('rigid_bodies', [])
        if not rigid_bodies:
            rospy.logwarn("No rigid bodies found in frame data.")
            return

        for rb in rigid_bodies:
            rb_id = rb.get('id')
            rb_name = self.get_rigid_body_name(rb_id)
            if not rb_name:
                rospy.logwarn(f"Rigid body ID {rb_id} not found in configuration.")
                continue

            # Publish Pose
            pose_msg = self.parse_pose(rb)
            self.publishers[rb_name]['pose'].publish(pose_msg)

            # Publish Pose2D
            pose2d_msg = self.parse_pose2d(rb)
            self.publishers[rb_name]['ground_pose'].publish(pose2d_msg)

            # Publish Odometry
            odom_msg = self.parse_odometry(rb)
            self.publishers[rb_name]['odom'].publish(odom_msg)

            rospy.logdebug(f"Published data for rigid body: {rb_name}")

    def get_rigid_body_name(self, rb_id):
        """
        Maps rigid body ID to its name based on configuration.
        Assumes that the configuration file contains a list of rigid body names
        corresponding to their IDs (e.g., index + 1).
        """
        rigid_body_names = self.config.get('rigid_body_names', [])
        if 1 <= rb_id <= len(rigid_body_names):
            return rigid_body_names[rb_id - 1]  # Assuming IDs start at 1
        else:
            rospy.logwarn(f"Rigid body ID {rb_id} is out of range.")
            return None

    def parse_pose(self, rb: dict) -> Pose:
        """
        Parses pose data from rigid body dictionary and returns a ROS Pose message.
        """
        pose_msg = Pose()
        try:
            # Assign positions
            pose_msg.position.x = rb.get('position_x', 0.0)
            pose_msg.position.y = rb.get('position_y', 0.0)
            pose_msg.position.z = rb.get('position_z', 0.0)

            # Assign orientations
            pose_msg.orientation.x = rb.get('orientation_x', 0.0)
            pose_msg.orientation.y = rb.get('orientation_y', 0.0)
            pose_msg.orientation.z = rb.get('orientation_z', 0.0)
            pose_msg.orientation.w = rb.get('orientation_w', 1.0)

        except KeyError as e:
            rospy.logerr(f"Missing key in Pose data: {e}")

        return pose_msg

    def parse_pose2d(self, rb: dict) -> Pose2D:
        """
        Parses 2D pose data from rigid body dictionary and returns a ROS Pose2D message.
        """
        pose2d_msg = Pose2D()
        try:
            pose2d_msg.x = rb.get('position_x', 0.0)
            pose2d_msg.y = rb.get('position_y', 0.0)
            # Calculate theta from orientation quaternion
            qx = rb.get('orientation_x', 0.0)
            qy = rb.get('orientation_y', 0.0)
            qz = rb.get('orientation_z', 0.0)
            qw = rb.get('orientation_w', 1.0)
            # Assuming yaw (theta) is the rotation around Z-axis
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            theta = math.atan2(siny_cosp, cosy_cosp)
            pose2d_msg.theta = theta

        except KeyError as e:
            rospy.logerr(f"Missing key in Pose2D data: {e}")

        return pose2d_msg

    def parse_odometry(self, rb: dict) -> Odometry:
        """
        Parses odometry data from rigid body dictionary and returns a ROS Odometry message.
        """
        odom_msg = Odometry()
        try:
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "odom"

            # Assign pose
            odom_msg.pose.pose.position.x = rb.get('position_x', 0.0)
            odom_msg.pose.pose.position.y = rb.get('position_y', 0.0)
            odom_msg.pose.pose.position.z = rb.get('position_z', 0.0)

            odom_msg.pose.pose.orientation.x = rb.get('orientation_x', 0.0)
            odom_msg.pose.pose.orientation.y = rb.get('orientation_y', 0.0)
            odom_msg.pose.pose.orientation.z = rb.get('orientation_z', 0.0)
            odom_msg.pose.pose.orientation.w = rb.get('orientation_w', 1.0)

            # Assign twist as zero (since velocity data isn't provided)
            odom_msg.twist.twist.linear.x = 0.0
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = 0.0

        except KeyError as e:
            rospy.logerr(f"Missing key in Odometry data: {e}")

        return odom_msg

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    default_config = os.path.join(rospack.get_path('multicast_parser'), 'config', 'config.yaml')

    config_path = rospy.get_param('~config', default_config)
    parser = MulticastDataParser(config_path)
    parser.run()