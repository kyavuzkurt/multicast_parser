#!/usr/bin/env python

import rospy
import socket
import struct
import yaml
from std_msgs.msg import UInt8MultiArray
import rospkg
import os

def load_config(config_path):
    """
    Load configuration from a YAML file.

    :param config_path: Path to the configuration file.
    :return: Dictionary containing configuration parameters.
    """
    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
            rospy.loginfo(f"Configuration loaded from {config_path}")
            return config
    except Exception as e:
        rospy.logerr(f"Failed to load configuration file: {e}")
        rospy.signal_shutdown("Configuration load failure")
        return None

def multicast_listener():
    rospy.init_node('multicast_listener', anonymous=True)
    pub = rospy.Publisher('multicast_data', UInt8MultiArray, queue_size=10)

    # Retrieve the configuration file path from ROS parameters
    config_path = rospy.get_param('~config', None)
    if not config_path:
        rospy.logerr("Configuration file path not provided. Shutting down node.")
        rospy.signal_shutdown("Missing configuration file path.")
        return

    config = load_config(config_path)
    if not config:
        rospy.signal_shutdown("Failed to load configuration.")
        return

    multicast_group = config.get('multicast_group', None)
    multicast_port = config.get('multicast_port', None)
    server_address = config.get('server_address', None)

    if not all([multicast_group, multicast_port, server_address]):
        rospy.logerr("Incomplete configuration parameters. Please check config.yaml.")
        rospy.signal_shutdown("Incomplete configuration.")
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', multicast_port))  

        mreq = struct.pack("4sl", socket.inet_aton(multicast_group), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        rospy.logdebug(f"Listening for multicast packets on {multicast_group}:{multicast_port}...")

        buffer_size = config.get('buffer_size', 1024)

        while not rospy.is_shutdown():
            # Receive data
            data, address = sock.recvfrom(buffer_size)  # Buffer size is 1024 bytes
            rospy.logdebug(f"Received data from {address}: {data}")
            if address[0] == server_address:
                # Convert bytes to list of unsigned integers
                data_list = list(data)
                
                # Validate data range
                invalid_bytes = [b for b in data_list if not (0 <= b <= 255)]
                if invalid_bytes:
                    rospy.logerr(f"Invalid byte values detected: {invalid_bytes}")
                    continue  # Skip publishing invalid data

                # Optional: Log data range for debugging
                max_byte = max(data_list) if data_list else 0
                min_byte = min(data_list) if data_list else 0
                rospy.logdebug(f"Data range - Min: {min_byte}, Max: {max_byte}")

                # Publish the data to the ROS topic
                msg = UInt8MultiArray()
                msg.data = data_list  # Already a list of integers from 0-255
                pub.publish(msg)
                rospy.logdebug(f"Published data: {msg.data}")
    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutdown requested.")
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
    finally:
        if 'mreq' in locals():
            # Leave the multicast group
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, mreq)
        sock.close()
        rospy.loginfo("Multicast listener stopped.")

if __name__ == '__main__':
    multicast_listener()
