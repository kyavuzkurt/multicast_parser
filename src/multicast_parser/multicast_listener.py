#!/usr/bin/env python

import rospy
import socket
import struct
import yaml
import argparse
from std_msgs.msg import String

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

def multicast_listener(config_path):
    rospy.init_node('multicast_listener', anonymous=True)
    pub = rospy.Publisher('multicast_data', String, queue_size=10)

    config = load_config(config_path)

    multicast_group = config['multicast_group']
    multicast_port = config['multicast_port']
    server_address = config['server_address']

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)


    sock.bind(('', multicast_port))  

    mreq = struct.pack("4sl", socket.inet_aton(multicast_group), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    rospy.loginfo(f"Listening for multicast packets on {multicast_group}:{multicast_port}...")

    log_level = config.get('log_level', 'INFO')
    buffer_size = config.get('buffer_size', 1024)

    try:
        while not rospy.is_shutdown():
            # Receive data
            data, address = sock.recvfrom(buffer_size)  # Buffer size is 1024 bytes
            rospy.loginfo(f"Received data from {address}: {data.decode('utf-8', errors='ignore')}")
            if address[0] == server_address:
                # Publish the data to the ROS topic
                msg = String()
                msg.data = data.decode('utf-8', errors='ignore')
                pub.publish(msg)
                rospy.loginfo(f"Published data: {msg.data}")
    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutdown requested.")
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
    finally:
        # Leave the multicast group
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, mreq)
        sock.close()
        rospy.loginfo("Multicast listener stopped.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Multicast Listener Node')
    parser.add_argument('--config', type=str, default='config/config.yaml',
                        help='Path to the configuration file')
    args = parser.parse_args()
    try:
        multicast_listener(args.config)
    except rospy.ROSInterruptException:
        pass
