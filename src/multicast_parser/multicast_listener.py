#!/usr/bin/env python

import rospy
import socket
import struct
import yaml
from std_msgs.msg import UInt8MultiArray
import rospkg
import os

class MulticastListener:
    def __init__(self):
        """
        Initialize the MulticastListener node.
        """
        rospy.init_node('multicast_listener', anonymous=True)
        self.pub = rospy.Publisher('multicast_data', UInt8MultiArray, queue_size=10)
        
        # Retrieve the configuration file path from ROS parameters
        config_path = rospy.get_param('~config', None)
        if not config_path:
            rospy.logerr("Configuration file path not provided. Shutting down node.")
            rospy.signal_shutdown("Missing configuration file path.")
            return

        self.config = self.load_config(config_path)
        if not self.config:
            rospy.signal_shutdown("Failed to load configuration.")
            return

        self.multicast_group = self.config.get('multicast_group', None)
        self.multicast_port = self.config.get('multicast_port', None)
        self.server_address = self.config.get('server_address', None)

        if not all([self.multicast_group, self.multicast_port, self.server_address]):
            rospy.logerr("Incomplete configuration parameters. Please check config.yaml.")
            rospy.signal_shutdown("Incomplete configuration.")
            return

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.mreq = None
        rospy.loginfo(f"Multicast listener initialized with: Multicast group: {self.multicast_group}, Port: {self.multicast_port}, Server address: {self.server_address}")

    def load_config(self, config_path):
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
            return None

    def setup_socket(self):
        """
        Set up the multicast socket.
        """
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(('', self.multicast_port))  

            self.mreq = struct.pack("4sl", socket.inet_aton(self.multicast_group), socket.INADDR_ANY)
            self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, self.mreq)

            rospy.logdebug(f"Listening for multicast packets on {self.multicast_group}:{self.multicast_port}...")
        except Exception as e:
            rospy.logerr(f"Socket setup failed: {e}")
            rospy.signal_shutdown("Socket setup failure")

    def listen(self):
        """
        Listen for multicast messages and publish them to a ROS topic.
        """
        buffer_size = self.config.get('buffer_size', 1024)
        try:
            while not rospy.is_shutdown():
                # Receive data
                data, address = self.sock.recvfrom(buffer_size)  # Buffer size is 1024 bytes
                rospy.logdebug(f"Received data from {address}: {data}")
                if address[0] == self.server_address:
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
                    self.pub.publish(msg)
                    rospy.logdebug(f"Published data: {msg.data}")
        except rospy.ROSInterruptException:
            rospy.loginfo("Node shutdown requested.")
        except Exception as e:
            rospy.logerr(f"An error occurred: {e}")
        finally:
            self.shutdown()

    def shutdown(self):
        """
        Clean up the socket and leave the multicast group.
        """
        try:
            if self.mreq:
                # Leave the multicast group
                self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, self.mreq)
            self.sock.close()
            rospy.loginfo("Multicast listener stopped.")
        except Exception as e:
            rospy.logerr(f"Error during shutdown: {e}")

if __name__ == '__main__':
    listener = MulticastListener()
    if listener.config:
        listener.setup_socket()
        listener.listen()
