#! /usr/bin/python3
from restful_fleet_client.client_node_config import ClientNodeConfig
from restful_fleet_client.client_config import ClientConfig
from restful_fleet_client.client import Client
import rospy
import rclpy

def main():
    rclpy.init()
    rospy.init_node('restful_client')
    client_node_config = ClientNodeConfig()
    client_config = ClientConfig()
    client = Client(client_config, client_node_config)
    client.start()

if __name__ == '__main__':
    main()