from re import S
import sys
import os
import rclpy
import argparse
import time
import json
import logging
import requests
from http import HTTPStatus
from threading import Thread

from flask import Flask, request, jsonify
from flask_cors import CORS
from flask_socketio import SocketIO, emit, disconnect
import asyncio

from restful_fleet_server.server_node_config import ServerNodeConfig
from restful_fleet_server.server_config import ServerConfig
from restful_fleet_server.server import Server
from restful_fleet_server.server import ServerNode

# in order to create the nodes
rclpy.init(args=None)

server_config = ServerConfig()
server_node_config = ServerNodeConfig()
server = Server(server_config)
server_node = ServerNode(server, server_node_config)

app = Flask(__name__)
CORS(app, origins=r"/*")


socketio = SocketIO(app, async_mode='threading')
socketio.init_app(app, cors_allowed_origins="*")


@app.route(server_config.robot_state_route, methods=['POST'])
def handle_robot_state():
    # save robot state into a dictionary
    robot_state_json = request.json
    server_node.get_logger().info(f"robot state: {robot_state_json}")
    if (robot_state_json["fleet_name"] != server_node_config.fleet_name):
        response = app.response_class(status=404)
        return response
    server_node.update_robot_state(robot_state_json)
    response = app.response_class(status=200)
    return response

@app.route(server_config.end_action_route, methods=['POST'])
def end_perform_action():
    #
    robot = request.json["robot"]
    server_node.publish_end_action(robot)
    response = app.response_class(status=200)

def restful_server_spin():
    rclpy.spin(server_node)


def main():
    server_ip = "0.0.0.0"
    port_num = 9000
    print(f"set Restful Server port to:{server_ip}:{port_num}")
    # spin node

    spin_thread = Thread(target=restful_server_spin, args=())
    spin_thread.start()
    app.run(host=server_ip, port=port_num, debug=True)
    rclpy.shutdown()

if __name__ =='__main__':
    main()