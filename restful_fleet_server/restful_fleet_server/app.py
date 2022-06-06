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



app = Flask(__name__)
CORS(app, origins=r"/*")
socketio = SocketIO(app, async_mode='threading')
socketio.init_app(app, cors_allowed_origins="*")
from restful_fleet_server.server_node_config import ServerNodeConfig
from restful_fleet_server.server_config import ServerConfig
from restful_fleet_server.server import Server
from restful_fleet_server.server import ServerNode

server_config = ServerConfig()
server_node_config = ServerNodeConfig()
server = Server(server_config)
server_node = ServerNode(server_node_config, server)


@app.route(server_config.robot_state_route, methods=['POST'])
def handle_robot_state():
    # save robot state into a dictionary
    robot_state_json = request.json
    if (robot_state_json["fleet_name"] != server_node_config.fleet_name):
        return
    server_node.update_robot_state(robot_state_json)

@app.route(server_config.end_action_route, methods=['POST'])
def end_perform_action():
    #
    robot = request.json["robot"]
    server_node.publish_end_action(robot)
