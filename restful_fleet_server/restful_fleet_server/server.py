import sys
import os
from urllib.error import HTTPError

import requests
from http import HTTPStatus
from threading import Thread

from flask import Flask, request, jsonify
from flask_cors import CORS
from flask_socketio import SocketIO, emit, disconnect
import asyncio

from restful_fleet_server.server_config import ServerConfig
from restful_fleet_server.server_node import ServerNode
from restful_fleet_server.server_node_config import ServerNodeConfig


class Server():
    # Note: This config is a different config from the server node
    def __init__(self,config=ServerConfig(), server_node_config=ServerNodeConfig()):
        self.logger = None
        self.config = config
        self.server_node_config = server_node_config
        self.server_node = ServerNode(self, server_node_config)
        self.app = Flask('restful_fleet_server')
        self.socketio = SocketIO(self.app, async_mode='threading')
        self.socketio.init_app(self.app, cors_allowed_origins="*")

        @self.app.route(self.config.robot_state_route, methods=['POST'])
        def handle_robot_state():
            # save robot state into a dictionary
            robot_state_json = request.json
            if (robot_state_json["fleet_name"] != self.server_node_config.fleet_name):
                response = self.app.response_class(status=404)
                return response
            self.server_node.update_robot_state(robot_state_json)
            response = self.app.response_class(status=200)
            return response

        @self.app.route(self.config.end_action_route, methods=['POST'])
        def end_perform_action():
            #
            robot = request.json["robot"]
            self.server_node.publish_end_action(robot)
            return self.app.response_class(status=200)

    def start(self):
        self.server_node.spin_thread.start()
        self.run_server()

    def run_server(self):
        self.app.run(host=self.config.server_ip, port=self.config.server_port)

    def send_mode_request(self, json_msg):
        url = self.get_url(self.config.mode_request_route)
        try:
            resp = requests.post(url, json=json_msg)
            # self.logger.info(f"mode request send status: {resp.status_code}")
        except:
            self.logger.info("client not up")

    def send_path_request(self, json_msg):
        url = self.get_url(self.config.path_request_route)
        try:
            resp = requests.post(url, json=json_msg)
            resp.raise_for_status()
            self.logger.info(f"path request send status: {resp.status_code}")
        except HTTPError as e:
            self.logger().info(f'HTTP error: {e}')
        except Exception as e:
            self.logger.info("client not up")


    def send_perform_action_request(self, json_msg):
        url = self.get_url(self.config.perform_action_request_route)
        try:
            resp = requests.post(url, json=json_msg)
            self.logger.info(f"perform_action request send status: {resp.status_code}")
        except:
            self.logger.info("client not up")


    def send_destination_request(self, json_msg):
        url = self.get_url(self.config.destination_request_route)
        try:
            resp = requests.post(url, json=json_msg)
            self.logger.info(f"destination request send status: {resp.status_code}")
        except:
            self.logger.info("client not up")

    def get_url(self, route):
        url = 'http://' + self.config.client_ip + ':' + str(self.config.client_port) + \
            route
        return url



