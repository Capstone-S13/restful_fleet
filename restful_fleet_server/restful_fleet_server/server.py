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

from restful_fleet_server.server_node import ServerNode


class Server():
    # Note: This config is a different config from the server node
    def __init__(self,config):
        self.logger = None
        self.config = config

    def send_mode_request(self, json_msg):
        url = self.get_url(self.config.mode_request_route)
        try:
            resp = requests.post(url, json=json_msg)
            self.logger.info(f"mode request send status: {resp.status_code}")
        except:
            self.logger.info("client not up")

    def send_path_request(self, json_msg):
        url = self.get_url(self.config.path_request_route)
        try:
            resp = requests.post(url, json=json_msg)
            self.logger.info(f"path request send status: {resp.status_code}")
        except:
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

