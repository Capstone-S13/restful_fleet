import sys
import os
import rospy
import argparse
import time
import json
import logging
import requests
from http import HTTPStatus
from threading import Thread

from flask import Config, Flask, request, jsonify
from flask_cors import CORS
from flask_socketio import SocketIO, emit, disconnect
import asyncio

# from client_config import ClientConfig
from restful_fleet_client.client_config import ClientConfig

class Client():
    def __init__(self, config):
        self.config = config

    def send_battery_state(self, json_msg):
        url = self.get_url(self.config.battery_state_route)
        try:
            resp = requests.post(url, json=json_msg)
            rospy.loginfo(f"battery state send status {resp.status_code}")
        except:
            rospy.loginfo("server not up")

    def send_robot_state(self, json_msg):
        url = self.get_url(self.config.robot_state_route)
        try:
            resp = requests.post(url, json=json_msg)
            rospy.loginfo(f"robot state send status {resp.status_code}")
        except:
            rospy.loginfo("server not up")

    def send_end_action(self, json_msg):
        url = self.get_url(self.config.end_action_route)
        try:
            resp = requests.post(url, json=json_msg)
            rospy.loginfo(f"end action send status {resp.status_code}")
        except:
            rospy.loginfo("server not up")


    def get_url(self, route):
        url = 'http://' + self.config.server_ip + ':' +\
            str(self.config.server_port) + route
        return url

