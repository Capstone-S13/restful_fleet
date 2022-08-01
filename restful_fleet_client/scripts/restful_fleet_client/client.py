from imp import reload
import rospy
import requests
from http import HTTPStatus

from flask import Flask, request
from flask_socketio import SocketIO

from restful_fleet_client.client_config import ClientConfig
from restful_fleet_client.client_node_config import ClientNodeConfig
from restful_fleet_client.client_node import ClientNode

class Client():
    def __init__(self, client_config=ClientConfig(), client_node_config=ClientNodeConfig()):
        self.client_config = client_config
        self.client_node_config = client_node_config
        self.client_node = ClientNode(client_node_config, self)
        self.app = Flask('restful_fleet_client')
        self.socketio = SocketIO(self.app, async_mode="threading")
        self.socketio.init_app(self.app, cors_allowed_origins="*")

        @self.app.route(self.client_config.path_request_route, methods=['POST'])
        def handle_path_request():
            path_request_json = request.json
            rospy.loginfo(f"received path request of id: {path_request_json['task_id']}\
                , of length {len(path_request_json['path'])}")
            response = self.app.response_class(status=HTTPStatus.NOT_ACCEPTABLE.value)
            if self.client_node.receive_path_request(path_request_json):
                rospy.loginfo("request is valid")
                response = self.app.response_class(status=200)
            return response

        @self.app.route(self.client_config.mode_request_route, methods=['POST'])
        def handle_mode_request():
            mode_request_json = request.json
            rospy.loginfo(f"received mode request: {mode_request_json}")
            self.client_node.receive_mode_request(mode_request_json)
            response = self.app.response_class(status=200)
            return response

        @self.app.route(self.client_config.perform_action_route, methods=['POST'])
        def handle_perform_action():
            perform_action_request = request.json
            rospy.loginfo(f"received perform action request for action: {perform_action_request['category']}")
            response = self.app.response_class(status=HTTPStatus.NOT_ACCEPTABLE.value)
            try:
                if (perform_action_request["robot"] != self.client_node.config.robot_name):
                    return response
                try:
                    self.client_node.receive_perform_action(perform_action_request)
                    response = self.app.response_class(status=200)
                    return response
                except Exception as e:
                    rospy.loginfo(f'Exception {e}')
                    return response
            except Exception as e:
                rospy.loginfo(f'Exception {e}')
                return response

    def run_server(self):
        self.app.run(host= self.client_config.client_ip,
            port=self.client_config.client_port, use_reloader=False)

    def start(self):
        self.client_node.start_threads()
        self.run_server()

    def send_battery_state(self, json_msg):
        url = self.get_url(self.client_config.battery_state_route)
        try:
            resp = requests.post(url, json=json_msg)
            rospy.loginfo(f"battery state send status {resp.status_code}")
        except:
            rospy.loginfo("server not up")

    def send_robot_state(self, json_msg):
        url = self.get_url(self.client_config.robot_state_route)
        try:
            resp = requests.post(url, json=json_msg)
            # rospy.loginfo(f"robot state send status {resp.status_code}")
        except:
            rospy.loginfo("server not up")

    def send_end_action(self, json_msg):
        url = self.get_url(self.client_config.end_action_route)
        try:
            resp = requests.post(url, json=json_msg)
            rospy.loginfo(f"end action send status {resp.status_code}")
        except:
            rospy.loginfo("server not up")


    def get_url(self, route):
        url = 'http://' + self.client_config.server_ip + ':' +\
            str(self.client_config.server_port) + route
        return url

