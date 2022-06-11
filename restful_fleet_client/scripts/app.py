#! /usr/bin/env python3

from distutils.command.config import config
import rospy
from http import HTTPStatus
from threading import Thread

from flask import Flask, request, jsonify
from flask_cors import CORS
from flask_socketio import SocketIO, emit, disconnect


# from client import Client
# from client_config import ClientConfig
# from client_node import ClientNode
# from client_node_config import ClientNodeConfig

from restful_fleet_client.client import Client
from restful_fleet_client.client_config import ClientConfig
from restful_fleet_client.client_node import ClientNode
from restful_fleet_client.client_node_config import ClientNodeConfig


# in order to create the nodes
rospy.init_node('restful_client_server', anonymous=True)
rate = rospy.Rate(3)

_client_config = ClientConfig()
_client_node_config = ClientNodeConfig()
_client = Client(_client_config)
_client_node = ClientNode( _client_node_config, _client)

app = Flask(__name__)
CORS(app, origins=r"/*")


socketio = SocketIO(app, async_mode='threading')
socketio.init_app(app, cors_allowed_origins="*")


@app.route(_client_config.path_request_route, methods=['POST'])
def handle_path_request():
    path_request_json = request.json
    rospy.loginfo(f"received path request: {path_request_json}")
    _client_node.receive_path_request(path_request_json)
    response = app.response_class(status=200)
    return response

@app.route(_client_config.mode_request_route, methods=['POST'])
def handle_mode_request():
    mode_request_json = request.json
    rospy.loginfo(f"received mode request: {mode_request_json}")
    _client_node.receive_mode_request(mode_request_json)
    response = app.response_class(status=200)
    return response

@app.route(_client_config.perform_action_route, methods=['POST'])
def handle_perform_action():
    perform_action_request = request.json
    rospy.loginfo(f"received perform action request: {perform_action_request}")
    try:
        if (perform_action_request["robot"] != _client_node.config.robot_name):
            return
        try:
            _client_node.receive_perform_action(perform_action_request)
            response = app.response_class(status=200)
            return response
        except:
            rospy.loginfo("server not up")
    except:
        rospy.loginfo("no such attribute 'robot' in request")

def restful_client_spin():
    rospy.spin()

def loop_spin():
    while not rospy.is_shutdown():
        _client_node.loop()
        rate.sleep()

def main():
    client_ip = "0.0.0.0"
    port_num = 9001


    # spin node
    print("starting to spin node")
    spin_thread = Thread(target=restful_client_spin, args=())
    spin_thread.start()
    print("starting loop thread")
    loop_thread = Thread(target=loop_spin, args=())
    loop_thread.start()

    print(f"set Restful client port to:{client_ip}:{port_num}")
    app.run(host=client_ip, port=port_num, debug=True, use_reloader=False)
    loop_thread.join()
    spin_thread.join()
    rospy.signal_shutdown("shutdown")

if __name__ =='__main__':
    main()