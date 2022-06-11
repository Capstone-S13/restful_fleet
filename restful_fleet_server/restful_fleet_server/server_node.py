import imp
from re import M
import rclpy
import time
import json

from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter

# Qos
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

# RMF
from rmf_fleet_msgs.msg import FleetState, ModeRequest, DestinationRequest, \
    PathRequest, RobotMode, Location, RobotState
from std_msgs.msg import String

from restful_fleet_server.server_node_config import ServerNodeConfig

class ServerNode(Node):
    def __init__(self, server, config=ServerNodeConfig()):
        super().__init__(config.node_name)
        self.transient_qos = QoSProfile(
                history=History.KEEP_LAST,
                depth=1,
                reliability=Reliability.RELIABLE,
                durability=Durability.TRANSIENT_LOCAL)
        self.server = server
        self.server.logger = self.get_logger()
        self.timer = self.create_timer(config.timer_period, self.timer_callback)
        self.robots = {}

        self.fleet_state_pub = self.create_publisher(FleetState,
            config.fleet_state_topic, config.fleet_state_pub_rate)

        self.end_action_pub = self.create_publisher(ModeRequest,
            config.action_execution_notice_topic,
            self.transient_qos)

        self.path_request_sub = self.create_subscription(PathRequest,
            config.path_request_topic,
            self.path_request_cb,
            config.sub_rate)

        self.perfrom_action_sub = self.create_subscription(String,
            config.perform_action_topic,
            self.perform_action_request_cb,
            config.sub_rate)

        self.destination_request_sub = self.create_subscription(
            DestinationRequest,
            config.destination_request_topic,
            self.destination_request_cb,
            config.sub_rate)

        self.config = config

    def path_request_cb(self, _msg):
        self.handle_path_request(_msg)
        return

    def perform_action_request_cb(self, _msg):
        self.handle_perform_action_request(_msg)
        return

    def destination_request_cb(self,_msg):
        self.handle_destination_request(_msg)
        return

    # this takes in type rmf_fleet_msgs.msg.Location
    def transform_fleet_to_rmf(self, rmf_frame_location):
        fleet_frame_location = Location()
        return fleet_frame_location

    def handle_destination_request(self, _msg):
        json_msg = {}
        json_msg["fleet_name"] = _msg.fleet_name
        json_msg["robot_name"] = _msg.robot_name
        json_msg["destination"] = self.transform_fleet_to_rmf(_msg.destination)
        self.server.send_destination_request(json_msg)
        self.get_logger().info("sending path request")

    def handle_path_request(self, _msg):
        self.get_logger().info(f"receive path request of fleet: {_msg.fleet_name}")
        if _msg.fleet_name != self.config.fleet_name:
            return
        json_msg = {}
        json_msg["fleet_name"] = _msg.fleet_name
        json_msg["robot_name"] = _msg.robot_name
        json_msg["path"] = []
        for i in range(len(_msg.path)):
            location = _msg.path[i]
            # location = self.transform_fleet_to_rmf(location)
            loc_json = self.convert_location_to_json(location)
            json_msg["path"].append(loc_json)
        json_msg["task_id"] = _msg.task_id
        self.server.send_path_request(json_msg)
        self.get_logger().info("sending path request")

    def handle_mode_request(self, _msg):
        return

    def handle_perform_action_request(self, _msg):
        json_msg = {}
        json_msg["data"] = _msg.data
        self.server.send_perform_action_request(json_msg)
        self.get_logger().info(f"sending perform action request: {_msg}")
        return

    def convert_location_to_json(self, location):
        locaction_json = {}
        locaction_json["t"] = {"sec" : location.t.sec,
            "nanosec": location.t.nanosec}
        locaction_json["x"] = location.x
        locaction_json["y"] = location.y
        locaction_json["yaw"] = location.yaw
        locaction_json["obey_approach_speed_limit"] = False
        locaction_json["approach_speed_limit"] = location.approach_speed_limit
        locaction_json["level_name"] = location.level_name
        locaction_json["index"] = location.index
        return locaction_json

    def convert_json_to_location(self, location_json) -> Location:
        location = Location()
        location.t.sec = int(location_json["t"]["sec"])
        location.t.nanosec = int(location_json["t"]["nanosec"])
        location.x = float(location_json["x"])
        location.y = float(location_json["y"])
        location.yaw = float(location_json["yaw"])
        # location.obey_approach_speed_limit = \
        #     location_json["obey_approach_speed_limit"]
        # location.approach_speed_limit = \
        #     location_json["approach_speed_limit"]
        location.level_name = location_json["level_name"]
        # location.index = location_json["index"]
        return location

    def convert_json_to_robot_mode(self, robot_mode_json) -> RobotMode:
        robot_mode = RobotMode()
        return robot_mode

    def convert_robot_mode_to_json(self, robot_mode):
        # TODO: find a better way to do this
        robot_mode_json = {}
        if robot_mode.mode == RobotMode.MODE_IDLE:
            robot_mode_json["mode"] = 0

        elif robot_mode.mode == RobotMode.MODE_CHARGING:
            robot_mode_json["mode"] = 1

        elif robot_mode.mode == RobotMode.MODE_MOVING:
            robot_mode_json["mode"] = 2

        elif robot_mode.mode == RobotMode.MODE_MOVING:
            robot_mode_json["mode"] = 3

        return


    def publish_fleet_state(self):
        # for each robot in the robot dictionary
        # convert fleet state frame into rmf frame
        fleet_state = FleetState()
        fleet_state.name = self.config.fleet_name
        for robot in self.robots:
            fleet_state.robots.append(self.robots[robot])
        self.fleet_state_pub.publish(fleet_state)
        self.get_logger().info("Publishing fleet_states")

    def publish_end_action(self, robot):
        mode_request = ModeRequest()
        mode_request.fleet_name = self.config.fleet_name
        mode_request.robot_name = robot
        mode_request.mode.mode = RobotMode.MODE_IDLE
        mode_request.task_id = "end_action"
        self.end_action_pub.publish(mode_request)

    def update_robot_state(self, json_msg):
        if json_msg["name"] not in self.robots:
            self.get_logger().info(f"registered new robot {json_msg['name']}")
        robot_state = RobotState()
        robot_state.name = json_msg["name"]
        robot_state.model = json_msg["model"]
        robot_state.task_id = json_msg["task_id"]
        robot_state.battery_percent = float(json_msg["battery_percent"])
        # convert robot mode
        robot_state.mode.mode = json_msg["robot_mode"]
        robot_state.location = \
            self.convert_json_to_location(json_msg["location"])
        for location_json in json_msg["path"]:
            robot_state.path.append(self.convert_json_to_location(location_json))
        self.robots[json_msg["name"]] = robot_state

    def timer_callback(self):
        self.publish_fleet_state()


