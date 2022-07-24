import imp
from re import M


import rclpy
import time
import json
import numpy as np

from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter
from threading import Semaphore, Thread

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
        self.robot_state_sem = Semaphore()

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

        self.spin_thread = Thread(target=self.spin_self)

        self.config = config

        self.rmf_to_fleet_mat =self.init_rmf_to_fleet_mat()

        self.fleet_to_rmf_mat = self.init_fleet_to_rmf_mat()

    def init_rmf_to_fleet_mat(self):
        return np.array([
            [np.cos(self.config.map_rotation), -np.sin(self.config.map_rotation),
            0.0, self.config.map_translate_x],

            [np.sin(self.config.map_rotation), np.cos(self.config.map_rotation),
            0.0, self.config.map_translate_y],

            [0.0, 0.0, 1.0, 0.0],

            [0.0, 0.0, 0.0,1.0]
        ])

    def init_fleet_to_rmf_mat(self):
        # origin scaled to map scale
        scaled_origin = np.array([
            [self.config.map_translate_x],
            [self.config.map_translate_y],
            [0.0]
        ])

        inverse_rot = np.zeros((3,3))

        # get inverse of rmf to fleet rotation matrix
        for i in range(3):
            for j in range(3):
                inverse_rot[j][i] = self.rmf_to_fleet_mat[i][j]

        # get rmf origin wrt to fleet origin
        inverse_origin = np.matmul(-inverse_rot, scaled_origin)

        # initialise the fleet to rmf transform matrix
        fleet_to_rmf_mat = np.zeros((4,4))
        for i in range(len(inverse_rot)):
            for j in range(len(inverse_rot[i])):
                fleet_to_rmf_mat[i][j] = inverse_rot[j][i]

        # populate the fleet to rmf transform martix with values
        for i in range(len(fleet_to_rmf_mat)):
           if i < 3:
               fleet_to_rmf_mat[i][-1] = inverse_origin[i][0]
           else:
               fleet_to_rmf_mat[i][-1] = 1.0
        return fleet_to_rmf_mat

    def spin_self(self):
        rclpy.spin(self)

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
    def transform_fleet_to_rmf(self, fleet_frame_location:Location):
        fleet_frame_mat = np.zeros((4,1))
        fleet_frame_mat[0][0] = fleet_frame_location.x
        fleet_frame_mat[1][0] = fleet_frame_location.y
        fleet_frame_mat[3][0] = 1.0
        rmf_frame_mat = np.matmul(self.fleet_to_rmf_mat, fleet_frame_mat)
        rmf_frame_mat = rmf_frame_mat*self.config.map_scale
        fleet_frame_location.x = rmf_frame_mat[0][0]
        fleet_frame_location.y = rmf_frame_mat[1][0]
        fleet_frame_location.yaw = fleet_frame_location.yaw - self.config.map_rotation
        return fleet_frame_location

    def transform_rmf_to_fleet(self, rmf_frame_location:Location):
        rmf_frame_mat = np.zeros((4,1))
        rmf_frame_mat[0][0] = rmf_frame_location.x
        rmf_frame_mat[1][0] = rmf_frame_location.y
        rmf_frame_mat[3][0] = 1.0
        fleet_frame_mat = np.matmul(self.rmf_to_fleet_mat, rmf_frame_mat)
        fleet_frame_mat = fleet_frame_mat/self.config.map_scale
        rmf_frame_location.x = fleet_frame_mat[0][0]
        rmf_frame_location.y = fleet_frame_mat[1][0]
        rmf_frame_location.yaw = rmf_frame_location.yaw + self.config.map_rotation
        return rmf_frame_location

    def handle_destination_request(self, _msg):
        json_msg = {}
        json_msg["fleet_name"] = _msg.fleet_name
        json_msg["robot_name"] = _msg.robot_name
        json_msg["destination"] = self.transform_rmf_to_fleet(_msg.destination)
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
            location = self.transform_rmf_to_fleet(location)
            loc_json = self.convert_location_to_json(location)
            json_msg["path"].append(loc_json)
        json_msg["task_id"] = _msg.task_id
        self.get_logger().info(
            f"sending path request of length {len(json_msg['path'])}")
        self.server.send_path_request(json_msg)
        self.get_logger().info(
            f"path request sent!")
        return

    def handle_mode_request(self, _msg):
        return

    def handle_perform_action_request(self, _msg):
        json_msg = {}
        json_msg["data"] = _msg.data
        self.server.send_perform_action_request(json_msg)
        self.get_logger().info(f"sending perform action request: {_msg}")
        return

    def convert_location_to_json(self, location):
        location_json = {}
        location_json["t"] = {"sec" : location.t.sec,
            "nanosec": location.t.nanosec}
        location_json["x"] = location.x
        location_json["y"] = location.y
        location_json["yaw"] = location.yaw
        location_json["obey_approach_speed_limit"] = False
        location_json["approach_speed_limit"] = location.approach_speed_limit
        location_json["level_name"] = location.level_name
        location_json["index"] = location.index
        return location_json

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
            self.robot_state_sem.acquire()
            fleet_state.robots.append(self.robots[robot])
            self.robot_state_sem.release()
        self.fleet_state_pub.publish(fleet_state)
        # self.get_logger().info("Publishing fleet_states")

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
        robot_state.battery_percent = 100.0
        # convert robot mode
        robot_state.mode.mode = json_msg["robot_mode"]
        robot_state.location = \
            self.transform_fleet_to_rmf(
                self.convert_json_to_location(json_msg["location"])
            )
        for location_json in json_msg["path"]:
            robot_state.path.append(
                self.transform_fleet_to_rmf(
                    self.convert_json_to_location(location_json)
                )
            )
        # self.get_logger().info(robot_state)
        self.robot_state_sem.acquire()
        self.robots[json_msg["name"]] = robot_state
        self.robot_state_sem.release()

    def timer_callback(self):
        self.publish_fleet_state()


