from asyncio import FastChildWatcher
from distutils.command.config import config
from flask import Config
from numpy import math

from requests import request
from rx import catch
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformListener, Buffer
from tf import transformations
from geometry_msgs.msg import TransformStamped

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib import SimpleActionClient, GoalStatus

# from  client import Client
# from  client_node_config import ClientNodeConfig
# from  utilities import is_transform_close, RobotMode

from  restful_fleet_client.client import Client
from  restful_fleet_client.client_node_config import ClientNodeConfig
from  restful_fleet_client.utilities import is_transform_close, RobotMode

class Goal():
    def __init__(self):
        self.level_name = ""
        self.goal = None
        self.sent = False
        self.aborted_count = 0
        self.goal_end_time = None


class ClientNode():
    def __init__(self,config, client):
        self.config = config
        self.client = client
        self.battery_sub = rospy.Subscriber(self.config.battery_topic,\
            BatteryState, self.battery_state_cb,queue_size=10)
        self.current_battery_state = BatteryState()
        self.previous_robot_transform = TransformStamped()
        self.current_robot_transform = TransformStamped()
        self.goal_path = []
        self.move_base_client = SimpleActionClient(self.config.move_base_server_name, MoveBaseAction)
        self.tf2_buffer = Buffer()
        self.tf2_listener = TransformListener(self.tf2_buffer)
        self.max_tries = 5
        self.emergency = False
        self.paused = False
        self.request_error= False
        self.current_task_id = ""

    def init(self):
        # in case we need to set up some stuff
        return

    def battery_state_cb(self, msg):
        self.current_battery_state = msg

    def is_valid_transform(self, request_fleet_name, request_robot_name,\
        request_task_id):

        return

    def is_valid_request(self, request_fleet_name, request_robot_name,\
        request_task_id) -> bool:
        if (self.current_task_id == request_task_id\
            or self.config.robot_name != request_robot_name\
            or self.config.fleet_name != request_fleet_name):
            rospy.loginfo("not a valid request")
            return False
        return True

    def get_robot_transform(self) -> bool:
        try:
            tmp_transform = self.tf2_buffer.lookup_transform(\
                self.config.map_frame,
                self.config.robot_frame,
                rospy.Time(0))

            self.previous_robot_transform = self.current_robot_transform
            self.current_robot_transform = tmp_transform
            # rospy.loginfo("updating robot transform")

        except:
            rospy.logwarn(f"error getting transform")
            return False

        return True

    def get_robot_mode(self):
        if self.request_error:
            RobotMode.MODE_ADAPTER_ERROR.value
            return
        if self.emergency:
            RobotMode.MODE_EMERGENCY.value
            return
        if self.current_battery_state.power_supply_status ==\
            self.current_battery_state.POWER_SUPPLY_STATUS_CHARGING:
            # Robot is charging
            return RobotMode.MODE_CHARGING.value
        if is_transform_close(self.current_robot_transform,\
            self.previous_robot_transform):
            return RobotMode.MODE_MOVING.value
        if self.paused:
            return RobotMode.MODE_PAUSED.value

        # otherwise robot mode is idle
        return RobotMode.MODE_IDLE.value

    def receive_path_request(self, path_req_json) -> bool:
        rospy.loginfo("received path request")
        if (self.is_valid_request(path_req_json["fleet_name"],\
            path_req_json["robot_name"], path_req_json["task_id"])):
            if (len(path_req_json["path"]) <= 0):
                return False
            dx = path_req_json["path"][0]["x"] - self.current_robot_transform.transform.translation.x
            dy = path_req_json["path"][0]["y"] - self.current_robot_transform.transform.translation.y
            dist_to_first_waypoint = math.sqrt(dx**2 + dy**2)
            rospy.loginfo(f"Distance to first waypoint: {dist_to_first_waypoint}")

            if (dist_to_first_waypoint > self.config.max_dist_to_first_waypoint):
                rospy.logwarn(f"distance was over threshold of: \
                    {self.config.max_dist_to_first_waypoint}")
                self.move_base_client.cancel_all_goals()
                self.goal_path.clear()
                self.request_error = True
                self.emergency = False
                self.paused = False
                return False
            # add path to goal_path
            for i in range(len(path_req_json["path"])):
                goal = Goal()
                goal_json = path_req_json["path"][i]
                goal.level_name = goal_json["level_name"]
                goal.goal =\
                    self.location_to_move_base_goal(goal_json)
                goal.sent = False
                goal.goal_end_time = rospy.Time(int(goal_json["t"]["sec"]),\
                    int(goal_json["t"]["nanosec"]))
                self.goal_path.append(goal)
            self.current_task_id = path_req_json["task_id"]

            if self.paused:
                self.paused = False
            self.request_error = False
            return True

        return False

    def receive_destination_request(self, destination_json):
        return

    def receive_mode_request(self, mode_json):
        return

    def receive_perform_action(self, perform_action_json):
        return

    def send_robot_state(self):
        robot_state_json = {}
        robot_state_json["fleet_name"] = self.config.fleet_name
        robot_state_json["name"] = self.config.robot_name
        robot_state_json["model"] = self.config.robot_model
        robot_state_json["task_id"] = self.current_task_id
        robot_state_json["battery_percent"] = 100*self.current_battery_state.percentage
        robot_state_json["robot_mode"] = self.get_robot_mode()
        robot_state_json["location"] = self.transform_to_location_json(self.current_robot_transform)
        path = []
        for i in range(len(self.goal_path)):
            location_json={}
            time_stamp_json = {}
            time_stamp_json["sec"] =\
                int(self.goal_path[i].goal.target_pose.header.stamp.secs)
            time_stamp_json["nanosec"] =\
                int(self.goal_path[i].goal.target_pose.header.stamp.nsecs)
            location_json["t"] = time_stamp_json
            location_json["x"] =\
                float(self.goal_path[i].goal.target_pose.pose.position.x)
            location_json["y"] =\
                float(self.goal_path[i].goal.target_pose.pose.position.y)
            quaternion = [0.0, 0.0, 0.0, 0.0]
            quaternion[0] = self.goal_path[i].goal.target_pose.pose.orientation.x
            quaternion[1] = self.goal_path[i].goal.target_pose.pose.orientation.y
            quaternion[2] = self.goal_path[i].goal.target_pose.pose.orientation.z
            quaternion[3] = self.goal_path[i].goal.target_pose.pose.orientation.w
            location_json["yaw"] = float(transformations.euler_from_quaternion(\
                quaternion)[2])
            location_json["level_name"] = self.config.level_name
            path.append(location_json)
        robot_state_json["path"] = path
        self.client.send_robot_state(robot_state_json)



    def location_to_move_base_goal(self, location_json) -> MoveBaseGoal:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.config.map_frame
        goal.target_pose.header.stamp.secs = location_json["t"]["sec"]
        goal.target_pose.header.stamp.nsecs = location_json["t"]["nanosec"]
        goal.target_pose.pose.position.x = location_json["x"]
        goal.target_pose.pose.position.y = location_json["y"]
        goal.target_pose.pose.position.z = 0.0
        # quarternion
        quaternion =transformations.quaternion_from_euler(\
            0.0, 0.0, float(location_json["yaw"]))
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        return goal

    def transform_to_location_json(self,transform):
        location_json = {}
        time_stamp_json = {}
        time_stamp_json["sec"] = transform.header.stamp.secs
        time_stamp_json["nanosec"] = transform.header.stamp.nsecs
        location_json["t"] = time_stamp_json
        location_json["x"] = transform.transform.translation.x
        location_json["y"] = transform.transform.translation.y
        quat = [transform.transform.rotation.x,transform.transform.rotation.y\
            ,transform.transform.rotation.z, transform.transform.rotation.w]
        location_json["yaw"] =\
            transformations.euler_from_quaternion(quat)[2]
        location_json["level_name"] = self.config.level_name
        return location_json


    def loop(self):
        self.get_robot_transform()
        self.send_robot_state()
        if (len(self.goal_path) != 0):
            if (not self.goal_path[0].sent):
                rospy.loginfo("Sending new goal!")
                try:
                    self.move_base_client.send_goal(self.goal_path[0].goal)
                    self.goal_path[0].sent = True
                    return
                except Exception as e:
                    rospy.loginfo("failed to send goal")
                    rospy.loginfo(f"{e}")
            current_goal_state = self.move_base_client.get_state()
            if current_goal_state == GoalStatus.SUCCEEDED:
                rospy.loginfo("current goals state: SUCCEEDED")
                if rospy.Time.now() >= self.goal_path[0].goal_end_time:
                    self.goal_path.pop(0)
                else:
                    wait_time_remaining = self.goal_path[0].goal_end_time - rospy.Time.now()
                    rospy.loginfo(f"we reached our goalearly! Waiting for \
                        {wait_time_remaining.to_sec()} more seconds")
                return
            elif current_goal_state == GoalStatus.ACTIVE:
                return
            elif current_goal_state == GoalStatus.ABORTED:
                self.goal_path[0].aborted_cout += 1

                if (self.goal_path[0].aborted_cout < self.max_tries):
                    rospy.loginfo(f"robot's naviation stack has been aborted the \
                        current goal {self.goal_path[0].aborted_count} times. Check if there is \
                            nothing in the way of the robot. Trying again .....")
                    self.move_base_client.cancel_goal()
                    self.goal_path[0].sent = False
                    return

                # aborted more than 5 times
                else:
                    rospy.loginfo(f"robot's naviation stack has already been aborted the \
                        current goal {self.goal_path[0].aborted_count} times. I GIVE UP! \
                            please make sure nothing in the way of the robot.")
                    self.move_base_client.cancel_goal()
                    self.goal_path.clear()
                    return
        else:
            # TODO: find out how to print goal state string in python
            # rospy.loginfo(f"Undesireable goal state: {current_goal_state.to}")
            self.move_base_client.cancel_goal()
            self.goal_path.clear()
            return





