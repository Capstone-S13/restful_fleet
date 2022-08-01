import base64
import os
import time
import yaml
import rospy

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from numpy import math
from threading import Semaphore, Thread

from sensor_msgs.msg import BatteryState
from tf2_ros import TransformListener, Buffer
from tf import transformations
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib import SimpleActionClient, GoalStatus
from nav_msgs.srv import LoadMapRequest, LoadMap

from restful_fleet_client.utilities import is_transform_close, RobotMode
from restful_fleet_client.client_node_config import ClientNodeConfig

from vs_interfaces.msg import VisionServoActionGoal, VisionServoActionAction
from payload_interfaces.msg import PayLoadActionGoal, PayLoadActionAction, PayLoadMode

from hub_interfaces.msg import HubOperation
from hub_interfaces.srv import HubService
from hub_interfaces.action import HubAction

class Goal():
    def __init__(self):
        self.level_name = ""
        self.goal = None
        self.sent = False
        self.aborted_count = 0
        self.goal_end_time = None


class ClientNode():
    def __init__(self,config: ClientNodeConfig, client):
        self.config = config
        self.client = client
        self.set_config()
        self.config.print_config()

        self.battery_sub = rospy.Subscriber(self.config.battery_topic,\
            BatteryState, self.battery_state_cb,queue_size=10)
        self.current_battery_state = BatteryState()
        self.previous_robot_transform = TransformStamped()
        self.current_robot_transform = TransformStamped()
        self.goal_path = []
        self.move_base_client = \
            SimpleActionClient(self.config.move_base_server_name,MoveBaseAction)
        self.tf2_buffer = Buffer()
        self.tf2_listener = TransformListener(self.tf2_buffer)
        self.max_tries = 5
        self.emergency = False
        self.paused = False
        self.request_error= False
        self.current_task_id = ""
        self.task_id_semaphore = Semaphore()
        self.goal_path_semaphore = Semaphore()
        self.loop_rate = rospy.Rate(0.5)
        self.loop_thread = Thread(target=self.loop_self, args=())
        self.spin_thread = Thread(target=self.spin_self, args=())

        # for docking
        self.docking_action_client = SimpleActionClient('vs_action', VisionServoActionAction)

        # for slider
        self.payload_action_client = SimpleActionClient('payload_action', PayLoadActionAction)

        # for map change feature
        self.map_service_client = rospy.ServiceProxy('change_map', LoadMap)
        self.initial_pose_pub = rospy.Publisher('initialpose',
                PoseWithCovarianceStamped, queue_size=10)

        # ROS 2
        self.ros2_node = rclpy.create_node('restful_fleet_client')

        # for hub actions and services
        self.hub_service_client = self.ros2_node.create_client(HubService, 'hub_service')
        self.hub_action_client = ActionClient(self.ros2_node, HubAction, 'hub_action')


    def init(self):
        # in case we need to set up some stuff
        return

    def set_config(self):
        # client node config
        _ns="~"
        try:
            _ns = "~"
            self.config.fleet_name = rospy.get_param(f'{_ns}/fleet_name')
            self.config.robot_name = rospy.get_param(f'{_ns}/robot_name')
            self.config.robot_model = rospy.get_param(f'{_ns}/robot_model')
            self.config.level_name = rospy.get_param(f'{_ns}/level_name')
            self.config.battery_topic = rospy.get_param(f'{_ns}/battery_topic')
            self.config.map_frame = rospy.get_param(f'{_ns}/map_frame')
            self.config.robot_frame = rospy.get_param(f'{_ns}/robot_frame')
            self.config.move_base_server_name = rospy.get_param(f'{_ns}/move_base_server_name')
            self.config.docking_trigger_name = rospy.get_param(f'{_ns}/docking_trigger_name')
            self.config.max_dist_to_first_waypoint = rospy.get_param(f'{_ns}/max_dist_to_first_waypoint')

            # client config
            self.client.client_config.server_ip =  rospy.get_param(f'{_ns}/server_ip')
            self.client.client_config.server_port =  rospy.get_param(f'{_ns}/server_port')
            self.client.client_config.client_ip =  rospy.get_param(f'{_ns}/client_ip')
            self.client.client_config.client_port =  rospy.get_param(f'{_ns}/client_port')
            self.client.client_config.mode_request_route =  rospy.get_param(f'{_ns}/mode_request_route')
            self.client.client_config.path_request_route =  rospy.get_param(f'{_ns}/path_request_route')
            self.client.client_config.perform_action_route =  rospy.get_param(f'{_ns}/perform_action_route')
            self.client.client_config.robot_state_route =  rospy.get_param(f'{_ns}/robot_state_route')
            self.client.client_config.end_action_route =  rospy.get_param(f'{_ns}/end_action_route')

        except rospy.ROSException as e:
            print(f'Exception: {e}')

    def loop_self(self):
        while not rospy.is_shutdown():
            self.loop()
            self.loop_rate.sleep()
        rospy.loginfo("exiting loop thread")

    def spin_self(self):
        rospy.spin()
        rospy.loginfo("exiting spin thread")

    def spin_self_2(self):
        while not rclpy.shutdown():
            rclpy.spin_once()

    def start_threads(self):
        self.loop_thread.start()
        self.spin_thread.start()

    def battery_state_cb(self, msg):
        self.current_battery_state = msg

    def is_valid_transform(self, request_fleet_name, request_robot_name,\
        request_task_id):

        return

    def is_valid_request(self, request_fleet_name, request_robot_name,\
        request_task_id) -> bool:
        self.task_id_semaphore.acquire()
        if (self.current_task_id == request_task_id\
            or self.config.robot_name != request_robot_name\
            or self.config.fleet_name != request_fleet_name):
            rospy.loginfo("not a valid request")
            self.task_id_semaphore.release()
            return False
        self.current_task_id = request_task_id
        self.task_id_semaphore.release()
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
            return RobotMode.MODE_ADAPTER_ERROR.value
        if self.emergency:
            return RobotMode.MODE_EMERGENCY.value

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
            # we wanna move the setting of the task id to be
            # part of the critical section in the is valid request section
            self.task_id_semaphore.acquire()
            self.current_task_id = path_req_json["task_id"]
            self.task_id_semaphore.release()
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
        category = perform_action_json['category']
        description = perform_action_json['description']
        action = getattr(self, category)
        try:
            action(description)
        except Exception as e:
            rospy.logwarn(f"Peform Action exception: {e}")

    # collecting from hub
    def hub_collect(self, description):
        rospy.loginfo('executing hub collection')

        # check services
        if not self.hub_action_client.server_is_ready():
            rospy.logwarn('hub action server is not up! Aborting action')
            raise Exception('Hub service action is not up!')

        if not self.hub_service_client.service_is_ready():
            rospy.logwarn('hub service server is not up! Aborting action')
            raise Exception('Hub service is not up!')

        # call hub service
        description['operation'] = HubOperation.OPERATION_COLLECT
        request = self.get_hub_request(description)
        self.hub_future = self.hub_service_client.call_async(request)


        # check if order is valid
        rospy.loginfo('Chacking order validity')
        rclpy.spin_until_future_complete(self.ros2_node, self.hub_future)
        result = self.hub_future.result()
        rospy.loginfo(f'dock available: {result.available_dock},\
                valid order: {result.is_valid}')
        # if not result.available_dock or not result.is_valid:
        #     rospy.logwarn('Invalid hub request, aborting hub operation')
        #     return

        # dock to april tag
        rospy.loginfo('Calling docking action')
        dock_goal = self.get_dock_action_goal(HubOperation.OPERATION_COLLECT)
        if dock_goal is not None:
            self.docking_action_client.send_goal_and_wait(dock_goal)
        else:
            rospy.logwarn('Aborting docking action: invalid dock goal')
            return

        # call hub action
        rospy.loginfo('Calling hub action')
        hub_action_goal = self.get_hub_action_goal(description)
        self.hub_action_done = False
        self.hub_future = self.hub_action_client.send_goal_async(hub_action_goal,
                feedback_callback=self.hub_action_feedback_cb)
        self.hub_future.add_done_callback(self.hub_goal_response_cb)
        # rclpy.spin_until_future_complete(self.ros2_node, self.hub_future)
        self.wait_hub_action()
        return

    # depositing to hub
    def hub_deposit(self, description):
        rospy.loginfo('executing hub deposit')

        # check services
        if not self.hub_action_client.server_is_ready():
            rospy.logwarn('hub action server is not up! Aborting action')
            raise Exception('Hub service action is not up!')

        if not self.hub_service_client.service_is_ready():
            rospy.logwarn('hub service server is not up! Aborting action')
            raise Exception('Hub service is not up!')

        # call hub service
        rospy.loginfo('Calling Hub service')
        description['operation'] = HubOperation.OPERATION_DEPOSIT
        request = self.get_hub_request(description)
        self.hub_future = self.hub_service_client.call_async(request)
        rclpy.spin_until_future_complete(self.ros2_node, self.hub_future)

        # check if order is valid
        rospy.loginfo('Checking order validity')
        result = self.hub_future.result()
        rospy.loginfo(f'dock available: {result.available_dock},\
                valid order: {result.is_valid}')
        # if not result.available_dock or not result.is_valid:
        #     rospy.logwarn('Invalid hub request, aborting hub operation')
        #     return

        # dock to april tag
        rospy.loginfo('Calling docking action')
        dock_goal = self.get_dock_action_goal(HubOperation.OPERATION_DEPOSIT)
        if dock_goal is not None:
            self.docking_action_client.send_goal(dock_goal)
            rospy.loginfo("sent goal and now waiting for result")
            self.docking_action_client.wait_for_result()
            dock_result = self.docking_action_client.get_result()
            rospy.loginfo(f"dock result {dock_result}")
        else:
            rospy.logwarn('Aborting docking action: invalid dock goal')
            return

        # call hub action
        rospy.loginfo('Calling hub action')
        hub_action_goal = self.get_hub_action_goal(description)
        # self.hub_action_result = self.hub_action_client.send_goal(hub_action_goal)
        self.hub_action_done = False
        self.hub_future = self.hub_action_client.send_goal_async(hub_action_goal,
                feedback_callback=self.hub_action_feedback_cb)
        self.hub_future.add_done_callback(self.hub_goal_response_cb)
        # rclpy.spin_until_future_complete(self.ros2_node, self.hub_future)
        self.wait_hub_action()

        # call slider
        rospy.loginfo('Calling slider action')
        payload_action_goal = self.get_payload_action_goal(PayLoadMode.PUSHER_OUT)
        self.payload_action_client.send_goal(payload_action_goal)
        self.payload_action_client.wait_for_result()
        self.payload_action_client.get_result()
        time.sleep(3)
        payload_action_goal = self.get_payload_action_goal(PayLoadMode.PUSHER_IN)
        self.payload_action_client.send_goal(payload_action_goal)
        self.payload_action_client.wait_for_result()
        self.payload_action_client.get_result()

        # call hub action
        rospy.loginfo('Calling hub action')
        # self.hub_action_result = self.hub_action_client.send_goal(hub_action_goal)
        self.hub_action_done = False
        self.hub_future = self.hub_action_client.send_goal_async(hub_action_goal,
                feedback_callback=self.hub_action_feedback_cb)
        self.hub_future.add_done_callback(self.hub_goal_response_cb)
        # rclpy.spin_until_future_complete(self.ros2_node, self.hub_future)
        self.wait_hub_action()



        rospy.loginfo('hub action complete')
        return

    def hub_goal_response_cb(self, future):
        goal_handle= future.result()
        if not goal_handle.accepted:
            rospy.loginfo('hub action goal rejected')
            return
        rospy.loginfo('hub action goal accepted')

        self.__get_hub_result_future = goal_handle.get_result_async()
        self.__get_hub_result_future.add_done_callback(self.hub_action_result_cb)

    def hub_action_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.msg
        rospy.loginfo(f'hub action feedback: {feedback}')

    def hub_action_result_cb(self, future):
        result = future.result().result
        rospy.loginfo(f"got hub action result {result}")
        self.hub_action_done = True

    def wait_hub_action(self):
        while not self.hub_action_done:
            rospy.loginfo('waiting for hub action')
            rclpy.spin_once(self.ros2_node)

    def get_hub_request(self, description) -> HubService.Request:
        request = HubService.Request()
        request.operation.operation = description['operation']
        request.company_name = description['company_name']
        request.order_id = description['id']
        return request

    def get_hub_action_goal(self, description) -> HubAction.Goal:
        goal = HubAction.Goal()
        goal.company_name = description['company_name']
        goal.order_id = description['id']
        goal.operation.operation = description['operation']
        return goal


    def get_dock_action_goal(self, operation) -> VisionServoActionGoal:
        goal = None
        if operation == HubOperation.OPERATION_DEPOSIT:
            goal = VisionServoActionGoal(tag_id=self.config.deposit_tag_id)
        elif operation == HubOperation.OPERATION_COLLECT:
            goal = VisionServoActionGoal(tag_id=self.config.collect_tag_id)
        else:
            rospy.loginfo('Hub operation not valid for docking')
            return None

        if goal == None:
            raise Exception("unable to create docking goal")
        return goal

    def get_payload_action_goal(self, operation) -> PayLoadActionGoal:
        goal = PayLoadActionGoal()
        goal.pusher_action.operation = operation
        return goal

    def send_hub_request(self, request):
        self.hub_service_client.call(request)


    def eject_robot(self,description):
        rospy.loginfo("ejecting robot")
        initial_pose  = description['initial_pose']
        map_json = description['map']
        new_host_json = description['new_host']

        self.end_action()
        self.change_host(new_host_json)
        self.change_map(map_json)
        self.initialise_pose(initial_pose)
        return

    def change_host(self, new_host_json):
        # TODO: there should be a way to implement a way for clients to be
        # discovered and know their IP and port.
        self.client.client_config.server_ip = new_host_json['ip']
        self.client.client_config.server_port = new_host_json['port']
        return


    # changes the navigation map
    def change_map(self, map_json) -> bool:
        # NOTE: this implementation saves the map data into yaml and pgm files.
        # This is an convenient for us to just call the 'change_map' service,
        # but it is an inefficient method, since it involves saving the map and
        # invoking a service call which will read the map files again.
        # A better method would be invoking the map change directly with the
        # byte string and meta data received in map_json.
        pgm_b64 = map_json['pgm']
        pgm_data = base64.b64decode(pgm_b64)
        del map_json['pgm']
        path = self.config.map_directory
        pgm_filepath = os.path.join(path, map_json['image'])
        # TODO: should not assume the file is a pgm file
        yaml_filepath = os.path.join(path, f"{map_json['image'].split('.')[0]}")

        try:
            # create yaml file
            f = open(yaml_filepath, 'w')
            f.write(yaml.dump(map_json))
            f.close()

            # create image file
            f = open(pgm_filepath, 'wb')
            f.write(pgm_data)
            f.close()

        except Exception as e:
            print(f"exception: {e}")
            return False

        map_request = LoadMapRequest()
        map_request.map_url = yaml_filepath
        self.map_service_client(map_request)
        return True

    def initialise_pose(self, initial_pose):
        msg  = PoseWithCovarianceStamped()
        msg.header.frame_id = self.config.map_frame
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = float(initial_pose[0])
        msg.pose.pose.position.y = float(initial_pose[1])
        msg.pose.pose.position.z = float(initial_pose[2])

        # convert euler to quaternion
        q = transformations.quaternion_from_euler(
            initial_pose[3], initial_pose[4], initial_pose[5])

        msg.pose.pose.orientation.x = float(q[0])
        msg.pose.pose.orientation.y = float(q[1])
        msg.pose.pose.orientation.z = float(q[2])
        msg.pose.pose.orientation.w = float(q[3])

        self.initial_pose_pub.publish(msg)

    def end_action(self):
        # end action
        self.paused = True
        self.task_id_semaphore.acquire()
        self.current_task_id = "end_action"
        self.task_id_semaphore.release()

        self.goal_path_semaphore.acquire()
        self.goal_path.clear()
        self.goal_path_semaphore.release()

        # end_action_json = {}
        # end_action_json['robot'] = {'id': self.config.robot_name}
        # self.client.send_end_action()


    def send_robot_state(self):
        robot_state_json = {}
        robot_state_json["fleet_name"] = self.config.fleet_name
        robot_state_json["name"] = self.config.robot_name
        robot_state_json["model"] = self.config.robot_model
        self.task_id_semaphore.acquire()
        robot_state_json["task_id"] = self.current_task_id
        self.task_id_semaphore.release()
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
        rospy.loginfo(f"the goal path length is {len(self.goal_path)}")
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
        # execute path request if any
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
                    rospy.loginfo(f"we reached our goal early! Waiting for \
                        {wait_time_remaining.to_sec()} more seconds")
                    self.goal_path.pop(0)
                return
            elif current_goal_state == GoalStatus.ACTIVE:
                return
            elif current_goal_state == GoalStatus.ABORTED:
                self.goal_path[0].aborted_count += 1

                if (self.goal_path[0].aborted_count < self.max_tries):
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






