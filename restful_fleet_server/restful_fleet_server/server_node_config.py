import rclpy

class ServerNodeConfig():
    def __init__(self):
        self.node_name = "restful_fleet_server"
        self.fleet_name = "Unodopo"
        self.timer_period = 0.5
        self.fleet_state_pub_rate = 10
        self.sub_rate = 10

        # scale from robot to rmf
        self.map_scale = 1.00
        # measured with robot reference frame
        # distance of rmf origin from robot origin
        self.map_translate_x = -34.172
        self.map_translate_y = 8.74
        self.map_rotation = 0.0

        # RMF
        self.fleet_state_topic = "/fleet_states"
        self.action_execution_notice_topic = "/action_execution_notice"
        self.path_request_topic = "/robot_path_requests"
        self.perform_action_topic = f"/{self.fleet_name}/perform_action"
        self.destination_request_topic= "/robot_destination_requests"
        self.mode_request_topic = "/robot_mode_requests"

        # FF
        self.robot_state_route = "/robot-state"
        self.mode_request_route = "/mode-request"
        self.path_request_route = "/path-request"
        self.perform_action_request_route = "/perform-action"


    def print_config(self):
        print(f'Node Name: {self.node_name}')
        print(f'Fleet Name: {self.fleet_name}')
        print(f'Timer Period: {self.timer_period}')
        print(f'Fleet State Pub Rate: {self.fleet_state_pub_rate}')
        print(f'Subscriber Rate: {self.sub_rate}')
        print(f'Map Scale: {self.map_scale}')
        print(f'Map Translate X: {self.map_translate_x}')
        print(f'Map Translate Y: {self.map_translate_y}')
        print(f'Map Rotation: {self.map_rotation}')
        print(f'Fleet State Topic: {self.fleet_state_topic}')
        print(f'Action Execution Notice topic: {self.action_execution_notice_topic}')
        print(f'Path Request Topic: {self.path_request_topic}')
        print(f'Perform Action: {self.perform_action_topic}')
        print(f'Destination Request: {self.destination_request_topic}')
        print(f'Mode Request Topic: {self.mode_request_topic}')
        print(f'Robot State Route: {self.robot_state_route}')
        print(f'Mode Request Route: {self.mode_request_route}')
        print(f'Path Request Route: {self.path_request_route}')
        print(f'Perform Action Request Route: {self.perform_action_request_route}')
