
class ServerNodeConfig():
    def __init__(self):
        self.node_name = "restful_fleet_server"
        self.fleet_name = "Unodopo"
        self.timer_period = 0.5
        self.fleet_state_pub_rate = 10
        self.sub_rate = 10

        # scale from robot to rmf
        self.map_scale = 1.05
        # measured with robot reference frame
        # distance of rmf origin from robot origin
        self.map_translate_x = -32.306
        self.map_translate_y = 8.186
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

    def _get_params(self):
        return

    def print_settings(self):
        return
