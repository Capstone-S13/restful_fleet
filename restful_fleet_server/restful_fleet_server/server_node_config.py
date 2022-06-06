
class ServerNodeConfig():
    def __init__(self):
        self.node_name = "restful_fleet_server"
        self.fleet_name = "unodopo"

        # RMF
        self.fleet_state_topic = "fleet_states"

        # FF
        self.restful_robot_state_route = "/robot-state"
        self.restful_mode_request_route = "/mode-request"
        self.restful_path_request_route = "/path-request"
        self.restful_perform_action_request_route = "/perform-action"

    def _get_params(self):
        return

    def print_settings(self):
        return
