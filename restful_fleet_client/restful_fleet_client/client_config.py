class ClientConfig():
    def __init__(self):
        self.server_ip = "0.0.0.0"
        self.server_port = 9000
        # receiving
        self.mode_request_route = "/battery_state"
        self.path_request_route = "/path-request"
        self.perform_action_route = "/perform-action"
        self.battery_state_route = "/battery-state"
        self.robot_state_route = "/robot-state"
        self.perform_action_route = "/perfrom-action"
        self.end_action_route = "/end_action"