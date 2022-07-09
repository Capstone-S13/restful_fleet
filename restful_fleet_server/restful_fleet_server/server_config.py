class ServerConfig():
    def __init__(self):
        self.server_ip = "0.0.0.0"
        self.server_port = 9000
        self.client_ip = "0.0.0.0"
        self.client_port = 9001
        self.robot_state_route = "/robot-state"
        self.mode_request_route = "/mode-request"
        self.path_request_route = "/path-request"
        self.end_action_route = "/end-action"
        self.perform_action_request_route = "/perform-action"