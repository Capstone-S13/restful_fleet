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

    def print_config(self):
        print(f'Server Ip: {self.server_ip}')
        print(f'Server Port: {self.server_port}')
        print(f'Client Ip: {self.client_ip}')
        print(f'Client Port: {self.client_port}')
        print(f'Robot State Route: {self.robot_state_route}')
        print(f'Mode Request Route: {self.mode_request_route}')
        print(f'Path Request Route: {self.path_request_route}')
        print(f'End Action Route: {self.end_action_route}')
        print(f'Perform Action Request Route: {self.perform_action_request_route}')
