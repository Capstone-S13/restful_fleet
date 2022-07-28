class ClientConfig():
    def __init__(self):
        self.server_ip = "0.0.0.0"
        self.server_port = 9000
        self.client_ip = "0.0.0.0"
        self.client_port = 9001
        # routes
        self.mode_request_route = "/mode-state"
        self.path_request_route = "/path-request"
        self.perform_action_route = "/perform-action"
        self.robot_state_route = "/robot-state"
        self.perform_action_route = "/perfrom-action"
        self.end_action_route = "/end-action"

    def print_config(self):
        print(f'Server IP: {self.server_ip}')
        print(f'Server Port: {self.server_port}')
        print(f'Client IP: {self.client_ip}')
        print(f'Client Port: {self.client_port}')

        # routes
        print(f'Mode Request Route: {self.mode_request_route}')
        print(f'Path Request Route: {self.path_request_route}')
        print(f'Perform Action Route: {self.perform_action_route}')
        print(f'Robot State Route: {self.robot_state_route}')
        print(f'Perform Action Route: {self.perform_action_route}')
        print(f'End Action Route: {self.end_action_route}')