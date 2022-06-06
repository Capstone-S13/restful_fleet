class ServerConfig():
    def __inti__(self):
        self.client_ip = "0.0.0.0"
        self.client_port = 9000
        self.robot_state_route = "/robot-state"
        self.end_action_route = "/end-action"