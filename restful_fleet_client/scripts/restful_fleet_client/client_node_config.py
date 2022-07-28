
class ClientNodeConfig():
    def __init__(self):
        self.fleet_name = "Unodopo"
        self.robot_name = "Unodopo1"
        self.robot_model = "robot_model"
        self.level_name = "L1"
        self.battery_topic = "/battery_state"
        self.map_frame = "map"
        self.robot_frame = "base_link"
        self.move_base_server_name = "move_base"
        self.docking_trigger_name =""
        self.max_dist_to_first_waypoint = 10.0

    def print_config(self):
        print(f'Fleet Name: {self.fleet_name}')
        print(f'Robot Name: {self.robot_name}')
        print(f'Robot Model: {self.robot_model}')
        print(f'Level Name: {self.level_name}')
        print(f'Battery Topic: {self.battery_topic}')
        print(f'Map Frame: {self.map_frame}')
        print(f'Robot Frame: {self.robot_frame}')
        print(f'Move Base Server Name: {self.move_base_server_name}')
        print(f'Docking Trigger Name: {self.docking_trigger_name}')
        print(f'Maximum Distance to First Waypoint: {self.max_dist_to_first_waypoint}')
        return
