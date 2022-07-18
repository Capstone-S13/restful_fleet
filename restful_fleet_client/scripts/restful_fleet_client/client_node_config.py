
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
        return
