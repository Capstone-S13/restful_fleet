from websockets import server
from restful_fleet_server.server_config import ServerConfig
from restful_fleet_server.server import Server
from restful_fleet_server.server_node_config import ServerNodeConfig
from restful_fleet_server.server_node import ServerNode
import rclpy

def main():
    rclpy.init()
    server_config = ServerConfig()
    server_node_config = ServerNodeConfig()
    server = Server(server_config, server_node_config)
    server.start()


if __name__ == '__main__':
    main()

