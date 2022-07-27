# RESTFUL FLEET MANAGER

## About
`restful fleet` is a fleet manager package for [`RMF`](https://github.com/open-rmf/rmf). The implementation of this package is largely based on [`free fleet`](https://github.com/open-rmf/free_fleet). `free fleet` uses DDS as its middleware to communicate between the server and client. However, it can be cumbersome to allow DDS packets to be transferred across the Wide Area Network. Threfore the middleware implementation of `restful fleet` is using restful http requests.

## Installation
```
cd <ws_path>/src
git clone https://github.com/Capstone-S13/restful_fleet.git
cd <ws_path>
colcon build --symlink-install
```

## Running Restful Fleet
Before running the restful fleet client, be sure to run your robot's navigation stack with its movebase server.
```
cd <ws_path>
source install/setup.bash
rosrun restful_fleet_client main.py
```


## For Developers

There are four main classes in the restful fleet package.
* Server
* ServerNode
* Client
* ClientNode

### Server and Client
The middleware specific components and methods are encapsulated within the `Server` and `Client` classes. In the case of `restful fleet` this refers to the flask app and the methods for making restful requests.

### ServerNode
The `ServerNode` contains the ROS publishers, subscribers to publish and subscribe relevant information to and from the fleet adapter. The most basic function of the `ServerNode` is to publish it's fleet's `robot_state` into the`fleet states` topic and relay the `path request` and `mode request` to the robot client.


### ServerNode and ClientNode
Likewise, the `ServerNode` contains ROS specific classes and callback functions. The `ClientNode` contains the publishers, subscribers, action client, action servers required determin the robot's state as well as to command the robot to carry out necessary tasks.


## Future Improvements
* Multi client discovery.
* ROS2 version of restful fleet client
* More efficient method of calling map service