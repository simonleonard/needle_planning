# needle_planning

## Build package

`colcon build`

## Run Planning Service
`ros2 run needle_planner planner`

## Run Client from the Command Line
`ros2 service call /needle_planner needle_planner_msgs/srv/NeedlePlan 'target: {x: 0.0,  y: 0.0,  z: 2.0}'`
