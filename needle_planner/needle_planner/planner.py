from needle_planner_msgs.srv import NeedlePlan

import rclpy
from geometry_msgs.msg import Point32
from rclpy.node import Node

class NeedlePlanningService(Node):
    
    def __init__(self):
        super().__init__('planning_service')
        self.srv = self.create_service(NeedlePlan, 'needle_planner', self.planning_callback)

    def planning_callback(self, request, response):
        self.get_logger().info('Planning Request for target: %f %f %f' % (request.target.x, request.target.y, request.target.z ))
        for i in range(10):
            p = Point32()
            p.x = float(i);
            p.y = 0.0;
            p.z = 0.0;
            response.plan.polygon.points.append(p)

        return response

def main():
    rclpy.init()
    service = NeedlePlanningService()
    rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
