from needle_planner_msgs.srv import NeedlePlan

import rclpy
from geometry_msgs.msg import Point32
from rclpy.node import Node
from sklearn.linear_model import LinearRegression
import numpy as np  
import pickle
import os
from ament_index_python.packages import get_package_share_directory


class NeedlePlanningService(Node):
    
    def __init__(self):
        super().__init__('planning_service')
        self.srv = self.create_service(NeedlePlan, 'needle_planner', self.planning_callback)
        package_name = 'needle_planner'
        ws_path = get_package_share_directory(package_name).split('install')[0]
        with open(ws_path + 'needle_planning/needle_planner/needle_planner/linear_regression_model.pkl', 'rb') as file:
            self.reg = pickle.load(file)

    def planning_callback(self, request, response):
        self.get_logger().info('Planning Request for target: %f %f %f' % (request.target.x, request.target.y, request.target.z ))
        
        # The tissue_props defined here correspond to three layers of tissue
        # With stiffnesses of .02 and boundaries at 33 and 66 mm
        tissue_props = np.array([.02, .02, .02, 33, 66])
        predc = self.reg.predict(tissue_props.reshape(1, -1))
        for i in range(10):
            p = Point32()
            p.x = float(i);
            p.y = 0.0;
            p.z = float(predc[0, round(i*5)]);
            response.plan.polygon.points.append(p)

        return response

def main():
    rclpy.init()
    service = NeedlePlanningService()
    rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
