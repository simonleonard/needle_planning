#!/usr/bin/env python

from needle_planner_msgs.srv import NeedlePlan

import rclpy
from geometry_msgs.msg import Point32
from rclpy.node import Node
from sklearn.linear_model import LinearRegression
import numpy as np  
import pickle
import os
from ament_index_python.packages import get_package_share_directory
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import model_v4
# from planner.scripts import model_v4

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
        
        # the needle insertion depts (mm) where the tissue stiffness changes
        pos_vec = [ 10,  20,  30,  40,
            50,  60, 70, 80, 90]
        
        # the stiffness of the tissue at each layer
        stiffness_vec = [0.073, 0.025, 0.060, 0.04, 0.057,
        0.075, 0.028, 0.072 , 0.056, 0.073]
                
        # the needle insertion depts (mm) where the guide position changes
        guide_pos_ins = [ 10,  20,  30,  40,
            50,  60, 70, 80, 90]
        
        # the position of the guide (mm)
        guide_pos_vec = [ 0,  .2, .4,  1, .5,
            0 , -.5,  -.5, -.5,  -.5]
        
        results = model_v4.run_main(stiffness_vec, pos_vec, guide_pos_vec, guide_pos_ins)
        y_tip, y_disp, y_rxn, single_rand_vec, full_paths, full_rxns = results
        # predc = self.reg.predict(tissue_props.reshape(1, -1))
        predc = y_disp
        offset = request.target.z - predc[round(request.target.x * 5)]
        for i in range(10):
            p = Point32()
            p.x = float(i);
            p.y = request.target.y;
            p.z = float(predc[round(i * 5)] + offset);
            response.plan.polygon.points.append(p)

        return response

def main():
    rclpy.init()
    service = NeedlePlanningService()
    rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
