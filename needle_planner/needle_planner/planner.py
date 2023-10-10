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
from sklearn.decomposition import PCA
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression

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
        # Needle should deflect the in the -x direction
        start = [0,0,0]
        end = [request.target.x, request.target.y, request.target.z]
        mid = [ max(start[0] + .2, end[0] + .2), (start[1] + end[1]) * .5, end[2] * .4]
        X = np.array([start, mid, end])

        # Transform 3D points into 2D PCA space
        pca = PCA(n_components=2)
        X_pca = pca.fit_transform(X)

        # Fit a trinomial to the 2D points
        poly = PolynomialFeatures(degree=2)
        X_poly = poly.fit_transform(X_pca[:, 0].reshape(-1, 1))
        reg = LinearRegression().fit(X_poly, X_pca[:, 1])

        # Generate trinomial curve for specific out_x values in PCA-transformed space
        out_x = np.arange(0, end[2]+2, .5).reshape(-1, 1)
        X_out_3D = np.column_stack((np.zeros(out_x.shape), np.zeros(out_x.shape), out_x))
        X_out_pca = pca.transform(X_out_3D)
        x_pca = X_out_pca[:, 0]
        x_pca_poly = poly.transform(x_pca.reshape(-1, 1))
        y_pca_pred = reg.predict(x_pca_poly)

        curve_pca = np.column_stack((x_pca, y_pca_pred))
        curve_3D = pca.inverse_transform(curve_pca)
        for point in curve_3D:
            p = Point32()
            p.x = float(point[0])
            p.y = float(point[1])
            p.z = float(point[2])
            
            response.plan.polygon.points.append(p)

        return response

def main():
    rclpy.init()
    service = NeedlePlanningService()
    rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
