# ar_test/src/compute_cubic_coeffs.py

import rclpy
from rclpy.node import Node
import numpy as np
from ar_interface.srv import ComputeCubicTraj

class CubicCoeffsCalculator(Node):
    def __init__(self):
        super().__init__('compute_cubic_coeffs')
        
        
        self.srv = self.create_service(
            ComputeCubicTraj, 
            '/compute_cubic_traj', 
            self.compute_coeffs
        )
        
        self.get_logger().info("Coefficient Calculator Service Started")

    def compute_coeffs(self, request, response):
        params = request.params
        
        
        A = np.array([
            [1, params.t0, params.t0**2, params.t0**3],
            [0, 1, 2*params.t0, 3*params.t0**2],
            [1, params.tf, params.tf**2, params.tf**3],
            [0, 1, 2*params.tf, 3*params.tf**2]
        ])
        
        
        b = np.array([
            params.p0,
            params.v0,
            params.pf,
            params.vf
        ])
        
        try:
            
            x = np.linalg.solve(A, b)
            response.a0 = x[0]
            response.a1 = x[1]
            response.a2 = x[2]
            response.a3 = x[3]
            
            self.get_logger().info(f"Computed coefficients: {x}")
            return response
            
        except np.linalg.LinAlgError:
            self.get_logger().error("Matrix is singular!")
            return response

def main(args=None):
    rclpy.init()
    node = CubicCoeffsCalculator()
    rclpy.spin(node)
    rclpy.shutdown()

#if __name__ == '__main__':
#    main()
