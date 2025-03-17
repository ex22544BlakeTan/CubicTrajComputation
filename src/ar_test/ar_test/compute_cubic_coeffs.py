
import rclpy
from rclpy.node import Node
from ar_interface.srv import ComputeCubicTraj
import numpy as np

class ComputeCubicCoeffs(Node):
    def __init__(self):
        super().__init__('compute_cubic_coeffs')
        self.srv = self.create_service(ComputeCubicTraj, 'compute_cubic_traj', self.compute_coeffs)
        self.get_logger().info("Compute Cubic Coefficients Node Started.")

    def compute_coeffs(self, request, response):
        """Computes cubic trajectory coefficients given input parameters"""
        self.get_logger().info(f"Computing coefficients for: {request}")

        if request.t0 == request.tf:
            self.get_logger().error("Error: t0 and tf cannot be equal!")
            return response

        # Solve for cubic coefficients
        A = np.array([[1, request.t0, request.t0**2, request.t0**3],
                      [0, 1, 2*request.t0, 3*request.t0**2],
                      [1, request.tf, request.tf**2, request.tf**3],
                      [0, 1, 2*request.tf, 3*request.tf**2]])
        B = np.array([request.p0, request.v0, request.pf, request.vf])

        try:
            solution = np.linalg.solve(A, B)

            response.a0, response.a1, response.a2, response.a3 = map(float, solution)
            #response.t0, response.tf = float(request.t0), float(request.tf)
            #t0, tf = float(request.t0), float(request.tf)
            response.t0 = float(request.t0)
            response.tf = float(request.tf)
            self.get_logger().info(f"Computed Coefficients: a0={response.a0}, a1={response.a1}, a2={response.a2}, a3={response.a3}")
        except np.linalg.LinAlgError:
            self.get_logger().error("Error: Singular matrix! Cannot compute trajectory.")
            response.a0, response.a1, response.a2, response.a3 = 0.0, 0.0, 0.0, 0.0

        return response

def main():
    rclpy.init()
    node = ComputeCubicCoeffs()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

