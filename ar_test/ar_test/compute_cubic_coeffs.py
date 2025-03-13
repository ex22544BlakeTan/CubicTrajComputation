import rclpy
from rclpy.node import Node
from ar_interface.srv import ComputeCubicTraj  # Import the service definition

class ComputeCubicCoeffs(Node):
    def __init__(self):
        super().__init__('compute_cubic_coeffs')

        # Create the service that responds to trajectory computation requests
        self.srv = self.create_service(ComputeCubicTraj, 'compute_cubic_traj', self.compute_trajectory)
        
        self.get_logger().info("Compute Cubic Coefficients Service is now available.")

    def compute_trajectory(self, request, response):
        """ Compute the coefficients for the cubic polynomial trajectory. """
        t0, tf = request.t0, request.tf
        p0, pf = request.p0, request.pf
        v0, vf = request.v0, request.vf

        dt = tf - t0
        if dt == 0:
            self.get_logger().error("Invalid time duration: tf cannot be equal to t0")
            return response

        # Compute cubic trajectory coefficients
        response.a0 = p0
        response.a1 = v0
        response.a2 = (3 * (pf - p0) - (2 * v0 + vf) * dt) / (dt ** 2)
        response.a3 = (2 * (p0 - pf) + (v0 + vf) * dt) / (dt ** 3)

        self.get_logger().info(f"Received: p0={p0}, pf={pf}, v0={v0}, vf={vf}, t0={t0}, tf={tf}")
        self.get_logger().info(f"Computed: a0={response.a0}, a1={response.a1}, a2={response.a2}, a3={response.a3}")

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ComputeCubicCoeffs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Compute Cubic Coefficients service")
    finally:
        node.destroy_node()
        rclpy.shutdown()
