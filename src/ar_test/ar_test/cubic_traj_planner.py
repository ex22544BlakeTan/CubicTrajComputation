
import rclpy
from rclpy.node import Node
from ar_interface.msg import CubicTrajParams, CubicTrajCoeffs
from ar_interface.srv import ComputeCubicTraj

class CubicTrajPlanner(Node):
    def __init__(self):
        super().__init__('cubic_traj_planner')
        self.publisher = self.create_publisher(CubicTrajCoeffs, 'cubic_traj_coeffs', 10)
        self.subscription = self.create_subscription(CubicTrajParams, '/params', self.trajectory_callback, 10)
        self.client = self.create_client(ComputeCubicTraj, 'compute_cubic_traj')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for compute_cubic_traj service...')

        self.get_logger().info("Cubic Trajectory Planner Node Started.")

    def trajectory_callback(self, msg):
        """Handles new trajectory requests and calls the compute_cubic_traj service"""
        self.get_logger().info(f'Received Params: {msg}')

        request = ComputeCubicTraj.Request()
        request.p0, request.pf, request.v0, request.vf, request.t0, request.tf = msg.p0, msg.pf, msg.v0, msg.vf, msg.t0, msg.tf

        future = self.client.call_async(request)
        future.add_done_callback(self.trajectory_response)

    def trajectory_response(self, future):
        """Handles the response from the service and publishes computed coefficients"""
        try:
            response = future.result()
            msg = CubicTrajCoeffs(a0=response.a0, a1=response.a1, a2=response.a2, a3=response.a3, t0=response.t0, tf=response.tf)
            self.publisher.publish(msg)
            self.get_logger().info(f'Published Coefficients: {msg}')
        except Exception as e:
            self.get_logger().error(f'Failed to compute cubic trajectory: {e}')

def main():
    rclpy.init()
    node = CubicTrajPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

