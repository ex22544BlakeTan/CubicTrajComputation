import rclpy
from rclpy.node import Node
from ar_interface.msg import CubicTrajCoeffs
from std_msgs.msg import Float32MultiArray
import numpy as np
import time

class PlotCubicTraj(Node):
    def __init__(self):
        super().__init__('plot_cubic_traj')

        # Subscribe to cubic_traj_coeffs topic
        self.subscription = self.create_subscription(
            CubicTrajCoeffs,
            'cubic_traj_coeffs',
            self.trajectory_callback,
            10
        )

        # Create three publishers for position, velocity, and acceleration
        self.pos_publisher = self.create_publisher(Float32MultiArray, 'position_trajectory', 10)
        self.vel_publisher = self.create_publisher(Float32MultiArray, 'velocity_trajectory', 10)
        self.acc_publisher = self.create_publisher(Float32MultiArray, 'acceleration_trajectory', 10)

        self.get_logger().info("Plot Cubic Trajectory Node Started.")

    def trajectory_callback(self, msg):
        """ Callback function when new coefficients are received. """
        self.get_logger().info(f"Received Coefficients: a0={msg.a0}, a1={msg.a1}, a2={msg.a2}, a3={msg.a3}, t0={msg.t0}, tf={msg.tf}")

        # Generate time samples from t0 to tf
        num_samples = 100  # Resolution of the plot
        t = np.linspace(msg.t0, msg.tf, num_samples)

        # Compute position, velocity, and acceleration using the cubic polynomial
        p = msg.a0 + msg.a1 * t + msg.a2 * t**2 + msg.a3 * t**3
        v = msg.a1 + 2 * msg.a2 * t + 3 * msg.a3 * t**2
        a = 2 * msg.a2 + 6 * msg.a3 * t

        # Publish computed trajectories
        self.publish_trajectory(self.pos_publisher, p, "Position")
        self.publish_trajectory(self.vel_publisher, v, "Velocity")
        self.publish_trajectory(self.acc_publisher, a, "Acceleration")

        # Wait for tf seconds before clearing the plot for next update
        time.sleep(msg.tf - msg.t0)

    def publish_trajectory(self, publisher, data, label):
        """ Publishes the computed trajectory to the respective topic. """
        msg = Float32MultiArray()
        msg.data = data.tolist()
        publisher.publish(msg)
        self.get_logger().info(f"Published {label} Trajectory")

def main(args=None):
    rclpy.init(args=args)
    node = PlotCubicTraj()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
