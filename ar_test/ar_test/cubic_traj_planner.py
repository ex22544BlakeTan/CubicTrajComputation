import rclpy
from rclpy.node import Node
from ar_interface.msg import CubicTrajParams, CubicTrajCoeffs
from ar_interface.srv import ComputeCubicTraj

class CubicTrajPlanner(Node):
    def __init__(self):
        super().__init__('cubic_traj_planner')      
        # Create a subscriber to listen to cubic_traj_params messages
        self.subscription = self.create_subscription(CubicTrajParams,'params',self.trajectory_callback,10)
        # Create a publisher to publish cubic_traj_coeffs messages
        self.publisher = self.create_publisher(CubicTrajCoeffs, 'cubic_traj_coeffs', 10)
        # Create a client to call the compute_cubic_traj service
        self.client = self.create_client(ComputeCubicTraj, 'compute_cubic_traj')
        self.get_logger().info("Cubic Trajectory Planner Node Started.")
        
    def trajectory_callback(self, msg):
        """ Callback function when a new cubic_traj_params message is received. """
        self.get_logger().info(f'Received Trajectory Parameters: p0={msg.p0}, pf={msg.pf}, v0={msg.v0}, vf={msg.vf}, t0={msg.t0}, tf={msg.tf}')
        
        # Create a service request with received trajectory parameters
        request = ComputeCubicTraj.Request()
        request.p0 = msg.p0
        request.pf = msg.pf
        request.v0 = msg.v0
        request.vf = msg.vf
        request.t0 = msg.t0
        request.tf = msg.tf
        # Wait for the service to be available
        #while not self.client.wait_for_service(timeout_sec=1.0):
            #self.get_logger().warn('Waiting for compute_cubic_traj service...')

        # Call the service asynchronously
        future = self.client.call_async(request)
        future.add_done_callback(self.trajectory_response)

    def trajectory_response(self, future):
        """ Callback function to handle the response from compute_cubic_traj service. """
        try:
            response = future.result()
            
            # Create a message with the computed cubic trajectory coefficients
            msg = CubicTrajCoeffs()
            msg.a0 = response.a0
            msg.a1 = response.a1
            msg.a2 = response.a2
            msg.a3 = response.a3
            msg.t0 = response.t0
            msg.tf = response.tf
	    
            # Publish the computed coefficients
            self.publisher.publish(msg)

            self.get_logger().info(f'Published Coefficients: a0={msg.a0}, a1={msg.a1}, a2={msg.a2}, a3={msg.a3}, t0={msg.t0}, tf={msg.tf}')

        except Exception as e:
            self.get_logger().error(f'Failed to compute cubic trajectory: {e}')

def main(args=None):
    rclpy.init()
    node = CubicTrajPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
   
#if __name__ == '__main__':
#	main()

