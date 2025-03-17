import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # Using Float32
from ar_interface.msg import CubicTrajCoeffs
import time

class PlotCubicTraj(Node):
    def __init__(self):
        super().__init__('plot_cubic_traj')
        
        # Topic subscribtion /cubic_traj_coeffs
        self.subscription = self.create_subscription(
            CubicTrajCoeffs,
            '/cubic_traj_coeffs',
            self.coeffs_callback,
            10
        )
        
        # publish position, velocity and acceleration trajectory
        self.pos_publisher = self.create_publisher(Float32, '/position_trajectory', 10)
        self.vel_publisher = self.create_publisher(Float32, '/velocity_trajectory', 10)
        self.acc_publisher = self.create_publisher(Float32, '/acceleration_trajectory', 10)
        
        # Initiate the parameters
        self.current_coeffs = None
        self.t0 = 0.0
        self.tf = 0.0
        
    def coeffs_callback(self, msg):
        """Callback function when new traj params recieved"""
        self.get_logger().info("Recieved new traj params！")
        
        ## check tf-t0 is too short or not
        if msg.tf - msg.t0 < 3.0:
            self.get_logger().warn(" tf-t0 too short! Forced to 3")
            msg.tf = msg.t0 + 3.0
        
        #Saved coefficients and 
        self.current_coeffs = {
            'a0': msg.a0,
            'a1': msg.a1,
            'a2': msg.a2,
            'a3': msg.a3
        }
        self.t0 = msg.t0
        self.tf = msg.tf
        
        # calculate and publish the traj
        self.publish_trajectories()
    
    def publish_trajectories(self):
        """Calculation of position, velocity and acceleration"""
        if not self.current_coeffs:
            return
        
        # calculate the traj
        time_step = 0.02
        current_time = self.t0
        
        while current_time <= self.tf:
            # calculate time offset
            t_offset = current_time - self.t0
            
            # calculate position, velocity and acceleration
            pos = (
                self.current_coeffs['a0'] +
                self.current_coeffs['a1'] * t_offset +
                self.current_coeffs['a2'] * t_offset**2 +
                self.current_coeffs['a3'] * t_offset**3
            )
            vel = (
                self.current_coeffs['a1'] +
                2 * self.current_coeffs['a2'] * t_offset +
                3 * self.current_coeffs['a3'] * t_offset**2
            )
            acc = (
                2 * self.current_coeffs['a2'] +
                6 * self.current_coeffs['a3'] * t_offset
            )
            
            # pub the msg
            self.publish_value(self.pos_publisher, pos)
            self.publish_value(self.vel_publisher, vel)
            self.publish_value(self.acc_publisher, acc)
            
            # add delay
            time.sleep(0.02) # 10ms delay for better visualization
            
            # update the time
            current_time += time_step
        
        self.get_logger().info("Finished plotting，waiting for next params...")
    
    def publish_value(self, publisher, value):
        
        msg = Float32()  
        msg.data = value
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PlotCubicTraj()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
