# ar_test/src/plot_traj_node.py

import rclpy
from rclpy.node import Node
#from std_msgs.msg import Float32

class PlotTrajectory(Node):
    def __init__(self):
        super().__init__('plot_cubic_traj')#plot_traj_node
        
        # 创建订阅者
        self.subscription = self.create_subscription(CubicTrajCoeffs,'/trajectory_coeffs', self.points_callback,10)
        
        #self.current_t = None
        #self.get_logger().info("Plot Node Started")

    def points_callback(self, msg):
        # 交替接收时间和位置数据
        if self.current_t is None:
            self.current_t = msg.data
            if self.current_t == 0.0:
                self.get_logger().info("----- New Trajectory Started -----")
        else:
            current_p = msg.data
            self.get_logger().info(
                f"Plotting point: t={self.current_t:.2f}, p={current_p:.2f}"
            )
            self.current_t = None

def main(args=None):
    rclpy.init()
    node = PlotTrajectory()
    rclpy.spin(node)
    rclpy.shutdown()

#if __name__ == '__main__':
#    main()
