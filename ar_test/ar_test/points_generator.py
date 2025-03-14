import rclpy,random
from rclpy.node import Node
#from ar_test.msg import CubicTrajParams
from ar_interface.msg import CubicTrajParams


class PointGenerator(Node):
	def __init__(self):
		super().__init__('points_generator')
		self.publisher = self.create_publisher(CubicTrajParams, '/params',10)
		timer_period = 3.0	
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.get_logger().info("Node on, publishing random parameters every 10S")
	
	
	def timer_callback(self):
		msg = CubicTrajParams()
		#msg = cubic_traj_params()
	
		msg.p0 = random.uniform(-10.0,10.0)
		msg.pf = random.uniform(-10.0,10.0)
		
		msg.v0 = random.uniform(-10.0,10.0)
		msg.vf = random.uniform(-10.0,10.0)
		
		msg.t0 = 0.0
		dt = random.uniform(4.0,8.0)
		msg.tf = msg.t0+dt
		
		self.get_logger().info(f'Publishing:{msg}: p0 = {msg.p0}, pf = {msg.pf}, v0 = {msg.v0}, vf = {msg.vf}, t0 = {msg.t0}, tf = {msg.tf}')
		self.publisher.publish(msg)
		
		
	
def main(args=None):
	rclpy.init()
	node = PointGenerator()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
