import rclpy,random
from rclpy.node import Node
#from ar_test.msg import CubicTrajParams
from ar_interface.msg import CubicTrajParams


class PointGenerator(Node):
	def __init__(self):
		super().__init__('points_generator')
	
		self.publisher = self.create_publisher(CubicTrajParams, '/cubic_traj_params',10)
		#callback function to produce parameters every 10S
		self.timer = self.create_timer(10.0, self.timer_callback)
		self.get_logger().info("Node on, publishing random parameters every 10S")
	
	#call back function
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
		#self.get_logger().info(f'Parameters:"f"t0={msg.t0},tf={msg.tf:.2f}(dt={dt:.2f}),"f"p0={msg.p0:.2f},pf={msg.pf:.2f}"f"v0={msg.v0:.2f},vf={msg.vf:.2f}")
		
	
def main(args=None):
	rclpy.init(args=args)
	node = PointGenerator()
	rclpy.spin(node)
	rclpy.shutdown()

#if __name__ == '__main__':
#	main()
	
