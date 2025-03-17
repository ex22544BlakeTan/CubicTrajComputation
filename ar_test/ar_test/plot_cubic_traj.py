import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from ar_interface.msg import CubicTrajCoeffs  # 替换为实际包名和消息名

class PlotCubicTraj(Node):
    def __init__(self):
        super().__init__('plot_cubic_traj')
        
        # 订阅 /cubic_traj_coeffs 话题
        self.subscription = self.create_subscription(
            CubicTrajCoeffs,
            '/cubic_traj_coeffs',
            self.coeffs_callback,
            10
        )
        
        # 发布位置、速度、加速度轨迹话题
        self.pos_publisher = self.create_publisher(Float32, '/position_trajectory', 10)
        self.vel_publisher = self.create_publisher(Float32, '/velocity_trajectory', 10)
        self.acc_publisher = self.create_publisher(Float32, '/acceleration_trajectory', 10)
        
        # 初始化参数
        self.current_coeffs = None
        self.t0 = 0.0
        self.tf = 0.0
        
    def coeffs_callback(self, msg):
        """收到新系数时的回调函数"""
        self.get_logger().info("收到新轨迹系数！")
        
        # 保存系数和时间参数
        self.current_coeffs = {
            'a0': msg.a0,
            'a1': msg.a1,
            'a2': msg.a2,
            'a3': msg.a3
        }
        self.t0 = msg.t0
        self.tf = msg.tf
        
        # 计算并发布轨迹
        self.publish_trajectories()
    
    def publish_trajectories(self):
        """计算并发布位置、速度、加速度轨迹"""
        if not self.current_coeffs:
            return
        
        # 计算轨迹数据
        time_step = 0.1  # 时间步长
        current_time = self.t0
        
        while current_time <= self.tf:
            # 计算时间偏移
            t_offset = current_time - self.t0
            
            # 计算位置、速度、加速度
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
            
            # 发布消息
            self.publish_value(self.pos_publisher, pos)
            self.publish_value(self.vel_publisher, vel)
            self.publish_value(self.acc_publisher, acc)
            
            # 更新时间
            current_time += time_step
        
        self.get_logger().info("轨迹生成完成，等待下一组参数...")
    
    def publish_value(self, publisher, value):
        """发布单个值到指定话题"""
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
