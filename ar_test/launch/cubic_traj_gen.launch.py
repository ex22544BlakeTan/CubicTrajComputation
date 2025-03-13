from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
	return LaunchDescription([
		Node(
			package = 'ar_test',
			execuatble= 'points_generator',
			name= 'points_generator',
			output= 'screen'
		),
		Node(
			package = 'ar_test',
			execuatble= 'cubic_traj_planner',
			name= 'cubic_traj_planner',
			output= 'screen'
		),
		Node(
			package = 'ar_test',
			execuatble= 'compute_cubic_coeffs',
			name= 'compute_cubic_coeffs',
			output= 'screen'
		),
		Node(
			package = 'ar_test',
			execuatble= 'plot_cubic_traj',
			name= 'plot_cubic_traj',
			output= 'screen'
		),
		])
