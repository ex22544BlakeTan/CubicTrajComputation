from setuptools import find_packages, setup
#from setuptools import setup

package_name = 'ar_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    
    #New setup for message,service and luanch file path
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cubic_traj_gen.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ex22544',
    maintainer_email='ex22544@qmul.ac.uk',
    description='TODO: Package description',
    license='Apache-2.0',
    #tests_require=['pytest'],
    #New entry point for new setup
    entry_points={
        'console_scripts': ['points_generator = ar_test.points_generator:main',
        'cubic_traj_planner = ar_test.cubic_traj_planner:main',
        'compute_cubic_coeffs = ar_test.compute_cubic_coeffs:main',
        'plot_cubic_traj = ar_test.plot_cubic_traj:main'
        ],
    },
)
