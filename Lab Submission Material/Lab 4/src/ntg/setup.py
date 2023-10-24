from setuptools import find_packages, setup

package_name = 'ntg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hanyao',
    maintainer_email='hanyao@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'vector_generator = ntg.go_to_goal:main',
        'lidar_processor = ntg.object_range:main',
        'actuator = ntg.control:main',
        'parameter_changer = ntg.change_parameter:main',
        'lidar_viewer = ntg.plot_object_range:main',
        'odom_reseter = ntg.print_fixed_odometry:main',
        'stoper = ntg.stop:main',
        'vector_viewer = ntg.plot_strategy_vectors:main'
        ],
    },
)
