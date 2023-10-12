from setuptools import find_packages, setup

package_name = 'team12_chase_object'

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
        'image_processor = team12_chase_object.find_object:main',
        'lidar_processor = team12_chase_object.object_range:main',
        'actuator = team12_chase_object.chase_object_new_control:main',
        'parameter_processor = team12_chase_object.change_parameter:main'
        ],
    },
)
