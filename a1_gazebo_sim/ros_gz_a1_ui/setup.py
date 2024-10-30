from setuptools import find_packages, setup

package_name = 'ros_gz_a1_ui'

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
    maintainer='kdawg',
    maintainer_email='karsten.appenzeller@gmail.com',
    description='A collection of ROS2 Nodes to send /pose and /cmd_vel messages using a Pygame GUI',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_pub_gui = ros_gz_a1_ui.pose_pub_gui:main',
            'cmdvel_pub_gui = ros_gz_a1_ui.pose_pub_gui:main',

        ],
    },
)
