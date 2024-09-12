from setuptools import find_packages, setup

package_name = 'ros_gz_a1_controller'

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
    maintainer='k-dawg',
    maintainer_email='karsten.appenzeller@stud.uni-due.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher = ros_gz_a1_controller.joint_state_publisher:main',
            'pose_pub_gui = ros_gz_a1_controller.pose_pub_gui:main',
            'crawl_controller = ros_gz_a1_controller.crawl_controller:main',
            'stand_controller = ros_gz_a1_controller.stand_controller:main',
        ],
    },
)
