from setuptools import setup

package_name = 'goal_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'geometry_msgs'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A ROS 2 package to publish goals and engage scripts',
    license='Your License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_publisher = goal_publisher.goal_publisher:main',  # 添加这一行
        ],
    },
)
