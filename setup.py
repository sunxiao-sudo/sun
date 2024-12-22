from setuptools import setup

package_name = 'goal_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy', 'geometry_msgs'],
    entry_points={
        'console_scripts': [
            'goal_publisher = goal_publisher.goal_publisher:main',  # 注意这里的配置
        ],
    },
)
