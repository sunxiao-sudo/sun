from setuptools import setup

package_name = 'goal_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/ament_package', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'goal_publisher/goal_publisher.py']),
    ],
    install_requires=['setuptools', 'rclpy', 'geometry_msgs'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Goal publisher node for mission planning',
    license='Your License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_publisher = goal_publisher.goal_publisher:main',
        ],
    },
)
