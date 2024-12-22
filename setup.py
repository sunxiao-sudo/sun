from setuptools import setup

package_name = 'goal_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Goal publisher for ROS2',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_publisher = goal_publisher.goal_publisher:main',  # 确保 main() 方法的路径正确
        ],
    },
)
