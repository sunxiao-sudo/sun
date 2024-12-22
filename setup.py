from setuptools import setup

package_name = 'goal_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=['goal_publisher.goal_publisher'],  # 脚本所在模块
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A simple ROS 2 package for publishing goals',
    license='License',
    entry_points={
        'console_scripts': [
            'goal_publisher = goal_publisher.goal_publisher:main',  # 设置脚本入口
        ],
    },
)
