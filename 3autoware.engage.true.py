import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from tier4_external_api_msgs.msg import EngageStatus  # 导入 EngageStatus 消息类型
import subprocess
import time

class GoalPublisherAndEngage(Node):
    def __init__(self):
        super().__init__('goal_publisher_and_engage')
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/planning/mission_planning/goal',
            10
        )
        self.state_subscriber = self.create_subscription(
            EngageStatus,
            '/api/external/get/engage',  # 订阅 engage 状态话题
            self.on_engage_status_callback,
            10
        )
        self.timer = None  # 初始化为 None
        self.count = 0
        self.running = False  # 控制是否开始发布目标和执行脚本
        self.get_logger().info('Goal publisher has been initialized')

    def on_engage_status_callback(self, msg):
        # 检查 engage 状态
        if msg.engage:  # 如果 engage 为 True，开始发布目标并执行脚本
            self.get_logger().info("Engage is true, starting goal publishing and script execution.")
            self.start_timer()  # 启动定时器每30秒调用一次 timer_callback
        else:
            self.get_logger().info("Engage is false, stopping goal publishing.")
            if self.timer:
                self.timer.cancel()  # 如果 engage 为 False，停止定时器
                self.timer = None

    def start_timer(self):
        if not self.timer:  # 如果定时器还没有启动
            self.timer = self.create_timer(30.0, self.timer_callback)  # 每30秒调用一次 timer_callback
            self.get_logger().info("Timer started for goal publishing and script execution.")

    def timer_callback(self):
        if self.running:  # 确保节点仍在运行
            self.publish_goal_and_check_engage()

    def publish_goal_and_check_engage(self):
        # 发布目标消息
        goal_msg = self.create_goal_message()
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(
            f"Published goal #{self.count}: x={goal_msg.pose.position.x}, y={goal_msg.pose.position.y}")
        self.count += 1

        # 执行 engage 脚本
        self.call_engage_script()

    def create_goal_message(self):
        # 构建目标消息
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'

        # 预定义的目标位置
        goals = [
            {'Px': 258.93634033203125, 'Py': 284.6453857421875, 'Pz': 0.0, 'Ox': 0.0, 'Oy': 0.0, 'Oz': 0.7100277956406089, 'W': 0.7041736500450281},
            {'Px': 199.97434997558594, 'Py': 232.11181640625, 'Pz': 0.0, 'Ox': 0.0, 'Oy': 0.0, 'Oz': -0.721107438577692, 'W': 0.6928232545374964},
            {'Px': 257.9629211425781, 'Py': 137.67478942871094, 'Pz': 0.0, 'Ox': 0.0, 'Oy': 0.0, 'Oz': 0.7100277956406089, 'W': 0.7041736500450281},
        ]

        current_goal = goals[self.count % len(goals)]
        goal_msg.pose.position.x = current_goal['Px']
        goal_msg.pose.position.y = current_goal['Py']
        goal_msg.pose.position.z = current_goal['Pz']
        goal_msg.pose.orientation.x = current_goal['Ox']
        goal_msg.pose.orientation.y = current_goal['Oy']
        goal_msg.pose.orientation.z = current_goal['Oz']
        goal_msg.pose.orientation.w = current_goal['W']

        return goal_msg

    def call_engage_script(self):
        # 调用 engage.sh 脚本
        script_path = '/home/nvidia/code/kunyi/engage.sh'
        try:
            result = subprocess.run([script_path], check=True, text=True, capture_output=True)
            self.get_logger().info(f"Engage script executed successfully: {result.stdout}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Error executing engage script: {e.stderr}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {str(e)}")

def main():
    # 初始化 ROS 2
    rclpy.init()

    # 创建 GoalPublisherAndEngage 节点
    node = GoalPublisherAndEngage()

    # 启动多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # 启动 ROS 节点
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
