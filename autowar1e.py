import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from tier4_external_api_msgs.msg import EngageStatus
from autoware_auto_system_msgs.msg import AutowareState
from autoware_auto_vehicle_msgs.msg import VelocityReport
import subprocess
import time

class GoalPublisherAndEngage(Node):
    def __init__(self):
        super().__init__('goal_publisher_and_engage')

        # 发布目标消息到话题 "/planning/mission_planning/goal"
        self.goal_publisher = self.create_publisher(PoseStamped, '/planning/mission_planning/goal', 10)

        self.count = 0
        self.waiting_for_engage = False  # 是否处于等待 engage 状态
        self.engage = False  # 当前 engage 状态
        self.current_velocity = 0.0  # 当前车速
        self.last_time_below_threshold = None  # 记录车速低于阈值的时间点
        self.timer = None  # 用于控制定时器
        self.engage_retry_timer = None  # 用于执行 engage.sh 的定时器

        # 先发布第一个目标点
        self.publish_goal_and_check_engage()

        # 订阅 engage 状态
        self.engage_status_subscriber = self.create_subscription(
            EngageStatus,
            '/api/external/get/engage',
            self.on_engage_status_callback,
            10
        )

        # 订阅 Autoware 状态
        self.state_subscription = self.create_subscription(
            AutowareState,
            '/autoware/state',
            self.on_autoware_state_callback,
            10
        )

        # 订阅车速报告消息
        self.velocity_report_subscription = self.create_subscription(
            VelocityReport,
            'input/velocity_report',
            self.on_velocity_report,
            10
        )

        self.get_logger().info('Goal publisher and engage system has been initialized')

    def call_engage_script(self):
        script_path = '/home/nvidia/code/kunyi/engage.sh'
        try:
            result = subprocess.run([script_path], check=True, text=True, capture_output=True)
            self.get_logger().info(f"Engage script executed successfully: {result.stdout}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Error executing engage script: {e.stderr}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {str(e)}")

    def on_engage_status_callback(self, msg):
        self.engage = msg.engage
        self.get_logger().info(f"Received EngageStatus: {self.engage}")

        if self.engage:
            # 如果 EngageStatus 为 True，进入 20 秒循环发布下一个目标点
            self.get_logger().info("EngageStatus is True, entering 20-second goal publishing loop.")
            self.create_timer(20.0, self.publish_next_goal)
        else:
            # 如果 EngageStatus 为 False，停止目标发布，等待 0.5 秒后重试
            if not self.engage_retry_timer:
                self.get_logger().info("EngageStatus is False, checking Autoware state and retrying engage script.")
                self.engage_retry_timer = self.create_timer(0.5, self.retry_engage_script)

    def on_autoware_state_callback(self, msg):
        if msg.state == AutowareState.WAITING_FOR_ENGAGE:
            self.get_logger().info("Autoware is in WAITING_FOR_ENGAGE state.")
            self.waiting_for_engage = True
        else:
            self.waiting_for_engage = False

    def on_velocity_report(self, msg):
        # 更新当前车速
        self.current_velocity = msg.longitudinal_velocity
        self.get_logger().info(f"Received Velocity: {self.current_velocity} m/s")

        # 如果车速持续低于 0.3 m/s 且 EngageStatus 为 True，停止目标发布
        if self.current_velocity < 0.3:
            if self.last_time_below_threshold is None:
                self.last_time_below_threshold = time.time()
            else:
                if time.time() - self.last_time_below_threshold >= 10.0:
                    self.get_logger().info("Vehicle speed has been below 0.3 m/s for 10 seconds. Stopping goal publishing.")
                    # 停止定时器并重新启动
                    if self.timer:
                        self.timer.cancel()
                    self.get_logger().info("Restarting 20-second cycle due to low speed.")
                    self.create_timer(20.0, self.publish_goal_and_check_engage)  # 重新开始20秒计时
        else:
            # 车速恢复正常，重置计时器
            self.last_time_below_threshold = None

    def retry_engage_script(self):
        # 每 0.5 秒检查 Engage 状态并执行脚本
        if self.waiting_for_engage and not self.engage:
            self.get_logger().info("EngageStatus is False, retrying engage script.")
            self.call_engage_script()

    def publish_goal_and_check_engage(self):
        # 发布第一个目标并检查 Engage 状态
        goal_msg = self.create_goal_message()
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Published goal #{self.count}: x={goal_msg.pose.position.x}, y={goal_msg.pose.position.y}")
        self.count += 1

        # 执行 engage.sh 脚本
        self.call_engage_script()

        # 检查 EngageStatus，如果为 True 则进入 20 秒循环
        if self.engage:
            self.create_timer(20.0, self.publish_next_goal)

    def publish_next_goal(self):
        # 发布下一个目标
        goal_msg = self.create_goal_message()
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Published goal #{self.count}: x={goal_msg.pose.position.x}, y={goal_msg.pose.position.y}")
        self.count += 1

    def create_goal_message(self):
        # 构建目标消息
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'

        # 定义目标点
        goals = [
            {'Px': -2.711449384689331, 'Py': -10.973952293395996, 'Pz': 0.0, 'Ox': 0.0, 'Oy': 0.0, 'Oz': 0.46181049970081045, 'W': 0.8869786143792238},
            {'Px': 21.6401309967041, 'Py': -9.004338264465332, 'Pz': 0.0, 'Ox': 0.0, 'Oy': 0.0, 'Oz': -0.2434342710308319, 'W': 0.9699173963218144},
            {'Px': 24.50502586364746, 'Py': -34.16167449951172, 'Pz': 0.0, 'Ox': 0.0, 'Oy': 0.0, 'Oz': -0.8847398169089782, 'W': 0.46158524582512567},
            {'Px': -4.05436897277832, 'Py': -39.98098373413086, 'Pz': 0.0, 'Ox': 0.0, 'Oy': 0.0, 'Oz': 0.9361025948113448, 'W': 0.35172707030802647},
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
