import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from tier4_external_api_msgs.msg import EngageStatus
from autoware_auto_system_msgs.msg import AutowareState
from autoware_auto_vehicle_msgs.msg import VelocityReport
import subprocess
import time
import threading

class GoalPublisherAndEngage(Node):
    def __init__(self):
        super().__init__('goal_publisher_and_engage')

        # 发布目标消息到话题 "/planning/mission_planning/goal"
        self.goal_publisher = self.create_publisher(PoseStamped, '/planning/mission_planning/goal', 10)

        # 订阅 engage 状态
        self.engage_status_subscriber = self.create_subscription(
            EngageStatus,
            '/api/external/get/engage',  # 订阅 engage 状态
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

        # 用于检查 engage 状态的控制变量
        self.engage = False
        self.count = 0  # 目标计数器
        self.waiting_for_engage = False  # 是否处于等待 engage 状态
        self.current_velocity = 0.0  # 当前车速
        self.last_time_below_threshold = None  # 记录车速低于阈值的时间点

        # 用于控制60秒循环的变量
        self.timer = None
        self.get_logger().info('Goal publisher has been initialized')

    def call_quick_start_script(self):
        # 调用 quick_start1.sh 脚本
        script_path = '/home/nvidia/code/kunyi/quick_start1.sh'
        try:
            result = subprocess.run([script_path], check=True, text=True, capture_output=True)
            self.get_logger().info(f"Quick start script executed successfully: {result.stdout}")
            
            # 使用 time.sleep(30) 等待 30 秒
            time.sleep(30)  # 等待 30 秒
            
            # 在 30 秒后发布第一个目标
            self.publish_goal_and_check_engage()

        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Error executing quick start script: {e.stderr}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {str(e)}")

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

    def stop_autoware(self):
        # 调用 stop.sh 脚本停止 Autoware
        script_path = '/home/nvidia/code/kunyi/stop.sh'
        try:
            result = subprocess.run([script_path], check=True, text=True, capture_output=True)
            self.get_logger().info(f"Stop script executed successfully: {result.stdout}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Error executing stop script: {e.stderr}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {str(e)}")

    def on_engage_status_callback(self, msg):
        # 订阅 engage 状态，根据状态进行判断
        self.engage = msg.engage
        self.get_logger().info(f"Received EngageStatus: {self.engage}")
        
        # 如果 engage 为 False 且 Autoware 状态是 WAITING_FOR_ENGAGE, 则定时执行 engage.sh
        if not self.engage:
            self.get_logger().info("Engage is False. Checking Autoware state and re-running engage.sh.")
            if self.waiting_for_engage:
                self.get_logger().info("Autoware is waiting for engage. Re-running engage.sh after 0.5 seconds.")
                self.create_timer(0.5, self.retry_engage_script)  # 在 0.5 秒后尝试再次执行 engage.sh

    def on_autoware_state_callback(self, msg):
        # 监听 Autoware 状态
        if msg.state == AutowareState.WAITING_FOR_ENGAGE:
            self.get_logger().info("Autoware is in WAITING_FOR_ENGAGE state.")
            self.waiting_for_engage = True
        else:
            self.waiting_for_engage = False

    def on_velocity_report(self, msg):
        # 更新当前车速
        self.current_velocity = msg.longitudinal_velocity
        self.get_logger().info(f"Received Velocity: {self.current_velocity} m/s")

        # 检查车速是否低于 0.3 m/s 持续 10 秒
        if self.current_velocity < 0.3:
            if self.last_time_below_threshold is None:
                self.last_time_below_threshold = time.time()
            else:
                if time.time() - self.last_time_below_threshold >= 10.0:
                    self.get_logger().info("Vehicle speed has been below 0.3 m/s for 10 seconds. Stopping current goal publishing.")
                    # 停止定时器并重新启动
                    if self.timer:
                        self.timer.cancel()  # 取消当前定时器
                    self.get_logger().info("Restarting 60-second cycle due to low speed.")
                    self.create_timer(60.0, self.publish_goal_and_check_engage)  # 重新开始60秒计时
        else:
            # 车速恢复正常，重置计时器
            self.last_time_below_threshold = None

    def retry_engage_script(self):
        # 再次执行 engage.sh 脚本
        self.call_engage_script()

    def publish_goal_and_check_engage(self):
        # 发布目标并检查车速
        goal_msg = self.create_goal_message()
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Published goal #{self.count}: x={goal_msg.pose.position.x}, y={goal_msg.pose.position.y}")
        self.count += 1

        # 执行 engage.sh 脚本
        self.call_engage_script()

        # 启动一个定时器，每 60 秒发布目标
        self.timer = self.create_timer(60.0, self.publish_next_goal)

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
            {'Px': 258.93634033203125, 'Py': 284.6453857421875, 'Pz': 0.0, 'Ox': 0.0, 'Oy': 0.0, 'Oz': 0.7100277956406089, 'W': 0.7041736500450281},
            {'Px': 199.97434997558594, 'Py': 232.11181640625, 'Pz': 0.0, 'Ox': 0.0, 'Oy': 0.0, 'Oz': -0.721107438577692, 'W': 0.6928232545374964},
            {'Px': 257.9629211425781, 'Py': 137.67478942871094, 'Pz': 0.0, 'Ox': 0.0, 'Oy': 0.0, 'Oz': 0.7100277956406089, 'W': 0.7041736500450281},
            {'Px': 210.0, 'Py': 116.25, 'Pz': 0.0, 'Ox': 0.0, 'Oy': 0.0, 'Oz': 0.7100277956406089, 'W': 0.7041736500450281}
        ]

        goal = goals[self.count % len(goals)]
        goal_msg.pose.position.x = goal['Px']
        goal_msg.pose.position.y = goal['Py']
        goal_msg.pose.position.z = goal['Pz']
        goal_msg.pose.orientation.x = goal['Ox']
        goal_msg.pose.orientation.y = goal['Oy']
        goal_msg.pose.orientation.z = goal['Oz']
        goal_msg.pose.orientation.w = goal['W']

        return goal_msg

def main():
    rclpy.init()

    node = GoalPublisherAndEngage()
    
    # 监听用户输入
    while True:
        user_input = input("Enter command (stop to stop Autoware): ").strip().lower()
        if user_input == 'stop':
            node.stop_autoware()
        elif user_input == 'exit':
            break
        else:
            print("Unknown command. Please enter 'stop' to stop Autoware.")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
