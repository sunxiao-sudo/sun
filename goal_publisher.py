import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
import subprocess

class GoalPublisherAndEngage(Node):
    def __init__(self):
        super().__init__('goal_publisher_and_engage')
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/planning/mission_planning/goal',
            10
        )
        self.timer = self.create_timer(30.0, self.timer_callback)
        self.count = 0
        self.running = True
        self.get_logger().info('Goal publisher has been initialized')

    def timer_callback(self):
        if self.running:
            self.publish_goal_and_engage()

    def publish_goal_and_engage(self):
        goal_msg = self.create_goal_message()
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(
            f"Published goal #{self.count}: x={goal_msg.pose.position.x}, y={goal_msg.pose.position.y}")
        self.count += 1
        self.call_engage_script()

    def create_goal_message(self):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'

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
        script_path = '/home/nvidia/code/kunyi/engage.sh'
        try:
            result = subprocess.run([script_path], check=True, text=True, capture_output=True)
            self.get_logger().info(f"Engage script executed successfully: {result.stdout}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Error executing engage script: {e.stderr}")

def main():
    rclpy.init()

    node = GoalPublisherAndEngage()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
