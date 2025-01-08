import rclpy
from rclpy.node import Node
from autoware_auto_system_msgs.msg import AutowareState
import subprocess

class AutoEngage(Node):

    def __init__(self):
        super().__init__('auto_engage_node')

        # 创建订阅者，订阅 /autoware/state 话题
        self.subscriber = self.create_subscription(
            AutowareState,
            '/autoware/state',
            self.state_callback,
            10
        )

    def state_callback(self, msg):
        # 当接收到状态消息时检查是否为 WAITING_FOR_ENGAGE
        if msg.state == AutowareState.WAITING_FOR_ENGAGE:
            self.get_logger().info("State is WAITING_FOR_ENGAGE. Executing engage.sh...")
            self.execute_engage_script()

    def execute_engage_script(self):
        try:
            # 执行 engage.sh 脚本
            result = subprocess.run(['/path/to/engage.sh'], check=True)
            if result.returncode == 0:
                self.get_logger().info("engage.sh executed successfully.")
            else:
                self.get_logger().error(f"Failed to execute engage.sh. Return code: {result.returncode}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Error executing engage.sh: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    # 创建节点并启动
    auto_engage_node = AutoEngage()
    rclpy.spin(auto_engage_node)
    
    # 销毁节点
    auto_engage_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
