import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, NavSatFix, Imu
from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import math

a = 6378137.0
b = 6356752.3142
e2 = 1 - (b * b) / (a * a)

def latlon_to_xy(lat0, lon0, lat, lon):
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    dlat = lat_rad - lat0_rad
    dlon = lon_rad - lon0_rad
    sin_lat0 = math.sin(lat0_rad)
    Rn = a / math.sqrt(1 - e2 * sin_lat0 * sin_lat0)
    Rm = a * (1 - e2) / math.pow(1 - e2 * sin_lat0 * sin_lat0, 1.5)
    x = dlon * Rn * math.cos(lat0_rad)
    y = dlat * Rm
    return x, y

class IntegratedNode(Node):
    def __init__(self):
        super().__init__('integrated_node')

        self.lat = None
        self.lon = None
        self.theta = None
        self.origin_set = False
        self.lat0 = None
        self.lon0 = None
        self.lidar_points = None

        # 初始化动作向量（2x1）
        self.action = np.zeros((2, 1))

        self.create_subscription(PointCloud2, '/sensing/lidar/top/outlier_filtered/pointcloud', self.pointcloud_callback, 10)
        self.create_subscription(NavSatFix, '/sensing/gnss', self.gnss_callback, 10)
        self.create_subscription(Imu, '/sensing/imu', self.imu_callback, 10)
        self.create_subscription(VelocityReport, '/vehicle/status/velocity_status', self.velocity_callback, 10)
        self.create_subscription(SteeringReport, '/vehicle/status/steering_report', self.steering_callback, 10)

        # 定时器（10Hz）
        self.timer = self.create_timer(0.1, self.timer_callback)

    def pointcloud_callback(self, msg):
        points = np.array([[p[0], p[1]] for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)])
        if points.shape[0] == 0:
            return
        self.lidar_points = points.T
        self.get_logger().info(f"点云数量: {self.lidar_points.shape[1]}")

    def gnss_callback(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        if not self.origin_set:
            self.lat0 = self.lat
            self.lon0 = self.lon
            self.origin_set = True
            self.get_logger().info(f"原点自动设置为: lat0={self.lat0}, lon0={self.lon0}")

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.theta = yaw

    def velocity_callback(self, msg: VelocityReport):
        self.action[0, 0] = msg.longitudinal_velocity_mps
        self.get_logger().info(f"速度更新: {self.action[0, 0]:.2f} m/s")

    def steering_callback(self, msg: SteeringReport):
        self.action[1, 0] = msg.steering_tire_angle
        self.get_logger().info(f"转向角更新: {self.action[1, 0]:.2f} rad")

    def timer_callback(self):
        if self.origin_set and self.lat is not None and self.lon is not None and self.theta is not None:
            x, y = latlon_to_xy(self.lat0, self.lon0, self.lat, self.lon)
            self.get_logger().info(f"机器人状态: x={x:.3f}, y={y:.3f}, theta={self.theta:.3f}")
            self.get_logger().info(f"当前动作: speed={self.action[0, 0]:.2f}, steering={self.action[1, 0]:.2f}")
            # 机器人状态
            robot_state = np.array([x, y, self.theta, self.action[0, 0]])  # [x, y, theta, v]
            # 激光雷达扫描数据
            lidar_scan = self.lidar_points if self.lidar_points is not None else np.zeros((2, 0))
            # 目标点（可替换为参考轨迹）
            goal = np.array([x_goal, y_goal])  # 需提前定义目标点坐标
            # 标签（监督信号）：当前速度和转向角
            label = np.array([self.action[0, 0], self.action[1, 0]])  # [v, delta]
            # 打包成训练输入
            train_sample = {
                'robot_state': robot_state,      # 1维向量 [x, y, theta, v]
                'lidar_scan': lidar_scan,        # 2xN 或 1xN 数组
                'goal': goal,                    # 1维向量 [x_goal, y_goal]
                'label': label                   # 1维向量 [v, delta]
            }
            # 传递给训练模型或保存
            # model.train_step(train_sample)
            print(f"train_sample: {train_sample}")
        else:
            self.get_logger().warn("等待GNSS和IMU初始化...")

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
