import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
import math

# WGS84椭球参数
a = 6378137.0  # 长半轴
b = 6356752.3142  # 短半轴
e2 = 1 - (b*b)/(a*a)

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

def update_robot_state(robot_state):
    x, y, theta = robot_state
    print(f"机器人状态: x={x:.3f} m, y={y:.3f} m, theta={theta:.2f} deg")

class LatLonXYThetaPublisher(Node):
    def __init__(self):
        super().__init__('latlon_xy_theta_pub')
        self.lat = None
        self.lon = None
        self.theta = None
        self.origin_set = False

        # 声明并获取原点参数
        self.declare_parameter('lat0', None)
        self.declare_parameter('lon0', None)
        lat0_param = self.get_parameter('lat0').get_parameter_value()
        lon0_param = self.get_parameter('lon0').get_parameter_value()
        self.lat0 = lat0_param.double_value if lat0_param.type == 2 else None
        self.lon0 = lon0_param.double_value if lon0_param.type == 2 else None
        if self.lat0 is not None and self.lon0 is not None:
            self.origin_set = True
            self.get_logger().info(f"原点手动设置为: lat0={self.lat0}, lon0={self.lon0}")

        self.create_subscription(NavSatFix, '/sensing/gnss', self.gnss_callback, 10)
        self.create_subscription(Imu, '/sensing/imu', self.imu_callback, 10)
        self.timer = self.create_timer(0.1, self.publish_info)

    def gnss_callback(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        # 如果未手动设置原点，首次收到GNSS数据时自动设置
        if not self.origin_set and self.lat is not None and self.lon is not None:
            self.lat0 = self.lat
            self.lon0 = self.lon
            self.origin_set = True
            self.get_logger().info(f"原点自动设置为: lat0={self.lat0}, lon0={self.lon0}")

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.theta = math.degrees(yaw)
        if self.theta < 0:
            self.theta += 360

    def publish_info(self):
        if self.origin_set and self.lat is not None and self.lon is not None and self.theta is not None:
            x, y = latlon_to_xy(self.lat0, self.lon0, self.lat, self.lon)
            robot_state = [x, y, self.theta]
            update_robot_state(robot_state)

def main(args=None):
    rclpy.init(args=args)
    node = LatLonXYThetaPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



python latlon_xy_theta_pub.py --ros-args -p lat0:=31.123456 -p lon0:=121.654321






























pip install ros_numpy



import rclpy
from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

def pointcloud2_to_xy(msg):
    # 解析点云消息为 (N, 3) 的 numpy 数组
    points = np.array([ [p[0], p[1]] for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True) ])
    # 转置为 (2, N)
    return points.T

# 假设 state 是当前机器人位姿
action, info = planner.forward(state, points)








def ros_pointcloud2_to_project_points(msg, min_distance=0.2, max_distance=200):
    import numpy as np
    import sensor_msgs_py.point_cloud2 as pc2
    points = []
    for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True):
        dist = np.hypot(p[0], p[1])
        if min_distance <= dist <= max_distance:
            points.append([p[0], p[1]])
    if len(points) == 0:
        return None
    return np.array(points).T  # shape (2, N)








import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np

def pointcloud_callback(msg):
    # 提取x, y坐标
    points = []
    for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True):
        points.append([p[0], p[1]])
    if len(points) == 0:
        return
    points_np = np.array(points).T  # (2, N)
    # 这里可以直接传给你的主流程
    # action, info = planner.forward(state, points_np)

rospy.init_node('pointcloud_listener')
rospy.Subscriber('/your_pointcloud_topic', PointCloud2, pointcloud_callback)
rospy.spin()







import rospy
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np

def pointcloud_callback(msg):
    # 将PointCloud2消息转换为numpy结构化数组
    pc = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    # 提取x, y坐标并过滤无效点
    x = pc['x']
    y = pc['y']
    mask = ~np.isnan(x) & ~np.isnan(y)
    points = np.vstack((x[mask], y[mask]))  # shape: (2, N)
    # 这里可以将points传递给你的主流程
    # action, info = planner.forward(state, points)
    print(f"点云数量: {points.shape[1]}")

if __name__ == '__main__':
    rospy.init_node('pointcloud_listener')
    rospy.Subscriber('/your_pointcloud_topic', PointCloud2, pointcloud_callback)
    rospy.spin()

- /your_pointcloud_topic 替换为你实际的点云话题名（如 /velodyne_points ）。
- ros_numpy 可以方便地将 PointCloud2 转为 numpy 数组，便于后续处理。
- 得到的 points 就是 shape=(2, N) 的二维点云，可以直接对接你的规划主流程。







































import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import ControlCommand  # 替换为你的实际消息类型

class AckerControlPublisher(Node):
    def __init__(self):
        super().__init__('acker_control_publisher')
        self.publisher_ = self.create_publisher(ControlCommand, '/control/command/control_cmd', 10)

    def send_control(self, v, steering_angle):
        msg = ControlCommand()
        msg.velocity = float(v)           # 线速度，单位通常为 m/s
        msg.steering_angle = float(steering_angle)  # 转向角，单位通常为弧度
        self.publisher_.publish(msg)
        self.get_logger().info(f'发布控制指令: v={v:.2f}, steering_angle={steering_angle:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = AckerControlPublisher()
    # 示例：假设你已获得 action = [v, steering_angle]
    v = 1.0
    steering_angle = 0.2
    node.send_control(v, steering_angle)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
