import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QTextEdit, QPushButton
from PyQt5 import uic
import ruamel.yaml
from collections import OrderedDict  # 使用OrderedDict来确保顺序
from VehicleMode import Ui_VehicleModeWindow  # 导入子界面的UI类
from autoware import Ui_autowareWindow  # 导入主界面的UI类

# 特定键，需要双引号
quote_keys = {
    "base_link", "map", "DEAL_STEER_VEL", "IDEAL_STEER_ACC", 
    "IDEAL_STEER_ACC_GEARED", "DELAY_STEER_ACC", "DELAY_STEER_ACC_GEARED", 
    "ORIGIN", "INITIAL_POSE_TOPIC"
}

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # 初始化主界面
        self.ui = Ui_autowareWindow()
        self.ui.setupUi(self)

        # 连接主界面按钮事件
        self.ui.pushButton_7.clicked.connect(self.open_vehicle_mode_window)

    def open_vehicle_mode_window(self):
        """打开子界面"""
        print("Opening Vehicle Mode window...")
        self.vehicle_mode_window = VehicleModeWindow()
        self.vehicle_mode_window.show()


class VehicleModeWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # 初始化子界面
        print("Initializing Vehicle Mode window...")
        self.ui = Ui_VehicleModeWindow()
        self.ui.setupUi(self)

        # 加载YAML文件数据
        print("Loading YAML data...")
        self.yaml_data_1 = self.load_yaml('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/mirror.param.yaml')
        self.yaml_data_2 = self.load_yaml('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/simulator_model.param.yaml')
        self.yaml_data_3 = self.load_yaml('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/vehicle_info.param.yaml')

        # 初始化 textEdits 列表
        self.textEdits = []

        # 填充 QTextEdit 控件的初始文本
        print("Filling QTextEdit controls with YAML data...")
        self.fill_text_edits()

        # 连接 QTextEdit 控件文本改变信号
        self.connect_text_edits()

    def load_yaml(self, file_path):
        """加载 YAML 文件并确保顺序"""
        print(f"Loading YAML file: {file_path}")
        try:
            with open(file_path, 'r') as file:
                yaml = ruamel.yaml.YAML()
                # 使用 ruamel.yaml 来加载 YAML，保留顺序和注释
                data = yaml.load(file)

                # 对特定键（如 base_link, map 等）添加双引号
                self.add_quotes_to_special_keys(data)

                print(f"Data loaded from {file_path}: {data}")
                return data
        except Exception as e:
            print(f"Error loading YAML file {file_path}: {e}")
            return {}

    def add_quotes_to_special_keys(self, data):
        """确保特定的字符串键带双引号"""
        for key in data.get('/**', {}).get('ros__parameters', {}):
            if key in quote_keys and isinstance(data['/**']['ros__parameters'][key], str):
                data['/**']['ros__parameters'][key] = f'"{data["/**"]["ros__parameters"][key]}"'

    def fill_text_edits(self):
        """填充 QTextEdit 控件"""
        # 获取每个 YAML 文件的 ros__parameters 部分
        yaml_data_1 = self.yaml_data_1.get('/**', {}).get('ros__parameters', {})
        yaml_data_2 = self.yaml_data_2.get('/**', {}).get('ros__parameters', {})
        yaml_data_3 = self.yaml_data_3.get('/**', {}).get('ros__parameters', {})

        print(f"YAML data for mirror.param.yaml: {yaml_data_1}")
        print(f"YAML data for simulator_model.param.yaml: {yaml_data_2}")
        print(f"YAML data for vehicle_info.param.yaml: {yaml_data_3}")

        # 控件和 YAML 键值的映射关系
        mapping = [
            ('min_longitudinal_offset', 0), ('max_longitudinal_offset', 1), ('min_lateral_offset', 2),
            ('max_lateral_offset', 3), ('min_height_offset', 4), ('max_height_offset', 5),
            ('simulated_frame_id', 6), ('origin_frame_id', 7), ('vehicle_model_type', 8),
            ('initialize_source', 9), ('timer_sampling_time_ms', 10), ('add_measurement_noise', 11),
            ('vel_lim', 12), ('vel_rate_lim', 13), ('steer_lim', 14), ('steer_rate_lim', 15),
            ('acc_time_delay', 16), ('acc_time_constant', 17), ('steer_time_delay', 18),
            ('steer_time_constant', 19), ('x_stddev', 20), ('y_stddev', 21),
            ('wheel_radius', 22), ('wheel_width', 23), ('wheel_base', 24),
            ('wheel_tread', 25), ('front_overhang', 26), ('vehicle_height', 27),
            ('rear_overhang', 28), ('left_overhang', 29), ('right_overhang', 30),
            ('max_steer_angle', 31)
        ]

        # 填充数据到 QTextEdit 控件
        for key, idx in mapping:
            value = ''
            if key in yaml_data_1:  # 从 mirror.param.yaml 中获取值
                value = str(yaml_data_1.get(key, ''))
            elif key in yaml_data_2:  # 从 simulator_model.param.yaml 中获取值
                value = str(yaml_data_2.get(key, ''))
            elif key in yaml_data_3:  # 从 vehicle_info.param.yaml 中获取值
                value = str(yaml_data_3.get(key, ''))

            # 确保 textEdits 已经初始化
            if len(self.textEdits) <= idx:
                text_edit = self.findChild(QTextEdit, f'textEdit_{idx + 1:02d}')  # 格式化为 2 位数
                if text_edit:
                    self.textEdits.append(text_edit)
            # 更新 QTextEdit 控件的文本
            if idx < len(self.textEdits):
                print(f"Setting text for {key} (textEdit_{idx + 1:02d}) to: {value}")
                self.textEdits[idx].setText(value)

    def connect_text_edits(self):
        """连接每个 QTextEdit 的 textChanged 信号"""
        for i in range(32):
            text_edit = self.textEdits[i] if i < len(self.textEdits) else None
            if text_edit:
                print(f"Connecting textChanged signal for textEdit_{i + 1:02d}")
                text_edit.textChanged.connect(self.on_text_changed)

    def on_text_changed(self):
        """当文本修改时，更新对应的 YAML 文件中的值"""
        sender = self.sender()
        text = sender.toPlainText()

        mapping = [
            ('min_longitudinal_offset', 0), ('max_longitudinal_offset', 1), ('min_lateral_offset', 2),
            ('max_lateral_offset', 3), ('min_height_offset', 4), ('max_height_offset', 5),
            ('simulated_frame_id', 6), ('origin_frame_id', 7), ('vehicle_model_type', 8),
            ('initialize_source', 9), ('timer_sampling_time_ms', 10), ('add_measurement_noise', 11),
            ('vel_lim', 12), ('vel_rate_lim', 13), ('steer_lim', 14), ('steer_rate_lim', 15),
            ('acc_time_delay', 16), ('acc_time_constant', 17), ('steer_time_delay', 18),
            ('steer_time_constant', 19), ('x_stddev', 20), ('y_stddev', 21),
            ('wheel_radius', 22), ('wheel_width', 23), ('wheel_base', 24),
            ('wheel_tread', 25), ('front_overhang', 26), ('vehicle_height', 27),
            ('rear_overhang', 28), ('left_overhang', 29), ('right_overhang', 30),
            ('max_steer_angle', 31)
        ]

        for key, idx in mapping:
            if sender == self.textEdits[idx]:
                print(f"Updating {key} with new value: {text}")
                # 根据文本内容更新相应的 YAML 数据
                value = text  # 保持为原文本

                # 根据 key 将值更新到相应的数据字典
                if idx < 12:
                    self.yaml_data_1['/**']['ros__parameters'][key] = value
                elif idx < 24:
                    self.yaml_data_2['/**']['ros__parameters'][key] = value
                else:
                    self.yaml_data_3['/**']['ros__parameters'][key] = value

                # 保存 YAML 数据
                self.save_yaml()

    def save_yaml(self):
        """保存 YAML 数据到文件"""
        try:
            # 保存镜像参数文件
            with open('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/mirror.param.yaml', 'w') as file:
                yaml = ruamel.yaml.YAML()
                yaml.dump(self.yaml_data_1, file)
            with open('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/simulator_model.param.yaml', 'w') as file:
                yaml.dump(self.yaml_data_2, file)
            with open('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/vehicle_info.param.yaml', 'w') as file:
                yaml.dump(self.yaml_data_3, file)

            print("YAML data saved successfully.")
        except Exception as e:
            print(f"Error saving YAML data: {e}")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
