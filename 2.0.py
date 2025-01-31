import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QTextEdit, QPushButton
from PyQt5 import uic
import yaml
from collections import OrderedDict  # 使用OrderedDict来确保顺序
from VehicleMode import Ui_VehicleModeWindow  # 导入子界面的UI类
from autoware import Ui_autowareWindow  # 导入主界面的UI类

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
                # 使用OrderedDict来保留顺序
                data = yaml.safe_load(file) or {}
                # 如果返回的是字典，转换为OrderedDict，确保顺序
                if isinstance(data, dict):
                    data = OrderedDict(data)
                print(f"Data loaded from {file_path}: {data}")
                return data
        except Exception as e:
            print(f"Error loading YAML file {file_path}: {e}")
            return {}

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

        print(f"Text changed in sender: {sender} with new value: {text}")

        # 确保索引在范围内
        for key, idx in mapping:
            if sender == self.textEdits[idx]:
                print(f"Updating {key} with new value: {text}")
                # 更新对应的 YAML 文件中的值
                if idx < 12:  # mirror.param.yaml
                    self.yaml_data_1['/**']['ros__parameters'][key] = float(text)  # 确保存储为浮动类型
                    self.save_yaml('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/mirror.param.yaml', self.yaml_data_1)
                elif idx < 24:  # simulator_model.param.yaml
                    self.yaml_data_2[key] = text
                    self.save_yaml('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/simulator_model.param.yaml', self.yaml_data_2)
                else:  # vehicle_info.param.yaml
                    self.yaml_data_3[key] = text
                    self.save_yaml('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/vehicle_info.param.yaml', self.yaml_data_3)

    def save_yaml(self, file_path, data):
        """保存 YAML 文件"""
        print(f"Saving YAML file: {file_path}")
        try:
            # 在保存之前，确保将 OrderedDict 转换为普通字典
            data_dict = self.convert_ordered_dict_to_dict(data)
            print(f"Converted data to dict: {data_dict}")  # 打印转换后的数据，检查其格式
            with open(file_path, 'w') as file:
                # 使用yaml.safe_dump避免Python对象类型
                yaml.safe_dump(data_dict, file, default_flow_style=False, allow_unicode=True)  # 使用safe_dump来避免YAML中嵌入Python对象
            print(f"YAML file saved successfully: {file_path}")
        except Exception as e:
            print(f"Error saving YAML file {file_path}: {e}")

    def convert_ordered_dict_to_dict(self, data):
        """递归地将 OrderedDict 转换为普通字典"""
        if isinstance(data, OrderedDict):
            return {key: self.convert_ordered_dict_to_dict(value) if isinstance(value, (OrderedDict, dict)) else value
                    for key, value in data.items()}
        elif isinstance(data, dict):
            return {key: self.convert_ordered_dict_to_dict(value) for key, value in data.items()}
        else:
            return data


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()  # 启动主界面
    window.show()
    sys.exit(app.exec_())
