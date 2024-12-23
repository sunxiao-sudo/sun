import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QTextEdit, QPushButton
from PyQt5 import uic
import ruamel.yaml
from collections import OrderedDict
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
            yaml = ruamel.yaml.YAML()

            # 设置为不使用默认的引号，只对字符串值加双引号
            yaml.default_style = None  # 默认为不加引号
            yaml.preserve_quotes = True  # 保留原始文件中的引号

            # 加载YAML文件内容
            data = yaml.load(open(file_path, 'r'))
            print(f"Data loaded from {file_path}: {data}")
            return data
        except Exception as e:
            print(f"Error loading YAML file {file_path}: {e}")
            return {}

    def save_yaml(self, file_path, data):
        """保存 YAML 文件时确保值为字符串时加引号，值为其他类型时不加引号"""
        print(f"Saving YAML file: {file_path}")

        try:
            yaml = ruamel.yaml.YAML()
            yaml.default_style = None  # 不为默认值加引号
            yaml.preserve_quotes = True  # 保留原始引号

            # 处理所有键值对
            def representer(dumper, data):
                if isinstance(data, str):  # 如果值是字符串
                    return dumper.represent_scalar('tag:yaml.org,2002:str', data, style='"')  # 给字符串值加双引号
                return dumper.represent_scalar('tag:yaml.org,2002:str', data)  # 非字符串值保持原样

            yaml.representer.add_representer(str, representer)

            # 将数据写入文件
            with open(file_path, 'w') as f:
                yaml.dump(data, f)
            print(f"YAML file saved successfully to {file_path}")

        except Exception as e:
            print(f"Error saving YAML file {file_path}: {e}")

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
                if idx < 6:  # mirror.param.yaml
                    self.yaml_data_1['/**']['ros__parameters'][key] = text
                elif idx < 22:  # simulator_model.param.yaml
                    self.yaml_data_2['/**']['ros__parameters'][key] = text
                else:  # vehicle_info.param.yaml
                    self.yaml_data_3['/**']['ros__parameters'][key] = text

                # 保存更新后的 YAML 文件
                if idx < 6:
                    self.save_yaml('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/mirror.param.yaml', self.yaml_data_1)
                elif idx < 22:
                    self.save_yaml('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/simulator_model.param.yaml', self.yaml_data_2)
                else:
                    self.save_yaml('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/vehicle_info.param.yaml', self.yaml_data_3)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()  # 启动主界面
    window.show()
    sys.exit(app.exec_())
