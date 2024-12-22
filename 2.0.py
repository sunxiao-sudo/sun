import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QTextEdit, QPushButton
from PyQt5 import uic
import yaml
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
        self.vehicle_mode_window = VehicleModeWindow()
        self.vehicle_mode_window.show()


class VehicleModeWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # 初始化子界面
        self.ui = Ui_VehicleModeWindow()
        self.ui.setupUi(self)

        # 加载YAML文件数据
        self.yaml_data_1 = self.load_yaml('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/mirror.param.yaml')
        self.yaml_data_2 = self.load_yaml('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/simulator_model.param.yaml')
        self.yaml_data_3 = self.load_yaml('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/vehicle_info.param.yaml')

        # 填充QTextEdit控件的初始文本
        self.fill_text_edits()

        # 连接QTextEdit控件文本改变信号
        self.textEdits = []
        for i in range(1, 33):
            text_edit = self.findChild(QTextEdit, f'textEdit_{i}')
            self.textEdits.append(text_edit)
            text_edit.textChanged.connect(self.on_text_changed)

    def load_yaml(self, file_path):
        """加载YAML文件"""
        try:
            with open(file_path, 'r') as file:
                return yaml.safe_load(file)
        except Exception as e:
            print(f"Error loading YAML file {file_path}: {e}")
            return {}

    def fill_text_edits(self):
        """填充QTextEdit控件"""
        yaml_data = {
            **self.yaml_data_1, 
            **self.yaml_data_2, 
            **self.yaml_data_3
        }

        mapping = [
            ('wheel_radius', 0), ('wheel_width', 1), ('wheel_base', 2),
            ('front_overhang', 3), ('rear_overhang', 4), ('left_overhang', 5),
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
            value = str(yaml_data.get(key, ''))
            text_edit = self.textEdits[idx]
            text_edit.setText(value)

    def on_text_changed(self):
        """当文本修改时，更新对应的YAML文件中的值"""
        sender = self.sender()
        text = sender.toPlainText()

        mapping = [
            ('wheel_radius', 0), ('wheel_width', 1), ('wheel_base', 2),
            ('front_overhang', 3), ('rear_overhang', 4), ('left_overhang', 5),
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
                # 更新对应的yaml文件中的值
                if idx < 12:  # mirror.param.yaml
                    self.yaml_data_1[key] = text
                    self.save_yaml('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/mirror.param.yaml', self.yaml_data_1)
                elif idx < 24:  # simulator_model.param.yaml
                    self.yaml_data_2[key] = text
                    self.save_yaml('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/simulator_model.param.yaml', self.yaml_data_2)
                else:  # vehicle_info.param.yaml
                    self.yaml_data_3[key] = text
                    self.save_yaml('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/vehicle_info.param.yaml', self.yaml_data_3)

    def save_yaml(self, file_path, data):
        """保存YAML文件"""
        try:
            with open(file_path, 'w') as file:
                yaml.safe_dump(data, file)
        except Exception as e:
            print(f"Error saving YAML file {file_path}: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()  # 启动主界面
    window.show()
    sys.exit(app.exec_())
