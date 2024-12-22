import sys
import os
import yaml
from PyQt5.QtWidgets import QApplication, QMainWindow, QTextEdit, QVBoxLayout, QPushButton, QWidget
from VehicleMode import Ui_VehicleModeWindow  # 子界面的UI类名
from autoware import Ui_autowareWindow  # 主界面的UI类名


class VehicleModeWindow(QMainWindow, Ui_VehicleModeWindow):
    def __init__(self, yaml_data, yaml_file):
        super().__init__()
        self.setupUi(self)  # 设置UI界面

        # 将YAML文件数据和文件路径传入
        self.yaml_data = yaml_data
        self.yaml_file = yaml_file

        # 打印检查yaml_data
        print(f"Loaded YAML Data in VehicleModeWindow: {self.yaml_data}")

        # 映射YAML数据到QTextEdit控件
        self.map_yaml_to_controls()

        # 连接每个QTextEdit控件的textChanged信号，自动保存
        for i in range(1, 33):
            text_edit = getattr(self, f'textEdit_{i:02d}', None)
            if text_edit:
                text_edit.textChanged.connect(self.save_to_yaml)

    def map_yaml_to_controls(self):
        """将YAML文件中的参数映射到控件中"""
        print("Mapping YAML data to controls...")  # 打印映射过程

        # 映射 YAML 数据到对应的 QTextEdit 控件
        self.textEdit_01.setText(str(self.yaml_data.get('wheel_radius', '')))
        self.textEdit_02.setText(str(self.yaml_data.get('wheel_width', '')))
        self.textEdit_03.setText(str(self.yaml_data.get('wheel_base', '')))
        self.textEdit_04.setText(str(self.yaml_data.get('front_overhang', '')))
        self.textEdit_05.setText(str(self.yaml_data.get('rear_overhang', '')))
        self.textEdit_06.setText(str(self.yaml_data.get('left_overhang', '')))
        self.textEdit_07.setText(str(self.yaml_data.get('simulated_frame_id', '')))
        self.textEdit_08.setText(str(self.yaml_data.get('origin_frame_id', '')))
        self.textEdit_09.setText(str(self.yaml_data.get('vehicle_model_type', '')))
        self.textEdit_10.setText(str(self.yaml_data.get('initialize_source', '')))
        self.textEdit_11.setText(str(self.yaml_data.get('timer_sampling_time_ms', '')))
        self.textEdit_12.setText(str(self.yaml_data.get('add_measurement_noise', '')))
        self.textEdit_13.setText(str(self.yaml_data.get('vel_lim', '')))
        self.textEdit_14.setText(str(self.yaml_data.get('vel_rate_lim', '')))
        self.textEdit_15.setText(str(self.yaml_data.get('steer_lim', '')))
        self.textEdit_16.setText(str(self.yaml_data.get('steer_rate_lim', '')))
        self.textEdit_17.setText(str(self.yaml_data.get('acc_time_delay', '')))
        self.textEdit_18.setText(str(self.yaml_data.get('acc_time_constant', '')))
        self.textEdit_19.setText(str(self.yaml_data.get('steer_time_delay', '')))
        self.textEdit_20.setText(str(self.yaml_data.get('steer_time_constant', '')))
        self.textEdit_21.setText(str(self.yaml_data.get('x_stddev', '')))
        self.textEdit_22.setText(str(self.yaml_data.get('y_stddev', '')))
        self.textEdit_23.setText(str(self.yaml_data.get('wheel_radius', '')))
        self.textEdit_24.setText(str(self.yaml_data.get('wheel_width', '')))
        self.textEdit_25.setText(str(self.yaml_data.get('wheel_base', '')))
        self.textEdit_26.setText(str(self.yaml_data.get('wheel_tread', '')))
        self.textEdit_27.setText(str(self.yaml_data.get('front_overhang', '')))
        self.textEdit_28.setText(str(self.yaml_data.get('vehicle_height', '')))
        self.textEdit_29.setText(str(self.yaml_data.get('rear_overhang', '')))
        self.textEdit_30.setText(str(self.yaml_data.get('left_overhang', '')))
        self.textEdit_31.setText(str(self.yaml_data.get('right_overhang', '')))
        self.textEdit_32.setText(str(self.yaml_data.get('max_steer_angle', '')))

        print("YAML data mapping completed.")  # 显示映射完成

    def save_to_yaml(self):
        """保存修改后的值回YAML文件"""
        try:
            updated_data = {}  # 用来存储所有修改过的键值对

            # 只更新修改过的参数
            for i in range(1, 33):
                text_edit = getattr(self, f'textEdit_{i:02d}', None)
                if text_edit:
                    key = self.get_yaml_key_for_text_edit(i)
                    new_value = self.parse_float(text_edit.toPlainText())

                    # 如果值发生变化，更新yaml_data中的相应键
                    if self.yaml_data.get(key) != new_value:
                        updated_data[key] = new_value

            # 如果有修改，更新YAML文件
            if updated_data:
                with open(self.yaml_file, 'r') as file:
                    original_data = yaml.safe_load(file) or {}

                # 更新修改过的参数
                original_data.update(updated_data)

                # 保存回原始文件
                with open(self.yaml_file, 'w') as file:
                    yaml.dump(original_data, file)

                print(f"YAML file {self.yaml_file} updated successfully.")
            else:
                print("No changes detected, skipping save.")

        except Exception as e:
            print(f"Error saving YAML file: {e}")

    def parse_float(self, value):
        """尝试将输入的字符串解析为浮动数字，失败时返回默认值0.0"""
        try:
            return float(value)
        except ValueError:
            return 0.0

    def get_yaml_key_for_text_edit(self, index):
        """返回与每个textEdit控件对应的YAML参数名称"""
        key_map = {
            1: 'wheel_radius', 2: 'wheel_width', 3: 'wheel_base',
            4: 'front_overhang', 5: 'rear_overhang', 6: 'left_overhang',
            7: 'simulated_frame_id', 8: 'origin_frame_id', 9: 'vehicle_model_type',
            10: 'initialize_source', 11: 'timer_sampling_time_ms', 12: 'add_measurement_noise',
            13: 'vel_lim', 14: 'vel_rate_lim', 15: 'steer_lim',
            16: 'steer_rate_lim', 17: 'acc_time_delay', 18: 'acc_time_constant',
            19: 'steer_time_delay', 20: 'steer_time_constant', 21: 'x_stddev',
            22: 'y_stddev', 23: 'wheel_radius', 24: 'wheel_width',
            25: 'wheel_base', 26: 'wheel_tread', 27: 'front_overhang',
            28: 'vehicle_height', 29: 'rear_overhang', 30: 'left_overhang',
            31: 'right_overhang', 32: 'max_steer_angle'
        }
        return key_map.get(index)


class AutowareWindow(QMainWindow, Ui_autowareWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # 修改为指定的目录路径
        self.yaml_directory = '/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config'  # 这里需要指定YAML文件所在的目录
        self.yaml_files = self.load_yaml_files(self.yaml_directory)

        # 点击按钮打开子界面
        self.pushButton_7.clicked.connect(self.open_vehicle_mode_window)

    def load_yaml_files(self, directory):
        """加载目录下的所有YAML文件"""
        yaml_files = {}
        for filename in os.listdir(directory):
            if filename.endswith('.yaml'):
                file_path = os.path.join(directory, filename)
                with open(file_path, 'r') as file:
                    yaml_data = yaml.safe_load(file)
                    yaml_files[file_path] = yaml_data
        return yaml_files

    def open_vehicle_mode_window(self):
        """打开子界面，并将相应的YAML数据传递给子界面"""
        first_yaml_file = list(self.yaml_files.keys())[0]  # 默认打开第一个YAML文件
        yaml_data = self.yaml_files[first_yaml_file]

        # 打印加载的YAML数据，以确认其内容
        print(f"Loaded YAML Data in AutowareWindow: {yaml_data}")

        # 打开子界面并传入YAML数据
        self.vehicle_mode_window = VehicleModeWindow(yaml_data, first_yaml_file)  # 传递文件路径
        self.vehicle_mode_window.show()  # 使用show()来显示主窗口


if __name__ == '__main__':
    app = QApplication(sys.argv)
    autoware_window = AutowareWindow()
    autoware_window.show
