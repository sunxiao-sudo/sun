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

        # 映射YAML数据到QTextEdit控件
        self.map_yaml_to_controls()

        # 连接每个QTextEdit控件的textChanged信号，自动保存
        self.textedit01.textChanged.connect(self.save_to_yaml)
        self.textedit02.textChanged.connect(self.save_to_yaml)
        self.textedit03.textChanged.connect(self.save_to_yaml)
        self.textedit04.textChanged.connect(self.save_to_yaml)
        self.textedit05.textChanged.connect(self.save_to_yaml)
        self.textedit06.textChanged.connect(self.save_to_yaml)
        self.textedit07.textChanged.connect(self.save_to_yaml)

    def map_yaml_to_controls(self):
        """将YAML文件中的参数映射到控件中"""
        self.textedit01.setText(str(self.yaml_data.get('wheel_radius', '')))
        self.textedit02.setText(str(self.yaml_data.get('wheel_width', '')))
        self.textedit03.setText(str(self.yaml_data.get('wheel_base', '')))
        self.textedit04.setText(str(self.yaml_data.get('front_overhang', '')))
        self.textedit05.setText(str(self.yaml_data.get('rear_overhang', '')))
        self.textedit06.setText(str(self.yaml_data.get('left_overhang', '')))
        self.textedit07.setText(str(self.yaml_data.get('right_overhang', '')))

    def save_to_yaml(self):
        """保存修改后的值回YAML文件"""
        try:
            self.yaml_data['wheel_radius'] = self.parse_float(self.textedit01.toPlainText())
            self.yaml_data['wheel_width'] = self.parse_float(self.textedit02.toPlainText())
            self.yaml_data['wheel_base'] = self.parse_float(self.textedit03.toPlainText())
            self.yaml_data['front_overhang'] = self.parse_float(self.textedit04.toPlainText())
            self.yaml_data['rear_overhang'] = self.parse_float(self.textedit05.toPlainText())
            self.yaml_data['left_overhang'] = self.parse_float(self.textedit06.toPlainText())
            self.yaml_data['right_overhang'] = self.parse_float(self.textedit07.toPlainText())

            # 保存 YAML 文件
            with open(self.yaml_file, 'w') as file:
                yaml.dump(self.yaml_data, file)

            print("YAML file saved successfully.")
        except Exception as e:
            print(f"Error saving YAML file: {e}")

    def parse_float(self, value):
        """尝试将输入的字符串解析为浮动数字，失败时返回默认值0.0"""
        try:
            return float(value)
        except ValueError:
            return 0.0


class AutowareWindow(QMainWindow, Ui_autowareWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # 修改为指定的目录路径
        self.yaml_directory = '/home/username/yaml_files/'  # 这里需要指定YAML文件所在的目录
        self.yaml_files = self.load_yaml_files(self.yaml_directory)

        # 点击按钮打开子界面
        self.pushButton.clicked.connect(self.open_vehicle_mode_window)

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
        
        # 打开子界面并传入YAML数据
        self.vehicle_mode_window = VehicleModeWindow(yaml_data, first_yaml_file)
        self.vehicle_mode_window.show()  # 使用show()来显示主窗口


if __name__ == '__main__':
    app = QApplication(sys.argv)
    autoware_window = AutowareWindow()
    autoware_window.show()
    sys.exit(app.exec_())
