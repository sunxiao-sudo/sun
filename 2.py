def on_text_changed(self):
    """当文本修改时，更新对应的 YAML 文件中的值"""
    sender = self.sender()
    text = sender.toPlainText()

    # 不加双引号的特定键
    no_quotes_keys = {
        'min_longitudinal_offset', 'max_longitudinal_offset', 'x_stddev', 'right_overhang',
        'min_lateral_offset', 'max_lateral_offset', 'min_height_offset', 'max_height_offset',
        'simulated_frame_id', 'origin_frame_id', 'vehicle_model_type', 'initialize_source',
        'timer_sampling_time_ms', 'add_measurement_noise', 'vel_lim', 'vel_rate_lim',
        'steer_lim', 'steer_rate_lim', 'acc_time_delay', 'acc_time_constant', 'steer_time_delay',
        'steer_time_constant', 'wheel_radius', 'wheel_width', 'wheel_base', 'wheel_tread',
        'front_overhang', 'vehicle_height', 'rear_overhang', 'left_overhang', 'right_overhang',
        'max_steer_angle'
    }

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

            # 判断是否是数字或布尔值
            try:
                if text.lower() == 'true':
                    value = "true"  # 保持为字符串
                elif text.lower() == 'false':
                    value = "false"  # 保持为字符串
                elif text.isdigit():  # 处理整数
                    value = str(int(text))  # 保持为字符串
                else:
                    try:
                        value = str(float(text))  # 试图将文本转为浮动数值并转为字符串
                    except ValueError:
                        if key in no_quotes_keys:  # 如果是指定的不加双引号的键，保存为数字类型
                            value = float(text)
                        else:
                            value = f'"{text}"'  # 不是数字的话，直接作为字符串并加上双引号
            except Exception as e:
                print(f"Error converting text to appropriate type: {e}")
                value = f'"{text}"'  # 如果出现错误，直接作为字符串

            # 更新对应的 YAML 文件中的值
            if idx < 6:  # mirror.param.yaml
                if isinstance(value, str) and value not in ["true", "false"] and key not in no_quotes_keys:
                    value = f'"{value}"'  # 如果是字符串且不在 no_quotes_keys 中，添加双引号
                self.yaml_data_1['/**']['ros__parameters'][key] = value
                self.save_yaml('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/mirror.param.yaml', self.yaml_data_1)
            elif idx < 22:  # simulator_model.param.yaml
                if isinstance(value, str) and value not in ["true", "false"] and key not in no_quotes_keys:
                    value = f'"{value}"'  # 如果是字符串且不在 no_quotes_keys 中，添加双引号
                self.yaml_data_2['/**']['ros__parameters'][key] = value
                self.save_yaml('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/simulator_model.param.yaml', self.yaml_data_2)
            else:  # vehicle_info.param.yaml
                if isinstance(value, str) and value not in ["true", "false"] and key not in no_quotes_keys:
                    value = f'"{value}"'  # 如果是字符串且不在 no_quotes_keys 中，添加双引号
                self.yaml_data_3['/**']['ros__parameters'][key] = value
                self.save_yaml('/home/nvidia/code/kunyi/src/vehicle/carla_vehicle_launch/carla_vehicle_description/config/vehicle_info.param.yaml', self.yaml_data_3)
