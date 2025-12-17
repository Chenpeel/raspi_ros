# IMU 传感器完整数据
string sensor_type           # "imu"
uint8 sensor_id              # 传感器ID (默认0)

# 加速度计 (单位: g)
float64 accel_x
float64 accel_y
float64 accel_z

# 陀螺仪 (单位: rad/s)
float64 gyro_x
float64 gyro_y
float64 gyro_z

# 磁力计 (单位: µT)
float64 mag_x
float64 mag_y
float64 mag_z

# 四元数 (归一化)
float64 quat_w
float64 quat_x
float64 quat_y
float64 quat_z

# 欧拉角 (单位: deg)
float64 roll
float64 pitch
float64 yaw

# 气压计
float64 baro_height         # m
float64 baro_temp           # °C
float64 baro_pressure       # Pa

# 错误状态
uint8 error_code            # 0=正常, 1=I2C错误, 2=超时, 3=初始化失败

# 时间戳
builtin_interfaces/Time stamp
