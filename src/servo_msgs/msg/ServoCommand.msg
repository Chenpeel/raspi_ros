# 舵机控制命令消息
# 用于从控制层发送到驱动层

# 舵机类型
string servo_type  # "bus" 或 "pca"

# 舵机ID
uint16 servo_id  # 总线舵机: 0-200, PCA9685: 0-15

# 目标位置
uint16 position  # 角度(0-180) 或 脉宽(500-2500us) 或 PWM值(0-4095)

# 运动速度（可选，仅总线舵机）
uint16 speed  # 毫秒，0表示最快速度

# 时间戳
builtin_interfaces/Time stamp
