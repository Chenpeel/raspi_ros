# 舵机状态反馈消息
# 用于从驱动层发送到控制层

# 舵机类型
string servo_type  # "bus" 或 "pca"

# 舵机ID
uint16 servo_id  # 总线舵机: 0-200, PCA9685: 0-15

# 当前位置
uint16 position  # 角度(0-180) 或 脉宽(500-2500us) 或 PWM值(0-4095)

# 当前负载（可选，仅支持的舵机类型）
int16 load  # -1表示不支持，其他值表示负载百分比

# 当前温度（可选，仅支持的舵机类型）
int16 temperature  # -1表示不支持，单位：摄氏度

# 错误码（0表示正常）
uint8 error_code  # 0=正常, 1=通信错误, 2=过载, 3=过热, 4=其他错误

# 时间戳
builtin_interfaces/Time stamp
