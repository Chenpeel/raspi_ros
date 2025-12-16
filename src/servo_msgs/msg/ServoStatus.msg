# 系统整体状态消息
# 用于状态查询和监控

# 设备ID
string device_id  # 设备唯一标识符

# 总线舵机状态列表
ServoState[] bus_servos

# PCA舵机状态列表
ServoState[] pca_servos

# 系统状态
string system_state  # "idle"=空闲, "running"=运行中, "error"=错误

# 时间戳
builtin_interfaces/Time stamp
