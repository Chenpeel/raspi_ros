# 3-DOF并联控制器 (3-DOF Parallel Controller)

基于3-DOF并联机构运动学的3-DOF并联控制器，将RPY姿态角转换为舵机控制命令。

## 功能

- 接收脚踝RPY姿态命令（roll, pitch, yaw）
- 使用3-DOF Delta并联机构运动学模型进行求解
- 生成舵机控制命令并发布

## 安装

### 1. 构建包

```bash
cd <仓库根目录>
colcon build --packages-select parallel_3dof_controller
source install/setup.bash
```

### 2. 依赖

- rclpy
- geometry_msgs
- std_msgs
- servo_msgs
- numpy

## 使用方法

### 1. 启动控制器

```bash
# 使用默认参数
ros2 launch parallel_3dof_controller parallel_3dof_controller.launch.py

# 控制左脚踝
ros2 launch parallel_3dof_controller parallel_3dof_controller.launch.py ankle_side:=left

# 启用调试模式
ros2 launch parallel_3dof_controller parallel_3dof_controller.launch.py debug:=true
```

### 2. 发送RPY命令

```bash
# 发送零姿态命令
ros2 topic pub /parallel_3dof_controller/ankle_rpy geometry_msgs/Vector3 "{x: 0.0, y: 0.0, z: 0.0}"

# 发送小角度姿态 (度)
ros2 topic pub /parallel_3dof_controller/ankle_rpy geometry_msgs/Vector3 "{x: 10.0, y: 10.0, z: 5.0}"

# 发送边界姿态
ros2 topic pub /parallel_3dof_controller/ankle_rpy geometry_msgs/Vector3 "{x: 30.0, y: 30.0, z: 0.0}"
```

### 3. 监控输出

```bash
# 查看舵机命令
ros2 topic echo /servo/command

# 查看theta角反馈
ros2 topic echo /parallel_3dof_controller/ankle_theta
```

## 话题

### 订阅

- `~/ankle_rpy` (geometry_msgs/Vector3)
  - 脚踝RPY姿态命令（单位：度）
  - x: roll（横滚角）
  - y: pitch（俯仰角）
  - z: yaw（偏航角）

### 发布

- `~/servo/command` (servo_msgs/ServoCommand)
  - 舵机控制命令
  - id: 舵机ID
  - position: 位置值（us）
  - speed: 运动时间（毫秒）

- `~/ankle_theta` (std_msgs/Float32MultiArray)
  - 三个连杆的theta角反馈（单位：度）
  - 用于调试和监控

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| l0 | float | 0.02 | 平台半径（米） |
| l1 | float | 0.01 | 动平台到O点距离（米） |
| l2 | float | 0.03 | 静平台到O点距离（米） |
| ankle_side | string | 'right' | 控制哪侧脚踝（'right'/'left'） |
| default_speed | int | 100 | 默认舵机速度（毫秒） |
| debug | bool | false | 是否打印调试信息 |

## 工作空间限制

默认工作空间范围：
- Roll: ±30° (-0.524 ~ +0.524 rad)
- Pitch: ±30° (-0.524 ~ +0.524 rad)
- Yaw: ±30° (-0.524 ~ +0.524 rad)

超出范围的命令会被自动限制在安全范围内。

## 运动学模型

本控制器基于3-DOF Delta型并联机构，核心特点：

1. **几何结构**：
   - 动平台和静平台各有3个等角分布的点（120°间隔）
   - 通过3根连杆连接
   - 满足垂直约束条件

2. **求解方法**：
   - 正向运动学：从RPY → theta角（连杆与β平面夹角）
   - 舵机映射：theta角 → 舵机位置值（线性映射）

3. **假设**：
   - 舵机角度直接对应theta角
   - theta范围：0 ~ π/2 (0° ~ 90°)
   - 舵机位置：500-2500us 对应 0-180°

## 调试

### 1. 测试运动学求解器

```bash
cd src/parallel_3dof_controller
python3 parallel_3dof_controller/kinematics_solver.py
```

### 2. 查看调试信息

启用debug模式后，节点会打印详细的运动学计算信息：
```bash
ros2 launch parallel_3dof_controller parallel_3dof_controller.launch.py debug:=true
```

### 3. 验证约束满足

检查运动学求解的约束误差：
- 约束满足：误差 < 1e-6
- 需要调整：误差 > 1e-4

## 故障排查

### 问题1：约束未完全满足

**现象**：测试时看到 "警告: 约束未完全满足"

**原因**：几何参数 l0, l1, l2 与实际机构不匹配

**解决**：
1. 测量实际机构的几何参数
2. 修改config/ankle_params.yaml
3. 重新启动节点

### 问题2：舵机运动异常

**现象**：舵机角度与预期不符

**原因**：舵机映射关系或舵机ID配置错误

**解决**：
1. 检查kinematics_solver.py中的servo_config
2. 确认舵机ID分配
3. 调整offset和direction参数

### 问题3：工作空间受限

**现象**：某些RPY命令无法达到

**原因**：超出机构物理限制

**解决**：
1. 检查工作空间限制
2. 调整RPY范围
3. 考虑机构设计改进

## 开发

### 项目结构

```
parallel_3dof_controller/
├── parallel_3dof_controller/
│   ├── __init__.py
│   ├── kinematics_solver.py  # 运动学求解器
│   ├── controller_node.py    # ROS 2节点
│   └── tdpm.py               # 3-DOF并联机构运动学库
├── config/
│   └── ankle_params.yaml     # 参数配置
├── launch/
│   └── parallel_3dof_controller.launch.py
├── test/
│   └── (测试代码)
├── package.xml
├── setup.py
└── setup.cfg
```

### 扩展功能

可以添加的功能：
1. **逆向运动学**：从theta角反推RPY
2. **轨迹规划**：平滑的RPY轨迹插值
3. **奇异性检测**：避免奇异位形
4. **自适应标定**：在线调整几何参数

## 参考资料

- `parallel_3dof_controller/kinematics_solver.py`：运动学求解与舵机映射实现
- `parallel_3dof_controller/tdpm.py`：并联机构运动学模型
- `config/ankle_params.yaml`：几何参数与运行配置

## 许可

MIT License
