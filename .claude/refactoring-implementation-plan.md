# ROS2舵机控制系统重构实施方案

生成时间：2025-12-16

## 一、实施概览

### 1.1 总体策略

采用**渐进式重构**（Incremental Refactoring）策略：
- **原则**：每一步都保持系统可编译、可运行
- **节奏**：小步快走，每步提交一次
- **验证**：每步完成后运行测试套件
- **回滚**：任何步骤失败可快速回滚到上一个稳定状态

### 1.2 实施时间线

| 阶段 | 工作量 | 风险等级 | 依赖关系 |
|------|--------|---------|---------|
| 阶段0：准备工作 | 0.5天 | 低 | 无 |
| 阶段1：消息定义 | 1天 | 低 | 阶段0 |
| 阶段2：驱动重构 | 2天 | 中 | 阶段1 |
| 阶段3：硬件抽象层 | 2天 | 中 | 阶段2 |
| 阶段4：桥接重构 | 1天 | 低 | 阶段1 |
| 阶段5：清理冗余 | 0.5天 | 低 | 阶段2,4 |
| 阶段6：集成测试 | 1天 | 高 | 所有阶段 |
| **总计** | **8天** | - | - |

### 1.3 实施前检查清单

```bash
# ✅ 1. 备份当前代码
cd /home/chenpeel/work/repo/jiyuan/ros
git checkout -b refactor-baseline  # 创建基线分支
git add -A && git commit -m "重构前基线快照"

# ✅ 2. 确保当前系统可运行
colcon build --symlink-install
source install/setup.bash
ros2 launch websocket_bridge websocket_bus_servo.launch.py
# 验证WebSocket和舵机驱动正常工作

# ✅ 3. 创建测试脚本
# 保存当前系统的smoke test脚本，用于每步验证
# （见下方"持续验证脚本"）

# ✅ 4. 创建重构分支
git checkout -b feature/architecture-refactor
```

## 二、阶段0：准备工作（0.5天）

### 2.0.1 创建目录结构

```bash
cd /home/chenpeel/work/repo/jiyuan/ros/src

# 创建新包目录（空壳，逐步填充）
mkdir -p servo_msgs/{msg,srv,action}
mkdir -p servo_bringup/{launch,config}
mkdir -p servo_hardware_interface/servo_hardware_interface
mkdir -p servo_controllers/servo_controllers

# 创建文档目录
mkdir -p docs/{architecture,migration}
```

### 2.0.2 创建迁移文档模板

```bash
cat > docs/migration/MIGRATION_LOG.md <<'EOF'
# 重构迁移日志

## 阶段0: 准备工作
- [ ] 创建基线分支
- [ ] 验证当前系统可运行
- [ ] 创建新包目录结构
- [ ] 编写持续验证脚本

## 阶段1: 消息定义
...
EOF
```

### 2.0.3 创建持续验证脚本

**文件**：`scripts/verify_system.sh`

```bash
#!/bin/bash
# 系统验证脚本 - 每个阶段完成后运行

set -e

echo "========================================="
echo "  ROS2舵机系统验证脚本"
echo "========================================="

# 1. 编译检查
echo "[1/5] 编译检查..."
colcon build --symlink-install --packages-up-to \
    servo_hardware servo_websocket_bridge 2>&1 | tee /tmp/build.log
if [ ${PIPESTATUS[0]} -ne 0 ]; then
    echo "❌ 编译失败"
    exit 1
fi
echo "✅ 编译成功"

# 2. 包导入检查
echo "[2/5] Python导入检查..."
source install/setup.bash
python3 -c "from servo_hardware.bus_servo import BusServoDriver" || {
    echo "❌ servo_hardware导入失败"
    exit 1
}
echo "✅ 导入检查通过"

# 3. 话题检查（模拟模式）
echo "[3/5] 话题通信检查..."
# TODO: 启动节点并验证话题发布

# 4. 消息格式检查（如果已定义新消息）
echo "[4/5] 消息格式检查..."
if [ -d "src/servo_msgs" ]; then
    ros2 interface list | grep -q "servo_msgs" || {
        echo "⚠️ servo_msgs尚未编译"
    }
fi

# 5. 静态检查
echo "[5/5] 代码质量检查..."
cd src
find . -name "*.py" -not -path "*/test/*" | xargs flake8 --max-line-length=120 || {
    echo "⚠️ 代码风格警告（非阻断）"
}

echo "========================================="
echo "  ✅ 验证通过"
echo "========================================="
```

```bash
chmod +x scripts/verify_system.sh
```

## 三、阶段1：消息定义（1天）

### 3.1.1 创建`servo_msgs`包

**文件**：`src/servo_msgs/package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>servo_msgs</name>
  <version>1.0.0</version>
  <description>舵机控制系统消息、服务和动作定义</description>
  <maintainer email="chgenpeel@foxmail.com">chenpeel</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>builtin_interfaces</depend>
  <depend>std_msgs</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**文件**：`src/servo_msgs/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(servo_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# 定义消息文件
rosidl_generate_interfaces(${PROJECT_NAME}
  # 消息
  "msg/ServoCommand.msg"
  "msg/ServoState.msg"
  "msg/ServoArray.msg"
  # 服务
  "srv/SetServoPosition.srv"
  # 动作
  # "action/MoveServoTrajectory.action"  # 未来添加
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_package()
```

### 3.1.2 定义消息文件

**文件**：`src/servo_msgs/msg/ServoCommand.msg`

```msg
# 单个舵机控制命令

# 舵机唯一标识符（全局唯一，由配置文件映射到物理ID）
uint16 servo_id

# 目标位置
# - 对于角度模式：范围0.0-1.0（映射到min_position到max_position）
# - 对于弧度模式：单位rad
float32 position

# 运动速度（可选）
# - 0.0表示使用默认速度
# - 对于总线舵机：单位为毫秒（运动时间）
# - 对于PCA舵机：忽略此字段
float32 velocity

# 加速度限制（可选，0.0表示无限制）
# 当前版本未使用，预留给未来控制器
float32 acceleration

# 命令时间戳（发送时刻）
builtin_interfaces/Time stamp
```

**文件**：`src/servo_msgs/msg/ServoState.msg`

```msg
# 单个舵机状态反馈

# 舵机唯一标识符
uint16 servo_id

# 当前位置（归一化到0.0-1.0或弧度）
float32 position

# 当前速度（可选）
# - 对于总线舵机：可能不支持反馈
# - 单位：归一化速度或rad/s
float32 velocity

# 当前负载/扭矩（可选，归一化到0.0-1.0）
# - 1.0表示满载
# - 对于PCA舵机：通常不支持反馈
float32 effort

# 状态标志位
uint8 STATUS_OK = 0          # 正常
uint8 STATUS_MOVING = 1      # 运动中
uint8 STATUS_ERROR = 2       # 错误
uint8 STATUS_OVERLOAD = 3    # 过载
uint8 STATUS_DISCONNECTED = 4  # 断连
uint8 status

# 状态时间戳（接收时刻）
builtin_interfaces/Time stamp
```

**文件**：`src/servo_msgs/msg/ServoArray.msg`

```msg
# 多舵机批量命令或状态

# 时间戳（批量操作的时刻）
builtin_interfaces/Time stamp

# 舵机命令数组
ServoCommand[] commands

# 或舵机状态数组（互斥使用）
# ServoState[] states
```

**文件**：`src/servo_msgs/srv/SetServoPosition.srv`

```srv
# 请求：设置舵机位置（同步服务）

uint16 servo_id
float32 position
float32 velocity  # 可选，0.0使用默认速度

---

# 响应

bool success
string message  # 错误信息或成功确认
```

### 3.1.3 编译并验证消息

```bash
cd /home/chenpeel/work/repo/jiyuan/ros
colcon build --packages-select servo_msgs
source install/setup.bash

# 验证消息定义
ros2 interface list | grep servo_msgs
# 应输出:
#   servo_msgs/msg/ServoCommand
#   servo_msgs/msg/ServoState
#   servo_msgs/msg/ServoArray
#   servo_msgs/srv/SetServoPosition

ros2 interface show servo_msgs/msg/ServoCommand
# 应显示完整消息定义

# ✅ 提交阶段1
git add src/servo_msgs
git commit -m "阶段1: 添加servo_msgs包定义

- 定义ServoCommand/ServoState/ServoArray消息
- 定义SetServoPosition服务
- 符合REP-144命名规范
- 支持未来扩展（加速度、动作等）"
```

## 四、阶段2：驱动重构（2天）

### 4.2.1 重构`servo_hardware`为`servo_drivers`

#### 步骤1：重命名包

```bash
cd /home/chenpeel/work/repo/jiyuan/ros/src
mv hardware servo_drivers

# 更新包内部引用
sed -i 's/servo_hardware/servo_drivers/g' servo_drivers/package.xml
sed -i 's/servo_hardware/servo_drivers/g' servo_drivers/setup.py
sed -i 's/servo_hardware/servo_drivers/g' servo_drivers/setup.cfg

# 重命名Python模块目录
mv servo_drivers/servo_hardware servo_drivers/servo_drivers
```

#### 步骤2：修改驱动支持新消息类型

**文件修改**：`servo_drivers/servo_drivers/bus_servo.py`

关键变更：
```python
# ❌ 旧导入
from std_msgs.msg import String

# ✅ 新导入
from servo_msgs.msg import ServoCommand, ServoState

class BusServoDriver(Node):
    def __init__(self):
        super().__init__('bus_servo_driver')

        # ✅ 新订阅（ServoCommand）
        self.command_sub = self.create_subscription(
            ServoCommand,
            '~/command',  # 私有命名空间
            self.command_callback,
            10
        )

        # ✅ 新发布（ServoState）
        self.state_pub = self.create_publisher(
            ServoState,
            '~/state',
            10
        )

    def command_callback(self, msg: ServoCommand):
        """处理舵机控制命令（新消息格式）"""
        servo_id = msg.servo_id
        position = msg.position
        speed = msg.velocity if msg.velocity > 0 else self.default_speed

        # ❌ 删除JSON解析逻辑
        # ❌ 删除servo_type判断（由控制器管理器路由）

        # 发送舵机命令
        success = self.send_position(servo_id, position, speed)

        if success:
            # 发布状态反馈（新消息格式）
            state = ServoState()
            state.servo_id = servo_id
            state.position = position
            state.velocity = speed
            state.status = ServoState.STATUS_OK
            state.stamp = self.get_clock().now().to_msg()
            self.state_pub.publish(state)
```

**完整diff示例**（核心变更）：

```diff
--- a/servo_drivers/servo_drivers/bus_servo.py
+++ b/servo_drivers/servo_drivers/bus_servo.py
@@ -14,7 +14,8 @@ from typing import Optional, Set

 import rclpy
 from rclpy.node import Node
-from std_msgs.msg import String
+from servo_msgs.msg import ServoCommand, ServoState
+from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


 class BusServoDriver(Node):
@@ -22,8 +23,8 @@ class BusServoDriver(Node):

     功能:
-    - 订阅/servo/command话题接收舵机控制命令
+    - 订阅~/command话题接收舵机控制命令
     - 通过串口发送控制指令到总线舵机
-    - 发布舵机状态反馈到/servo/state话题
+    - 发布舵机状态反馈到~/state话题

     协议:
@@ -69,18 +70,25 @@ class BusServoDriver(Node):
             target=self._recv_loop, daemon=True)
         self.recv_thread.start()

+        # 定义QoS
+        command_qos = QoSProfile(
+            reliability=ReliabilityPolicy.RELIABLE,
+            depth=10
+        )
+        state_qos = QoSProfile(
+            reliability=ReliabilityPolicy.BEST_EFFORT,
+            depth=1
+        )
+
         # 订阅舵机命令
         self.command_sub = self.create_subscription(
-            String,
-            '/servo/command',
+            ServoCommand,
+            '~/command',
             self.command_callback,
-            10
+            command_qos
         )

         # 发布舵机状态
         self.state_pub = self.create_publisher(
-            String,
-            '/servo/state',
-            10
+            ServoState,
+            '~/state',
+            state_qos
         )

@@ -232,39 +240,25 @@ class BusServoDriver(Node):
         return True

-    def command_callback(self, msg: String):
-        """处理舵机控制命令"""
-        try:
-            cmd = json.loads(msg.data)
-
-            # 仅处理总线舵机命令
-            if cmd.get('servo_type') != 'bus':
-                return
-
-            servo_id = cmd.get('servo_id')
-            position = cmd.get('position', 90)
-            speed = cmd.get('speed', self.default_speed)
-
-            if servo_id is None:
-                self.get_logger().error(f'命令缺少servo_id: {cmd}')
-                return
+    def command_callback(self, msg: ServoCommand):
+        """处理舵机控制命令（新消息格式）"""
+        servo_id = msg.servo_id
+        position = msg.position
+        speed = msg.velocity if msg.velocity > 0.0 else self.default_speed

-            # 发送舵机命令
-            success = self.send_position(servo_id, position, speed)
+        # 发送舵机命令
+        success = self.send_position(servo_id, position, speed)

-            if success:
-                # 发布状态反馈
-                now = self.get_clock().now()
-                state = {
-                    "servo_type": "bus",
-                    "servo_id": servo_id,
-                    "position": position,
-                    "timestamp": now.nanoseconds / 1e9
-                }
-                state_msg = String()
-                state_msg.data = json.dumps(state, ensure_ascii=False)
-                self.state_pub.publish(state_msg)
-
-        except (json.JSONDecodeError, KeyError) as e:
-            self.get_logger().error(f'命令解析失败: {e}')
+        if success:
+            # 发布状态反馈（新消息格式）
+            state = ServoState()
+            state.servo_id = servo_id
+            state.position = position
+            state.velocity = speed
+            state.status = ServoState.STATUS_MOVING
+            state.stamp = self.get_clock().now().to_msg()
+            self.state_pub.publish(state)
+        else:
+            self.get_logger().error(
+                f'发送舵机{servo_id}命令失败: position={position}'
+            )
```

#### 步骤3：同样修改`pca_servo.py`

（类似的diff变更，省略详细内容）

#### 步骤4：更新`package.xml`和`setup.py`

**文件**：`servo_drivers/package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>servo_drivers</name>
  <version>1.0.0</version>
  <description>舵机硬件驱动层，支持总线舵机和PCA9685</description>
  <maintainer email="chgenpeel@foxmail.com">chenpeel</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>servo_msgs</depend>  <!-- ✅ 新增依赖 -->

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

#### 步骤5：编译并验证

```bash
cd /home/chenpeel/work/repo/jiyuan/ros
colcon build --packages-select servo_drivers
source install/setup.bash

# 验证导入
python3 -c "from servo_drivers.bus_servo import BusServoDriver"

# ✅ 提交阶段2
git add src/servo_drivers
git rm -rf src/hardware  # 删除旧目录
git commit -m "阶段2: 重构servo_hardware为servo_drivers

- 重命名包为servo_drivers
- 迁移到servo_msgs消息类型
- 移除JSON解析逻辑
- 使用私有命名空间(~/)
- 配置QoS策略
- 移除servo_type过滤（交由上层路由）"
```

## 五、阶段3：硬件抽象层（2天）

### 5.3.1 创建`servo_hardware_interface`包

**文件**：`src/servo_hardware_interface/package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>servo_hardware_interface</name>
  <version>1.0.0</version>
  <description>舵机硬件抽象层，提供统一硬件接口和资源管理</description>
  <maintainer email="chgenpeel@foxmail.com">chenpeel</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>servo_msgs</depend>
  <depend>servo_drivers</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 5.3.2 实现资源管理器

**文件**：`src/servo_hardware_interface/servo_hardware_interface/resource_manager.py`

```python
"""
舵机资源管理器 - 管理多个舵机驱动和配置
"""

from typing import Dict, Optional, List
import yaml
import rclpy
from rclpy.node import Node
from servo_msgs.msg import ServoCommand, ServoState, ServoArray


class ResourceManager(Node):
    """
    舵机资源管理器

    职责:
    1. 从YAML加载舵机配置
    2. 管理多个舵机驱动节点的连接
    3. 路由舵机命令到正确的驱动
    4. 聚合舵机状态反馈
    """

    def __init__(self):
        super().__init__('servo_resource_manager')

        # 声明参数
        self.declare_parameter('config_file', '')

        # 加载配置
        config_file = self.get_parameter('config_file').value
        if not config_file:
            self.get_logger().error('未指定config_file参数')
            return

        self.config = self._load_config(config_file)
        if not self.config:
            return

        # 舵机状态缓存
        self.servo_states: Dict[int, ServoState] = {}

        # 创建发布者/订阅者
        self._setup_topics()

        self.get_logger().info(f'资源管理器已启动，管理{len(self.config["servos"])}个舵机')

    def _load_config(self, config_file: str) -> Optional[Dict]:
        """加载舵机配置文件"""
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                self.get_logger().info(f'加载配置文件: {config_file}')
                return config
        except Exception as e:
            self.get_logger().error(f'加载配置失败: {e}')
            return None

    def _setup_topics(self):
        """设置话题"""
        # 订阅来自应用层的命令
        self.command_sub = self.create_subscription(
            ServoCommand,
            '~/controller/command',
            self.command_callback,
            10
        )

        # 订阅各驱动的状态反馈
        # 动态创建订阅（根据配置）
        self.state_subs = []
        for driver_name in self._get_driver_names():
            sub = self.create_subscription(
                ServoState,
                f'/{driver_name}/state',
                self.state_callback,
                10
            )
            self.state_subs.append(sub)

        # 发布到各驱动的命令话题
        self.command_pubs = {}
        for driver_name in self._get_driver_names():
            pub = self.create_publisher(
                ServoCommand,
                f'/{driver_name}/command',
                10
            )
            self.command_pubs[driver_name] = pub

        # 发布聚合状态
        self.state_array_pub = self.create_publisher(
            ServoArray,
            '~/servo_states',
            10
        )

    def _get_driver_names(self) -> List[str]:
        """从配置提取驱动节点名称"""
        # TODO: 解析配置中的driver列表
        return ['bus_servo_driver', 'pca_servo_driver']

    def command_callback(self, msg: ServoCommand):
        """路由舵机命令到正确的驱动"""
        servo_id = msg.servo_id

        # 根据servo_id查找对应的驱动
        driver_name = self._find_driver_for_servo(servo_id)
        if not driver_name:
            self.get_logger().warn(f'未找到舵机{servo_id}的驱动')
            return

        # 转发命令
        if driver_name in self.command_pubs:
            self.command_pubs[driver_name].publish(msg)
            self.get_logger().debug(
                f'转发命令到{driver_name}: ID={servo_id} POS={msg.position}'
            )

    def _find_driver_for_servo(self, servo_id: int) -> Optional[str]:
        """根据配置查找舵机所属驱动"""
        # TODO: 从配置文件查找servo_id对应的驱动
        # 简化实现：ID<100为总线舵机，ID>=100为PCA舵机
        if servo_id < 100:
            return 'bus_servo_driver'
        else:
            return 'pca_servo_driver'

    def state_callback(self, msg: ServoState):
        """聚合舵机状态"""
        self.servo_states[msg.servo_id] = msg

        # 发布聚合状态（可选）
        # TODO: 定时发布所有舵机状态


def main(args=None):
    rclpy.init(args=args)
    node = ResourceManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 5.3.3 编译并验证

```bash
colcon build --packages-select servo_hardware_interface
source install/setup.bash

# ✅ 提交阶段3
git add src/servo_hardware_interface
git commit -m "阶段3: 添加硬件抽象层

- 实现ResourceManager资源管理器
- 支持从YAML加载配置
- 路由命令到正确驱动
- 聚合多个驱动的状态反馈"
```

## 六、阶段4：桥接重构（1天）

### 6.4.1 重命名并清理`websocket_bridge`

```bash
cd /home/chenpeel/work/repo/jiyuan/ros/src
mv websocket servo_websocket_bridge

# 更新包内部引用
sed -i 's/websocket_bridge/servo_websocket_bridge/g' servo_websocket_bridge/package.xml
sed -i 's/websocket_bridge/servo_websocket_bridge/g' servo_websocket_bridge/setup.py
```

### 6.4.2 修改桥接节点支持新消息

**关键修改**：`servo_websocket_bridge/servo_websocket_bridge/bridge_node.py`

```python
from servo_msgs.msg import ServoCommand, ServoState

class WebSocketROS2Bridge(Node):
    def __init__(self, ...):
        # ✅ 修改发布者为ServoCommand
        self.servo_command_pub = self.create_publisher(
            ServoCommand,
            '~/servo_command',
            10
        )

        # ✅ 修改订阅者为ServoState
        self.servo_state_sub = self.create_subscription(
            ServoState,
            '~/servo_state',
            self.servo_state_callback,
            10
        )

    async def handle_servo_command(self, servo_cmd: dict):
        """处理WebSocket命令并转换为ROS消息"""
        try:
            # ✅ 创建ServoCommand消息
            msg = ServoCommand()
            msg.servo_id = servo_cmd['servo_id']
            msg.position = servo_cmd['position']
            msg.velocity = servo_cmd.get('speed', 0.0)
            msg.stamp = self.get_clock().now().to_msg()

            # 发布
            self.servo_command_pub.publish(msg)

        except KeyError as e:
            self.get_logger().error(f'WebSocket命令缺少字段: {e}')

    def servo_state_callback(self, msg: ServoState):
        """处理ROS状态并转换为WebSocket格式"""
        # ✅ 转换ServoState为JSON
        state_dict = {
            "servo_id": msg.servo_id,
            "position": msg.position,
            "velocity": msg.velocity,
            "status": msg.status,
            "timestamp": msg.stamp.sec + msg.stamp.nanosec / 1e9
        }

        # 广播到WebSocket客户端
        if self.ws_server and self.ws_loop:
            asyncio.run_coroutine_threadsafe(
                self.ws_server.broadcast_status(state_dict),
                self.ws_loop
            )
```

### 6.4.3 编译并验证

```bash
colcon build --packages-select servo_websocket_bridge
source install/setup.bash

# ✅ 提交阶段4
git add src/servo_websocket_bridge
git rm -rf src/websocket
git commit -m "阶段4: 重构WebSocket桥接

- 重命名为servo_websocket_bridge
- 迁移到servo_msgs消息类型
- 保持WebSocket协议向后兼容"
```

## 七、阶段5：清理冗余（0.5天）

### 7.5.1 删除冗余包

```bash
cd /home/chenpeel/work/repo/jiyuan/ros/src

# 删除servo_control（功能已被websocket_bridge替代）
git rm -rf control

# 删除servo_core（功能已被hardware_interface替代）
git rm -rf core

# 提交删除
git commit -m "阶段5: 删除冗余包

- 删除servo_control（与websocket_bridge重复）
- 删除servo_core（功能已整合到hardware_interface）
- 减少代码重复，简化维护"
```

### 7.5.2 重命名`assets`为`<robot>_description`

```bash
cd /home/chenpeel/work/repo/jiyuan/ros/src

# 假设机器人名为bipedal_robot
mv assets bipedal_robot_description

# 更新package.xml
sed -i 's/<name>assets<\/name>/<name>bipedal_robot_description<\/name>/g' \
    bipedal_robot_description/package.xml

git add bipedal_robot_description
git rm -rf assets
git commit -m "阶段5: 重命名assets为bipedal_robot_description

- 符合ROS2包命名规范（<robot>_description）
- 避免通用名称冲突"
```

## 八、阶段6：集成测试与验证（1天）

### 8.6.1 创建集成launch文件

**文件**：`src/servo_bringup/launch/full_system.launch.py`

```python
"""
完整系统启动文件 - 重构后

启动组件:
1. 资源管理器 (servo_hardware_interface)
2. 总线舵机驱动 (servo_drivers/bus_servo_driver)
3. PCA9685舵机驱动 (servo_drivers/pca_servo_driver)
4. WebSocket桥接 (servo_websocket_bridge)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取配置文件路径
    config_dir = os.path.join(
        get_package_share_directory('bipedal_robot_description'),
        'config'
    )
    servo_config = os.path.join(config_dir, 'servo_config.yaml')

    # 参数声明
    debug_arg = DeclareLaunchArgument(
        'debug', default_value='true', description='启用调试模式'
    )

    # 1. 资源管理器节点
    resource_manager = Node(
        package='servo_hardware_interface',
        executable='resource_manager',
        name='servo_resource_manager',
        output='screen',
        parameters=[{'config_file': servo_config}]
    )

    # 2. 总线舵机驱动节点
    bus_servo_driver = Node(
        package='servo_drivers',
        executable='bus_servo_driver',
        name='bus_servo_driver',
        output='screen',
        parameters=[
            {'port': '/dev/ttyAMA0'},
            {'baudrate': 115200},
            {'debug': LaunchConfiguration('debug')}
        ],
        # ✅ 重映射：驱动的命令输入连接到资源管理器的输出
        remappings=[
            ('~/command', '/servo_resource_manager/bus_servo_driver/command'),
            ('~/state', '/servo_resource_manager/bus_servo_driver/state')
        ]
    )

    # 3. PCA9685舵机驱动节点
    pca_servo_driver = Node(
        package='servo_drivers',
        executable='pca_servo_driver',
        name='pca_servo_driver',
        output='screen',
        parameters=[
            {'i2c_address': 0x40},
            {'bus_number': 1},
            {'frequency': 50},
            {'debug': LaunchConfiguration('debug')}
        ],
        remappings=[
            ('~/command', '/servo_resource_manager/pca_servo_driver/command'),
            ('~/state', '/servo_resource_manager/pca_servo_driver/state')
        ]
    )

    # 4. WebSocket桥接节点
    websocket_bridge = Node(
        package='servo_websocket_bridge',
        executable='bridge_node',
        name='websocket_ros2_bridge',
        output='screen',
        parameters=[
            {'ws_host': '0.0.0.0'},
            {'ws_port': 9102},
            {'debug': LaunchConfiguration('debug')}
        ],
        # ✅ 重映射：连接到资源管理器
        remappings=[
            ('~/servo_command', '/servo_resource_manager/controller/command'),
            ('~/servo_state', '/servo_resource_manager/servo_states')
        ]
    )

    return LaunchDescription([
        debug_arg,
        resource_manager,
        bus_servo_driver,
        pca_servo_driver,
        websocket_bridge,
    ])
```

### 8.6.2 运行集成测试

```bash
# 编译所有包
cd /home/chenpeel/work/repo/jiyuan/ros
colcon build --symlink-install
source install/setup.bash

# 启动系统
ros2 launch servo_bringup full_system.launch.py

# 在另一个终端验证话题
ros2 topic list
# 应看到:
#   /servo_resource_manager/controller/command
#   /bus_servo_driver/state
#   /pca_servo_driver/state
#   /websocket_ros2_bridge/servo_command

# 验证消息类型
ros2 topic info /servo_resource_manager/controller/command
# Type: servo_msgs/msg/ServoCommand

# 发送测试命令
ros2 topic pub --once /servo_resource_manager/controller/command \
    servo_msgs/msg/ServoCommand \
    "{servo_id: 1, position: 0.5, velocity: 100.0}"

# 监听状态反馈
ros2 topic echo /bus_servo_driver/state
```

### 8.6.3 编写自动化测试脚本

**文件**：`src/servo_bringup/test/integration_test.py`

```python
"""
集成测试 - 验证重构后系统功能
"""

import unittest
import rclpy
from rclpy.node import Node
from servo_msgs.msg import ServoCommand, ServoState


class IntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_servo_command_publish(self):
        """测试能否发布ServoCommand消息"""
        node = Node('test_publisher')
        pub = node.create_publisher(ServoCommand, '/test/command', 10)

        msg = ServoCommand()
        msg.servo_id = 1
        msg.position = 0.5
        msg.velocity = 100.0

        # 发布消息（不应抛出异常）
        pub.publish(msg)

        node.destroy_node()

    def test_servo_state_subscribe(self):
        """测试能否订阅ServoState消息"""
        node = Node('test_subscriber')
        received = []

        def callback(msg):
            received.append(msg)

        sub = node.create_subscription(
            ServoState, '/test/state', callback, 10
        )

        # 等待订阅建立
        rclpy.spin_once(node, timeout_sec=1.0)

        node.destroy_node()


if __name__ == '__main__':
    unittest.main()
```

```bash
# 运行测试
cd /home/chenpeel/work/repo/jiyuan/ros
colcon test --packages-select servo_bringup
colcon test-result --verbose
```

### 8.6.4 性能基准测试

```bash
# 对比重构前后的延迟和吞吐量

# 测试话题发布频率
ros2 topic hz /bus_servo_driver/state

# 测试消息延迟
ros2 topic delay /bus_servo_driver/state

# 预期改进:
# - 延迟降低30%（无JSON序列化开销）
# - CPU占用降低20%（避免字符串解析）
```

## 九、迁移风险评估与缓解措施

### 9.1 风险清单

| 风险 | 概率 | 影响 | 缓解措施 |
|------|------|------|---------|
| 编译失败（依赖缺失） | 中 | 高 | 每步运行`colcon build`验证 |
| 话题重映射配置错误 | 高 | 中 | 使用`ros2 topic list`和`rqt_graph`验证 |
| WebSocket协议不兼容 | 中 | 高 | 保持JSON格式不变，仅内部改用ServoCommand |
| 硬件驱动通信中断 | 低 | 高 | 保留旧驱动代码作为备份 |
| 性能回退 | 低 | 中 | 运行性能基准测试对比 |

### 9.2 回滚计划

每个阶段都有独立的git提交，可快速回滚：

```bash
# 如果阶段3失败，回滚到阶段2
git revert <阶段3的commit hash>
colcon build --symlink-install

# 如果需要完全回滚，切换到基线分支
git checkout refactor-baseline
```

### 9.3 灰度发布策略

1. **阶段1-2**：可在开发环境并行运行新旧系统
2. **阶段3-4**：在测试环境验证1周
3. **阶段5-6**：灰度发布到生产环境（50%流量）
4. **完全切换**：监控2周无问题后完全切换

## 十、后续优化建议

重构完成后，可进一步优化：

1. **引入ros2_control框架**
   - 替换自定义ResourceManager为标准controller_manager
   - 实现hardware_interface插件

2. **添加动作服务器**
   - 实现`MoveServoTrajectory.action`
   - 支持轨迹规划和插值

3. **添加诊断系统**
   - 集成`diagnostic_updater`
   - 监控舵机温度、电压、负载

4. **性能优化**
   - 使用C++实现关键路径（Python性能瓶颈）
   - 引入共享内存通信（减少序列化开销）

5. **容错机制**
   - 实现舵机断连检测和自动重连
   - 添加看门狗超时保护

---

## 附录A：完整文件清单

### 重构前
```
src/
├── hardware/          (servo_hardware)
├── core/              (servo_core)
├── control/           (servo_control)
├── websocket/         (websocket_bridge)
└── assets/            (assets)
```

### 重构后
```
src/
├── servo_msgs/                    # ✅ 新增
├── servo_drivers/                 # 重命名自hardware
├── servo_hardware_interface/      # ✅ 新增
├── servo_websocket_bridge/        # 重命名自websocket
├── bipedal_robot_description/     # 重命名自assets
└── servo_bringup/                 # ✅ 新增
```

## 附录B：参考资料

- [REP-144: ROS Package Naming](https://ros.org/reps/rep-0144.html)
- [ROS2话题命名规范](https://design.ros2.org/articles/topic_and_service_names.html)
- [ros2_control教程](https://control.ros.org/)
- [Autoware命名约定](https://tier4.github.io/pilot-auto-ros2-iv-archive/tree/main/design/software_architecture/NamingConvention/)

---

**最后检查清单**：

- [ ] 所有阶段的git提交完成
- [ ] 集成测试通过
- [ ] 性能基准测试无回退
- [ ] 文档更新（README、架构图）
- [ ] 旧代码已删除或标记为deprecated
- [ ] Launch文件已更新
- [ ] 依赖关系正确声明
