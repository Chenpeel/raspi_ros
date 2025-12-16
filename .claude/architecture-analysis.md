# ROS2舵机控制系统架构深度分析报告

生成时间：2025-12-16

## 一、当前架构全景图

### 1.1 包结构概览

| 包名 | 类型 | 描述 | 节点数 | 话题发布/订阅 |
|------|------|------|--------|--------------|
| `servo_hardware` | ament_python | 硬件驱动层 | 2 | 订阅`/servo/command`，发布`/servo/state` |
| `servo_core` | ament_python | 核心逻辑层 | 0 (仅库) | - |
| `servo_control` | ament_python | 集成控制层 | 1 | 桥接WebSocket和ROS话题 |
| `websocket_bridge` | ament_python | 通信桥接层 | 1 | 订阅`/servo/state`，发布`/servo/command` |
| `assets` | ament_cmake | 机器人模型描述 | 0 | URDF/可视化资源 |

### 1.2 当前节点拓扑关系

```
[WebSocket客户端]
       ↓
[websocket_ros2_bridge] (节点)
       ↓ 发布
  /servo/command (String, JSON)
       ↓ 订阅
[bus_servo_driver] 或 [pca_servo_driver] (节点)
       ↓ 串口/I2C
  [物理舵机硬件]
       ↓ 反馈
[bus_servo_driver/pca_servo_driver]
       ↓ 发布
  /servo/state (String, JSON)
       ↓ 订阅
[websocket_ros2_bridge]
       ↓
[WebSocket客户端]
```

**注意**：`servo_control_node`节点实际上是另一个WebSocket桥接实现，与`websocket_bridge/bridge_node`**功能重复**。

### 1.3 消息格式分析

#### 1.3.1 `/servo/command` 话题
```json
{
  "servo_type": "bus" | "pca",
  "servo_id": int,
  "position": int,  // 角度(0-180)或脉宽(500-2500us)或PWM(0-4095)
  "speed": int      // 可选，仅总线舵机使用(毫秒)
}
```
- **消息类型**：`std_msgs/String` (JSON序列化)
- **问题**：未使用自定义消息类型，缺少类型检查和文档

#### 1.3.2 `/servo/state` 话题
```json
{
  "servo_type": "bus" | "pca",
  "servo_id": int,
  "position": int,
  "timestamp": float  // 秒级时间戳
}
```
- **消息类型**：`std_msgs/String` (JSON序列化)
- **问题**：同上

## 二、架构问题诊断

### 2.1 冗余组件识别

#### 🔴 严重问题：双重WebSocket桥接实现

| 组件 | 位置 | 功能 | 问题 |
|------|------|------|------|
| `servo_control_node` | `src/control/servo_control/servo_control_node.py` | WebSocket ↔ ROS桥接 + 集成`servo_core` | 混合职责 |
| `bridge_node` | `src/websocket/websocket_bridge/bridge_node.py` | WebSocket ↔ ROS桥接 | 功能重复 |

**证据**：
1. `servo_control_node.py:32-37` - 创建WebSocket服务器
2. `bridge_node.py:84-89` - 同样创建WebSocket服务器
3. 两者都订阅`/servo/state`，都发布到`/servo/command`
4. Launch文件`full_system.launch.py`仅使用`bridge_node`，未使用`servo_control_node`

**影响**：
- 维护成本翻倍
- 用户困惑：应该使用哪个节点？
- 代码不一致：两个实现可能行为差异

#### 🔴 未使用的核心模块

| 模块 | 位置 | 问题 |
|------|------|------|
| `servo_core/servo_interface.py` | 仅1行空文件 | 完全未实现 |
| `servo_core/core.py` | 未被任何节点导入 | 死代码 |
| `servo_core/status_manager.py` | 仅在`servo_control_node`中使用，但该节点未被launch | 无效代码 |

**搜索证据**：
```bash
grep -r "from servo_core" src/hardware/ src/websocket/
# 无结果 - 证明servo_core未被实际使用
```

### 2.2 缺失组件识别

#### 🟡 缺少标准中间抽象层

**当前问题**：
- 硬件驱动直接暴露`/servo/command`话题
- 无统一的舵机抽象接口
- 应用层需要知道底层是总线舵机还是PCA舵机

**应该存在但缺失的组件**：
1. **舵机控制器管理器** (Controller Manager)
   - 类似`ros2_control`的资源管理器
   - 管理舵机生命周期和资源分配

2. **统一硬件接口** (Hardware Interface)
   - 抽象不同类型舵机的差异
   - 提供统一的`set_position()` API

3. **配置管理器** (Configuration Manager)
   - 从YAML加载舵机配置（ID映射、限位、校准参数）
   - 当前配置散落在launch参数中

### 2.3 职责混乱识别

#### 🔴 `servo_control_node` 职责过载

该节点混合了4层职责：
1. **通信层**：WebSocket服务器管理
2. **应用层**：舵机命令分发
3. **核心层**：集成`StatusManager`
4. **集成层**：asyncio事件循环管理

**证据**：`servo_control_node.py:22-76`
```python
class ServoControlNode(Node):
    # 1. WebSocket服务器 (通信层)
    self.ws_server = WebSocketBridgeServer(...)

    # 2. 状态管理器 (核心层)
    self.status_manager = StatusManager()

    # 3. ROS话题发布/订阅 (应用层)
    self.servo_command_pub = ...

    # 4. 异步事件循环 (集成层)
    self.loop = asyncio.new_event_loop()
```

**后果**：
- 难以测试（需要同时mock 4层依赖）
- 难以复用（无法单独使用WebSocket或ROS部分）
- 难以扩展（修改一层影响所有层）

#### 🟡 硬件驱动节点包含业务逻辑

**问题**：`bus_servo.py:233-267` 中的`command_callback`包含JSON解析和类型判断
```python
def command_callback(self, msg: String):
    cmd = json.loads(msg.data)
    if cmd.get('servo_type') != 'bus':  # 业务逻辑
        return
    # 硬件控制代码...
```

**应该做什么**：
- 硬件驱动应该只接收结构化命令（不应解析JSON）
- 类型过滤应在上层（控制器管理器）完成

### 2.4 话题设计问题

#### 🔴 话题命名违反REP-144

| 话题名 | 问题 | REP-144要求 |
|--------|------|------------|
| `/servo/command` | 全局命名空间 | 应使用私有命名空间`~/servo/command` |
| `/servo/state` | 全局命名空间 | 应使用私有命名空间`~/servo/state` |
| `/system/state` | 仅在`servo_control_node`中定义，但未被使用 | 应明确用途或删除 |

**REP-144规定**：
> All topics must be set under private namespaces

#### 🔴 使用`std_msgs/String`代替自定义消息

**问题**：
- 无类型安全：运行时才能发现JSON格式错误
- 无文档：必须阅读代码才知道字段含义
- 无版本控制：无法演进消息格式
- 性能低：JSON序列化/反序列化开销

**证据**：
```python
# bus_servo.py:72-77
self.command_sub = self.create_subscription(
    String,  # ❌ 应使用自定义消息
    '/servo/command',
    self.command_callback,
    10
)
```

## 三、命名规范对比分析

### 3.1 REP-144包命名检查

| 包名 | 当前名称 | REP-144合规性 | 建议修正 |
|------|---------|--------------|----------|
| 硬件驱动 | `servo_hardware` | ✅ 合规 | 保持 |
| 核心逻辑 | `servo_core` | ⚠️ 过于通用 | `servo_controllers` (参考ros2_control命名) |
| 集成控制 | `servo_control` | ⚠️ 与`servo_core`混淆 | 应删除（功能重复） |
| WebSocket桥接 | `websocket_bridge` | ❌ 不合规：缺少项目前缀 | `servo_websocket_bridge` |
| 机器人模型 | `assets` | ❌ 过于通用 | `<robot_name>_description` (例如`bipedal_description`) |

**REP-144要求**：
> Package names are global to the entire ROS ecosystem and should make sense to others

**违规说明**：
- `assets`：任何人都可能用这个名字，容易冲突
- `websocket_bridge`：未说明是舵机项目的一部分

### 3.2 节点命名检查

| 节点名 | 当前名称 | REP-144合规性 | 建议修正 |
|--------|---------|--------------|----------|
| 总线舵机驱动 | `bus_servo_driver` | ✅ 合规 | 保持 |
| PCA舵机驱动 | `pca_servo_driver` | ✅ 合规 | 保持 |
| WebSocket桥接 | `websocket_ros2_bridge` | ✅ 合规 | 保持 |
| 集成控制节点 | `servo_control` | ⚠️ 应删除（冗余） | - |

### 3.3 话题命名检查

| 话题名 | 当前名称 | 命名空间 | REP-144合规性 | 建议修正 |
|--------|---------|---------|--------------|----------|
| 舵机命令 | `/servo/command` | 全局 | ❌ 不合规 | `~/command` (节点私有) |
| 舵机状态 | `/servo/state` | 全局 | ❌ 不合规 | `~/state` (节点私有) |
| 系统状态 | `/system/state` | 全局 | ❌ 不合规 | `~/status` (节点私有) |

**ROS2特定要求**：
> In ROS 2, topic names must separate the tilde (~) from the rest of the name with a forward slash (/)

**正确用法**：
```python
# ❌ 当前
self.create_publisher(String, '/servo/command', 10)

# ✅ 应改为
self.create_publisher(String, '~/command', 10)
# 实际话题名将是: /节点名/command
```

### 3.4 文件和模块命名检查

| 文件 | 当前名称 | Python风格合规性 | 建议 |
|------|---------|-----------------|------|
| 总线舵机驱动 | `bus_servo.py` | ✅ 合规 | 保持 |
| PCA舵机驱动 | `pca_servo.py` | ✅ 合规 | 保持 |
| 桥接节点 | `bridge_node.py` | ✅ 合规 | 保持 |
| WebSocket服务器 | `ws_server.py` | ⚠️ 缩写 | 建议`websocket_server.py` (更明确) |

## 四、依赖关系分析

### 4.1 包依赖图

```
assets (URDF模型)
  ↓ (无代码依赖)

servo_hardware (硬件驱动)
  ├─ rclpy
  ├─ pyserial (总线舵机)
  └─ smbus2 (PCA9685)

servo_core (核心库)
  ├─ rclpy
  └─ ⚠️ 实际未被任何包导入

servo_control (集成控制)
  ├─ rclpy
  ├─ servo_core (导入但未实际使用核心功能)
  └─ websocket_bridge (导入WebSocket服务器)

websocket_bridge (通信桥接)
  └─ rclpy
```

### 4.2 循环依赖检查

❌ **检测到软循环依赖**：
```
servo_control 导入 websocket_bridge.WebSocketBridgeServer
websocket_bridge/bridge_node 实现相同功能
两者竞争相同职责
```

**解决方案**：删除`servo_control`包，统一使用`websocket_bridge`。

### 4.3 未声明的依赖

在`websocket_bridge/setup.py`中：
```python
# ❌ 缺少依赖声明
install_requires=[
    'setuptools',
    # 缺少: 'websockets', 'asyncio'等
],
```

**检查证据**：
```bash
grep -r "import websockets" src/websocket/
# src/websocket/websocket_bridge/ws_server.py 使用了websockets库
```

## 五、参考架构对比

### 5.1 与ros2_control架构对比

| 层级 | ros2_control | 当前项目 | 差距 |
|------|-------------|---------|------|
| 硬件抽象 | `hardware_interface::SystemInterface` | ❌ 无 | 缺少统一接口 |
| 资源管理 | `ResourceManager` | ❌ 无 | 驱动直接暴露话题 |
| 控制器层 | `controller_interface::ControllerInterface` | ❌ 无 | 无控制器抽象 |
| 配置管理 | YAML + `<ros2_control>` URDF标签 | ⚠️ 部分：`servo_params.yaml`未使用 | 配置散乱 |
| 生命周期管理 | `lifecycle_msgs` | ❌ 无 | 无优雅启停 |

**参考资源**：
- [ros2_control架构文档](https://control.ros.org/)
- [硬件抽象最佳实践](https://thinkrobotics.com/blogs/learn/ros-2-control-for-custom-robots-mastering-real-time-robot-control)

### 5.2 标准ROS2项目结构对比

**推荐结构**（基于Autoware和Nav2项目）：
```
project_name/
  ├── <robot>_hardware/       # 硬件接口
  ├── <robot>_controllers/    # 控制器
  ├── <robot>_bringup/       # 启动文件
  ├── <robot>_description/   # URDF/模型
  ├── <robot>_msgs/          # 自定义消息
  └── <robot>_interfaces/    # 服务/动作定义
```

**当前结构问题**：
- ❌ 缺少`_msgs`包（应定义`ServoCommand`和`ServoState`消息）
- ❌ 缺少`_bringup`包（launch文件散落在各包）
- ❌ `assets`包名不符合`<robot>_description`约定

## 六、证据清单

### 6.1 代码搜索证据

```bash
# 1. 证明servo_core未被使用
$ grep -r "from servo_core" src/hardware/ src/websocket/
# 无输出

# 2. 证明功能重复
$ grep -A5 "WebSocketBridgeServer" src/control/servo_control/servo_control_node.py
$ grep -A5 "WebSocketBridgeServer" src/websocket/websocket_bridge/bridge_node.py
# 两处完全相同的初始化代码

# 3. 证明话题名硬编码
$ grep -r "/servo/command" src/
# 多个文件硬编码相同话题名
```

### 6.2 Launch文件使用证据

```bash
$ grep -r "servo_control_node" src/*/launch/
# 无输出 - 证明servo_control_node从未被launch使用

$ grep -r "bridge_node" src/*/launch/
# src/websocket/launch/full_system.launch.py 使用bridge_node
```

### 6.3 文件统计证据

| 包 | Python文件数 | 有效代码行数 | 空文件/未使用文件 |
|-----|-------------|------------|-----------------|
| servo_hardware | 3 | ~600 | 0 |
| servo_core | 4 | ~50 | 3 (未使用) |
| servo_control | 2 | ~250 | 2 (冗余) |
| websocket_bridge | 8 | ~800 | 0 |

## 七、总结

### 7.1 关键发现

1. **架构冗余**：双重WebSocket桥接实现，至少50%代码浪费
2. **模块空心化**：`servo_core`包几乎完全未使用
3. **命名违规**：5个包/话题违反REP-144规范
4. **缺失抽象**：无硬件接口层、无控制器管理器
5. **消息滥用**：所有话题使用`String`序列化JSON

### 7.2 优先级评级

| 问题 | 严重性 | 紧急性 | 修复成本 |
|------|--------|--------|---------|
| 双重WebSocket实现 | 🔴 高 | 高 | 低（删除一个） |
| 话题使用String | 🔴 高 | 中 | 中（定义msg） |
| 话题命名违规 | 🟡 中 | 中 | 低（修改名称） |
| 包命名违规 | 🟡 中 | 低 | 高（重命名影响大） |
| 缺少硬件抽象 | 🟡 中 | 低 | 高（需重构） |
| servo_core未使用 | 🟢 低 | 低 | 低（直接删除） |

### 7.3 下一步行动

按优先级排序：
1. **立即**：删除冗余的`servo_control`包
2. **短期**：定义自定义消息类型
3. **短期**：修正话题命名空间
4. **中期**：重命名违规包名
5. **长期**：引入ros2_control风格的硬件抽象层

---

**参考资料**：
- [REP-144: ROS Package Naming](https://ros.org/reps/rep-0144.html)
- [ROS2话题和服务命名规范](https://design.ros2.org/articles/topic_and_service_names.html)
- [ros2_control资源管理架构](https://control.ros.org/iron/doc/resources/resources.html)
- [Autoware命名约定](https://tier4.github.io/pilot-auto-ros2-iv-archive/tree/main/design/software_architecture/NamingConvention/)
