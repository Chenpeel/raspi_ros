# ROS 仓库重构实施方案

## 1. 文档目标

本方案用于把当前 ROS 2 仓库从“功能可用但边界混杂”的状态，
重构为“模块职责清晰、接口稳定、可逐步演进到 C++ 后端”的运行时工
程。

本文不是概念草图，而是后续实施时必须遵守的约束文档。重点解决四个问
题：

1. 现有 ROS 包职责混杂，目录无法表达真实边界。
2. WebSocket、仿真、硬件驱动、控制逻辑在启动链路中耦合过深。
3. 上层模块仍在直接依赖 `servo_msgs` 和硬件级话题。
4. 后续如果要将高频 Python 节点迁移到 C++，当前结构会导致接口和启
   动方式一起震荡。


## 2. 设计总原则

### 2.1 系统边界

- 本仓库只承载 ROS 内部实时链路、设备链路和仿真链路。
- 外部任务服务不属于本仓库。
- 所有正式外部任务请求只能通过 `task_service_bridge` 进入。
- 人工遥控、演示、调试入口只能通过 `teleoperation_bridge` 进入。

### 2.2 分层原则

- 桥接层负责协议接入和对外适配。
- 控制层负责运动学、轨迹、约束、闭环控制。
- 执行层负责执行仲裁、安全保护、驱动能力映射。
- 驱动层负责端口、寄存器、协议、硬件状态。
- 接口层负责跨模块消息、服务、动作定义。

### 2.3 重构原则

- 先稳住公共命名和接口，再移动代码。
- 先拆职责，再谈语言重写。
- 先引入中间适配层，再移除旧耦合点。
- 迁移过程中允许临时兼容，但不允许把兼容代码当长期架构。


## 3. 公共术语

以下名称为正式公共名称，后续目录、包名、文档标题、接口名都以此为
准：

- `motion_control`：运动控制层
- `speech_interface`：语音输入输出层
- `vision_perception`：视觉感知层
- `execution_manager`：执行协调与安全仲裁层
- `task_service_bridge`：外部任务服务桥接层
- `teleoperation_bridge`：遥控与调试桥接层
- `servo_hardware`：舵机驱动层
- `sensor_hardware`：传感器驱动层
- `simulation_bridge`：仿真桥接层

命名约束：

- 公共名称只能有一个正式写法。
- 历史名称只允许出现在迁移说明中。
- 算法缩写只能用于内部实现类，不能作为系统模块名。


## 4. 当前仓库事实

当前仓库中已经存在的主要 ROS 包和真实职责如下：

| 当前 ROS 包 | 构建类型 | 当前主要职责 | 当前问题 | 目标模块 |
| --- | --- | --- | --- | --- |
| `websocket_bridge` | `ament_python` | WebSocket 接入、直接发舵机命令、状态回传、IMU 调试、BVH 演示、Isaac 仿真桥启动入口 | 遥控、调试、仿真职责混在一个包里 | `teleoperation_bridge` + `simulation_bridge` |
| `parallel_3dof_controller` | `ament_python` | 并联机构运动学求解、直接发布 `ServoCommand` | 属于控制原型，但仍直接依赖驱动接口 | `motion_control` |
| `servo_hardware` | `ament_python` | 总线舵机、PCA、协议路由、IMU 驱动 | 一个 ROS 包同时承载舵机与传感器 | `servo_hardware` + `sensor_hardware` |
| `sim_servo_bridge_cpp` | `ament_cmake` / `rclcpp` | 仿真关节命令和 `servo_msgs` 互转 | 命名仍是历史名，且未纳入统一桥接层 | `simulation_bridge` |
| `servo_msgs` | `ament_cmake` | 驱动级消息和服务 | 边界基本正确 | `servo_msgs` |
| `record_load_action` | `ament_python` | BVH 等动作资源和播放器 | 应降级为工具或演示资源，不进入正式主链路 | `tools/record_load_action` |
| `robot_description` | `ament_cmake` | URDF、mesh、展示 | 边界基本正确 | `description/robot_description` |
| `mjc_viewer` | `ament_cmake` / `rclcpp` | MuJoCo 视图和仿真展示 | 属于描述/仿真辅助，不应参与正式执行链路 | `description/mjc_viewer` |

现阶段最核心的结构问题有三类：

1. `websocket_bridge` 既是人工入口，又承担仿真桥接，还直接发布
   `ServoCommand`。
2. `parallel_3dof_controller` 和 `websocket_bridge` 都在绕过执行层，
   直接碰驱动接口。
3. `servo_hardware` 包内部同时导出舵机和 IMU 节点，导致执行层与传
   感层边界不成立。


## 5. 目标架构

### 5.1 正式任务链路

```text
external task service
    ->
task_service_bridge
    ->
motion_control
    ->
execution_manager
    ->
servo_hardware / sensor_hardware
    ->
physical hardware
```

### 5.2 遥控链路

```text
operator tool
    ->
teleoperation_bridge
    ->
execution_manager
    ->
servo_hardware / sensor_hardware
```

### 5.3 仿真链路

```text
motion_control / execution_manager
    <->
simulation_bridge
    <->
simulator
```

### 5.4 感知和语音链路

```text
microphone -> speech_interface -> task_service_bridge
camera     -> vision_perception -> task_service_bridge / motion_control
```

### 5.5 反馈链路

```text
servo_hardware / sensor_hardware
    ->
execution_manager
    ->
motion_control
    ->
task_service_bridge
    ->
external task service
```


## 6. 目录与包策略

### 6.1 目标目录树

```text
src/
├── interfaces/
│   ├── task_api_msgs/
│   ├── motion_msgs/
│   ├── speech_msgs/
│   ├── perception_msgs/
│   └── servo_msgs/
├── control/
│   └── motion_control/
├── speech/
│   └── speech_interface/
├── perception/
│   └── vision_perception/
├── execution/
│   ├── execution_manager/
│   ├── servo_hardware/
│   └── sensor_hardware/
├── bridges/
│   ├── task_service_bridge/
│   ├── teleoperation_bridge/
│   └── simulation_bridge/
├── description/
│   ├── robot_description/
│   └── mjc_viewer/
└── tools/
    └── record_load_action/
```

### 6.2 目录与 ROS 包的关系

必须明确两层概念：

1. 一级目录用于表达职责域，例如 `control/`、`execution/`。
2. ROS 包名用于表达可构建、可安装、可被 launch 引用的边界。

因此：

- Phase 1 允许先移动目录，再逐步收敛包名。
- 在目录调整阶段，不要求所有 `package.xml` 立即改名。
- 但新增包必须直接使用目标正式名称，不再继续引入历史包名。

### 6.3 包级迁移矩阵

| 当前状态 | 第一阶段动作 | 第二阶段动作 | 最终状态 |
| --- | --- | --- | --- |
| `src/websocket` / `websocket_bridge` | 移入 `src/bridges/teleoperation_bridge`，保持包名不变 | 拆出仿真相关节点和 launch | `teleoperation_bridge` 与 `simulation_bridge` 分离 |
| `src/hardware` / `servo_hardware` | 移入 `src/execution/servo_hardware`，保持包名不变 | 从中抽离 `sensor_hardware` 独立 ROS 包 | 舵机与传感器分包 |
| `src/parallel_3dof_controller` | 移入 `src/control/parallel_3dof_controller` 或直接新建 `motion_control` 包承接 | 通过 `motion_msgs` 接入执行层 | 形成 `motion_control` |
| `src/sim_servo_bridge_cpp` | 先移入 `src/bridges/simulation_bridge_cpp` 或保留原位但文档归入仿真域 | 新建统一 `simulation_bridge` 包，收口 launch 与接口 | 仿真桥职责统一 |
| `src/record_load_action` | 移入 `src/tools/record_load_action` | 从正式链路依赖中剥离，只保留 teleop/demo 可选依赖 | 工具资源包 |


## 7. 模块职责与边界

### 7.1 motion_control

职责：

- 接收任务级执行目标或姿态目标。
- 执行逆解、轨迹生成、约束检查和闭环控制。
- 向 `execution_manager` 输出运动级命令，而不是驱动级命令。
- 接收执行反馈并产出任务执行状态。

不负责：

- 不直接读写串口、I2C 或总线协议。
- 不直接发布 `ServoCommand`。
- 不接入外部任务服务协议。

### 7.2 execution_manager

职责：

- 作为控制层和驱动层之间的唯一正式边界。
- 校验来自 `motion_control` 或 `teleoperation_bridge` 的执行请求。
- 执行控制权仲裁、急停、超时、模式切换、安全保护。
- 将高层命令映射为 `servo_msgs` 或其它驱动级接口。

不负责：

- 不做任务语义理解。
- 不做感知推理。
- 不承载外部协议解析。

### 7.3 servo_hardware / sensor_hardware

职责：

- 面向硬件端口和驱动协议。
- 输出最小可用状态和诊断信息。
- 对端口、校准、协议错误、连接状态负责。

不负责：

- 不理解任务语义。
- 不做轨迹规划。
- 不暴露正式外部 API。

### 7.4 task_service_bridge

职责：

- 作为外部任务服务唯一正式入口。
- 将外部任务请求转换为 ROS 内部任务执行请求。
- 将状态、对话和感知摘要返回外部任务服务。

不负责：

- 不直接发布 `ServoCommand`。
- 不访问硬件协议实现。
- 不绕过 `motion_control` 和 `execution_manager`。

### 7.5 teleoperation_bridge

职责：

- 提供人工遥控、调试和演示入口。
- 将人工输入转换为内部统一控制接口。
- 返回操作态状态和调试信息。

不负责：

- 不作为正式生产任务入口。
- 不长期占有主控制权。
- 不维护执行安全规则。

### 7.6 simulation_bridge

职责：

- 连接内部运行时与仿真器。
- 对仿真命令、仿真状态和 replay 验证负责。
- 为控制算法和执行链路提供仿真替身。

不负责：

- 不直接控制生产硬件。
- 不承担外部任务服务入口职责。
- 不承载业务语义。


## 8. 接口分层

### 8.1 外部接口包

仅供外部任务服务和正式任务入口使用：

- `task_api_msgs`

建议最少定义：

- `ExecuteTask.action`
- `TaskStatus.msg`
- `DialogueReply.msg`
- `TaskContextSignal.msg`

约束：

- 只表达任务语义。
- 必须支持长时执行、反馈、取消。
- 必须带 `task_id`、`trace_id`、`session_id` 等追踪字段。

### 8.2 内部控制接口包

- `motion_msgs`
- `speech_msgs`
- `perception_msgs`

约束：

- 强结构化表达，不依赖自由文本解析。
- 控制接口必须能表达 `status`、`reason`、`recoverable`。
- 不把长文本上下文、记忆原文、外部协议字段塞入内部消息。

### 8.3 驱动接口包

- `servo_msgs`

适用范围：

- `execution_manager`
- `servo_hardware`
- `sensor_hardware`
- `simulation_bridge` 与驱动兼容层

禁止：

- `task_service_bridge` 直接依赖驱动接口。
- `motion_control` 长期直接发布 `ServoCommand`。


## 9. Topic / Service / Action 规划

### 9.1 Topic

- `/speech/transcript`
- `/speech/synthesis/request`
- `/perception/object_list`
- `/perception/scene_state`
- `/motion/state_feedback`
- `/execution/state_feedback`
- `/execution/result`
- `/hardware/servo/state`
- `/hardware/sensor/imu`

### 9.2 Action

- `/task/execute`
- `/motion/execute`

### 9.3 Service

- 设备诊断
- 舵机位置读取
- 协议探测
- 校准接口
- 控制权查询或切换

### 9.4 统一约束

- 长时任务必须优先使用 `Action`。
- 高频状态必须优先使用 `Topic`。
- 调试查询使用 `Service`。
- 不允许通过调试入口绕过正式任务链路。


## 10. 执行仲裁规则

当前仓库缺少一个明确的控制权持有者，因此必须把仲裁规则写入主文档。

### 10.1 仲裁归属

- `execution_manager` 是唯一控制权仲裁者。
- `servo_hardware` 不做任务来源判断。
- `teleoperation_bridge` 只负责申请人工控制，不负责执行裁决。
- `task_service_bridge` 只负责申请正式任务执行，不负责执行裁决。

### 10.2 最小控制状态机

执行层至少支持以下模式：

- `idle`：空闲，无主动控制源
- `task_active`：正式任务执行中
- `teleop_active`：人工遥控已获得控制权
- `estop`：急停锁定

### 10.3 仲裁规则

1. 正式任务默认优先级高于普通调试请求。
2. 人工遥控如果要接管，必须通过显式申请进入 `teleop_active`。
3. `estop` 触发后，任何来源的执行命令都必须被拒绝。
4. 仿真桥不参与控制权竞争，只负责替换执行目标端。

### 10.4 迁移要求

在 `execution_manager` 落地前：

- 可以保留旧的 `/servo/command` 通路用于兼容。
- 但所有新增控制节点不得再直接设计成长期发布 `ServoCommand`。
- 兼容通路必须被标注为过渡态，后续必须移除。


## 11. Python 与 C++ 的实施策略

后续提升性能时，允许将部分 Python 节点迁移到 C++，但必须满足“接口稳
定优先，语言替换其次”。

### 11.1 不变项

以下内容在语言迁移时不得变化：

- 公共模块名称
- Topic / Service / Action 名称
- 消息定义
- 参数名称
- Launch 暴露给上层的主要开关

### 11.2 迁移判定标准

满足任一条件，才进入 C++ 迁移评估：

- 高频控制或桥接链路成为 CPU 热点。
- Python 节点在目标硬件上存在明显延迟抖动。
- 单节点承担高频序列化、解析、批量数值计算。
- 已经通过 profiling 证明瓶颈确实在 Python 运行时。

禁止仅凭“感觉更快”发起大规模重写。

### 11.3 建议保留 Python 的模块

- `task_service_bridge`
- `teleoperation_bridge`
- Launch 组织层
- 低频工具、调试和运维脚本

这些模块更偏协议整合和流程编排，优先保证开发效率和可读性。

### 11.4 优先评估 C++ 的模块

- `execution_manager`
- `motion_control` 的高频控制内核
- `simulation_bridge`
- `servo_hardware` 中高频路由或批量状态聚合部分

### 11.5 迁移方式

必须按“接口先稳定，后替换后端”的方式实施：

1. 先冻结公共接口和消息。
2. 保持旧节点继续可运行。
3. 新增 C++ 后端节点，输入输出完全复用原公共接口。
4. 在 launch 中通过参数切换后端实现。
5. 验证通过后再移除旧 Python 实现。

### 11.6 双栈命名约束

只有在确实需要双栈并行时，才允许临时出现实现后缀包名：

- `motion_control_cpp`
- `execution_manager_cpp`

约束：

- 这类后缀只允许存在于实现期。
- 对外暴露的模块文档名称仍然是 `motion_control`、
  `execution_manager`。
- 不允许因为语言迁移再次引入第二套公共命名。


## 12. 分阶段实施计划

### Phase 0：统一命名与冻结红线

产出：

- 本文档
- 统一公共术语
- 红线、仲裁规则、迁移判定标准

完成标准：

- 新增代码和文档不再继续引入历史缩写。
- 新增模块必须按目标职责域命名。

### Phase 1：目录重组，不改业务语义

目标：

- 先让目录表达职责边界。
- 保持当前 ROS 包仍能构建。

动作：

1. 建立 `interfaces/`、`control/`、`execution/`、`bridges/` 等一级目录。
2. 将已有包移入对应职责域目录。
3. 更新所有 launch、`FindPackageShare`、路径引用。
4. 不在本阶段引入大规模接口改名。

完成标准：

- `colcon build` 通过。
- 现有 launch 仍可启动。
- 目录层级能直接看出职责域。

### Phase 2：拆包，消除明显职责混合

目标：

- 先拆最混杂的 ROS 包。

动作：

1. 从 `websocket_bridge` 中拆出仿真相关职责。
2. 从 `servo_hardware` 中拆出 `sensor_hardware` ROS 包。
3. 将 `record_load_action` 从正式主链路依赖中降级为可选工具依赖。
4. 把 `full_system.launch.py` 拆为硬件、遥控、仿真三个子 launch。

完成标准：

- `teleoperation_bridge` 和 `simulation_bridge` 在包级别不再混用。
- IMU 节点不再由 `servo_hardware` 包导出。
- 主 launch 可以按场景组合，而不是所有组件硬绑在一起。

### Phase 3：建立接口包和执行边界

目标：

- 建立正式控制语义，停止上层直接碰驱动接口。

动作：

1. 新建 `task_api_msgs`、`motion_msgs`、`speech_msgs`、`perception_msgs`。
2. 引入最小可用 `execution_manager`。
3. 定义 `motion_control -> execution_manager -> servo_hardware` 的最小
   正式链路。
4. 保留 `servo_msgs` 作为驱动边界接口。

完成标准：

- 至少一个上层控制节点可以通过 `motion_msgs` 驱动执行层。
- 上层新增模块不再直接依赖 `servo_msgs`。

### Phase 4：收口正式入口和人工入口

目标：

- 明确谁是正式任务入口，谁是调试入口。

动作：

1. 新建 `task_service_bridge`。
2. 将外部任务请求统一接到 `/task/execute`。
3. 将 `teleoperation_bridge` 改造成只走调试或人工控制申请链路。
4. 在 `execution_manager` 中实现控制权仲裁。

完成标准：

- 外部任务服务无法绕过桥接层。
- 遥控入口和正式任务入口共享同一执行边界，但不共享默认优先级。

### Phase 5：补齐感知和语音模块

目标：

- 形成完整的内部运行时链路。

动作：

1. 新建 `speech_interface`。
2. 新建 `vision_perception`。
3. 接入 `task_service_bridge` 和 `motion_control`。

完成标准：

- 语音和视觉不再通过临时字段耦合到桥接代码中。
- 感知输出使用结构化接口。

### Phase 6：按 profiling 结果推进 C++ 迁移

目标：

- 在不改公共接口的前提下提升性能。

动作：

1. 对 `execution_manager`、`motion_control`、`simulation_bridge` 做性能分
   析。
2. 只迁移真正的热点路径。
3. 通过 launch 参数切换 Python / C++ 后端。
4. 在硬件和仿真环境分别做回归验证。

完成标准：

- C++ 迁移不改变外部接口。
- Python 与 C++ 后端切换不影响上层调用方式。


## 13. 当前仓库的优先改造顺序

基于当前代码实际情况，推荐顺序调整为：

1. `websocket_bridge`
   先把遥控职责和仿真职责拆开。
2. `servo_hardware`
   先把 IMU 和舵机拆包。
3. `execution_manager`
   尽快建立执行边界和控制仲裁。
4. `parallel_3dof_controller`
   从直接发布 `ServoCommand` 迁移到 `motion_msgs`。
5. `servo_msgs`
   固定为驱动边界接口包。
6. `task_api_msgs` / `motion_msgs`
   搭起正式接口层。
7. `task_service_bridge`
   锁定外部正式入口。
8. `speech_interface` / `vision_perception`
   作为后续能力接入点。


## 14. 架构红线

以下行为禁止进入主线设计：

1. 外部任务服务直接调用 `servo_hardware`
2. `task_service_bridge` 直接发布 `ServoCommand`
3. `motion_control` 直接访问串口或协议驱动
4. `teleoperation_bridge` 作为正式任务入口长期存在
5. 上层模块直接订阅驱动私有话题
6. 在业务包中随意新增 `msg/srv/action`，绕过接口包
7. 一个概念使用多个公共名字并行存在
8. 因为切换到 C++ 就重命名公共接口或公共模块


## 15. 验收标准

重构完成后，至少必须满足以下条件：

- 目录结构直接体现职责分层。
- 当前 ROS 包和目标职责之间存在清晰的一对一或一对多映射说明。
- 至少一条正式任务链路跑通：
  `task_service_bridge -> motion_control -> execution_manager -> servo_hardware`
- 至少一条反馈链路跑通：
  `servo_hardware -> execution_manager -> motion_control -> task_service_bridge`
- 至少一条人工遥控链路跑通：
  `teleoperation_bridge -> execution_manager -> servo_hardware`
- 至少一条仿真链路跑通：
  `motion_control / execution_manager -> simulation_bridge -> simulator`
- Python 与 C++ 的后端替换不改变公共接口。
- 任何新增模块只看名称即可理解职责，不依赖仓库内部历史背景。


## 16. 后续文档要求

每个正式模块都必须补充自己的职责文档，至少包含以下章节：

1. Purpose
2. Package Ownership
3. Inputs
4. Outputs
5. Public Interfaces
6. Dependencies
7. Non-Responsibilities
8. Failure Modes
9. Implementation Notes

模块文档中必须额外说明两件事：

- 当前实现语言是 Python 还是 C++
- 如果后续迁移到 C++，哪些接口必须保持不变
