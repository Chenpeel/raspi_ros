# 运行时模块职责说明

## 1. 文档目的

本文是 ROS 2 运行时模块职责的统一事实来源。它与 `plan.md` 配套，用于
约束后续的包拆分、接口设计、launch 组织以及 Python 到 C++ 的迁移。

所有正式模块都必须满足一个基本要求：

- 只看模块名就能理解职责。
- 不依赖仓库历史缩写才能读懂。
- 不因为实现语言变化而改变模块边界。


## 2. 系统总边界

- 本仓库负责内部实时运行时。
- 外部任务服务位于仓库外部。
- 外部正式任务请求只能进入 `task_service_bridge`。
- 人工遥控和调试请求只能进入 `teleoperation_bridge`。
- 所有执行请求必须经过 `execution_manager` 才能触达驱动层。
- 硬件驱动层不能自行暴露正式任务 API。


## 3. 模块清单

正式模块如下：

- `motion_control`
- `speech_interface`
- `vision_perception`
- `execution_manager`
- `servo_hardware`
- `sensor_hardware`
- `task_service_bridge`
- `teleoperation_bridge`
- `simulation_bridge`
- `task_api_msgs`
- `motion_msgs`
- `speech_msgs`
- `perception_msgs`
- `servo_msgs`


## 4. motion_control

### Purpose

- 负责运动学、轨迹生成、约束检查、闭环控制和执行状态产出。

### Package Ownership

- 目标 ROS 包：`motion_control`
- 当前承接包：`parallel_3dof_controller`
- 当前定位：控制原型，不是最终完整控制层

### Inputs

- 来自 `task_service_bridge` 的任务级目标
- 来自 `vision_perception` 的感知更新
- 来自 `execution_manager` 的执行反馈

### Outputs

- 发往 `execution_manager` 的运动级命令
- 发回 `task_service_bridge` 的任务进度和控制状态

### Public Interfaces

- 依赖 `motion_msgs`
- 可依赖 `perception_msgs`
- 与正式任务入口交互时依赖 `task_api_msgs`

### Dependencies

- 运动学求解器
- 约束和轨迹模块
- 必要的机器人参数和标定配置

### Non-Responsibilities

- 不直接访问串口、I2C、寄存器和驱动协议
- 不直接发布 `ServoCommand`
- 不处理语音识别、视觉推理、对外协议适配

### Failure Modes

- 目标不可达
- 约束冲突
- 重规划失败
- 控制超时

### Implementation Notes

- 当前 `parallel_3dof_controller` 直接依赖 `servo_msgs`，这是过渡态。
- 在 `execution_manager` 落地后，必须改为只输出 `motion_msgs`。
- 高频控制内核是后续优先评估迁移到 C++ 的部分。


## 5. speech_interface

### Purpose

- 负责音频输入、语音识别、语音合成和播放状态。

### Package Ownership

- 目标 ROS 包：`speech_interface`
- 当前承接包：暂无

### Inputs

- 麦克风或音频流
- 语音合成请求

### Outputs

- 转写结果
- 播放状态

### Public Interfaces

- `speech_msgs`

### Dependencies

- 音频设备
- ASR / TTS 后端适配层

### Non-Responsibilities

- 不做任务理解
- 不做控制决策
- 不直接接入硬件执行链路

### Failure Modes

- 设备不可用
- 识别失败
- 合成失败
- 中断或取消

### Implementation Notes

- 该模块更偏流程编排和外部能力适配，默认优先使用 Python 实现。
- 如果后续引入 C++，也只能替换音频热点路径，不能改变 `speech_msgs`。


## 6. vision_perception

### Purpose

- 负责图像接入、检测、跟踪、位姿估计和场景结构化输出。

### Package Ownership

- 目标 ROS 包：`vision_perception`
- 当前承接包：暂无

### Inputs

- 图像帧
- 视频流
- 相机标定和传感器元数据

### Outputs

- 目标列表
- 场景状态摘要
- 位姿或跟踪结果

### Public Interfaces

- `perception_msgs`

### Dependencies

- 相机输入
- 推理后端
- 标定配置

### Non-Responsibilities

- 不发布舵机命令
- 不做任务规划
- 不持有长期记忆

### Failure Modes

- 相机不可用
- 检测低置信度
- 跟踪丢失
- 位姿估计失败

### Implementation Notes

- 模型推理和图像处理可能成为后续 C++ 或加速后端迁移热点。
- 但对外必须始终维持结构化 `perception_msgs` 输出。


## 7. execution_manager

### Purpose

- 负责执行协调、安全保护、控制权仲裁和驱动能力映射。

### Package Ownership

- 目标 ROS 包：`execution_manager`
- 当前承接包：暂无
- 当前缺口：这是现阶段最需要尽快补齐的模块

### Inputs

- 来自 `motion_control` 的运动级命令
- 来自 `teleoperation_bridge` 的人工控制请求
- 来自 `servo_hardware` 和 `sensor_hardware` 的状态反馈

### Outputs

- 发往驱动层的执行请求
- 对上层发布的执行反馈、执行结果和安全事件

### Public Interfaces

- 上行依赖 `motion_msgs`
- 下行依赖 `servo_msgs`

### Dependencies

- 仲裁状态机
- 急停与超时控制
- 驱动映射层

### Non-Responsibilities

- 不做任务语义理解
- 不做轨迹生成
- 不直接对外暴露任务级 API

### Failure Modes

- 控制权冲突
- 非法命令
- 驱动不可用
- 急停或安全违规
- 超时未响应

### Implementation Notes

- `execution_manager` 是正式任务入口和人工入口的唯一汇合点。
- 它必须持有控制权，而不是把仲裁逻辑下放给桥接层。
- 这是后续最优先考虑迁移到 C++ 的模块之一。


## 8. servo_hardware

### Purpose

- 负责总线舵机、PCA 和相关执行器驱动。

### Package Ownership

- 目标 ROS 包：`servo_hardware`
- 当前承接包：`servo_hardware`
- 当前问题：与 `sensor_hardware` 混在同一个 ROS 包

### Inputs

- 来自 `execution_manager` 的驱动级执行请求

### Outputs

- 舵机状态
- 驱动诊断
- 协议错误和连接状态

### Public Interfaces

- `servo_msgs`

### Dependencies

- 串口、I2C、总线协议实现
- 设备映射和限制配置

### Non-Responsibilities

- 不理解任务语义
- 不做轨迹规划
- 不做正式对外 API 适配

### Failure Modes

- 通信失败
- 过载
- 过热
- ID 映射错误
- 协议探测失败

### Implementation Notes

- 当前包中保留 IMU 节点是过渡态，必须拆出 `sensor_hardware`。
- 高频状态聚合和高频路由如果成为瓶颈，可以评估迁移到 C++。
- `servo_msgs` 是此模块对外唯一正式边界。


## 9. sensor_hardware

### Purpose

- 负责 IMU 和未来板载传感器的底层接入。

### Package Ownership

- 目标 ROS 包：`sensor_hardware`
- 当前承接代码位置：`servo_hardware` 包中的 `sensor_hardware` Python 模块
- 当前问题：尚未成为独立 ROS 包

### Inputs

- 传感器轮询
- 中断或串口读取

### Outputs

- 传感器原始状态
- 最小必要预处理结果
- 诊断信息

### Public Interfaces

- 标准 ROS 传感器消息或专用传感器消息

### Dependencies

- 串口或设备驱动
- 标定配置

### Non-Responsibilities

- 不做任务推理
- 不做视觉感知
- 不做控制决策

### Failure Modes

- 读取失败
- 设备断开
- 标定错误
- 时间戳漂移

### Implementation Notes

- 必须在 Phase 2 拆成独立 ROS 包。
- 拆包后不得再通过 `servo_hardware` 的 `entry_points` 导出。


## 10. task_service_bridge

### Purpose

- 作为外部任务服务进入 ROS 运行时的唯一正式桥接层。

### Package Ownership

- 目标 ROS 包：`task_service_bridge`
- 当前承接包：暂无

### Inputs

- 外部任务请求
- 会话上下文
- 来自 `motion_control` 的执行状态
- 来自 `speech_interface` 和 `vision_perception` 的补充结果

### Outputs

- 进入 ROS 内部的任务执行请求
- 返回外部的任务状态、对话回复和结果摘要

### Public Interfaces

- `task_api_msgs`
- 可读但不拥有 `speech_msgs`
- 可读但不拥有 `perception_msgs`

### Dependencies

- 外部任务协议适配层
- 会话追踪和超时处理

### Non-Responsibilities

- 不直接发 `ServoCommand`
- 不直接访问硬件协议
- 不绕过 `motion_control` 或 `execution_manager`

### Failure Modes

- 任务载荷非法
- 下游模块超时
- 会话不匹配
- 取消或中断

### Implementation Notes

- 该模块更适合保留 Python 实现。
- 后续如果要迁移局部热点，公共动作接口 `task_api_msgs` 必须保持不变。


## 11. teleoperation_bridge

### Purpose

- 提供人工遥控、调试和演示入口。

### Package Ownership

- 目标 ROS 包：`teleoperation_bridge`
- 当前承接包：`websocket_bridge` 的主要桥接逻辑
- 当前问题：与仿真桥、演示资源、硬件级控制逻辑混在一起

### Inputs

- WebSocket 或其他人工操作流

### Outputs

- 进入运行时的人工控制请求
- 返回操作态状态和调试信息

### Public Interfaces

- 调试模式下可以使用内部控制接口
- 正式模式下不得替代 `task_service_bridge`

### Dependencies

- WebSocket 服务端
- 操作态认证或会话信息

### Non-Responsibilities

- 不负责正式任务编排
- 不持有最终控制权
- 不直接对硬件长期开放驱动接口

### Failure Modes

- 连接中断
- 非法操作命令
- 仲裁拒绝
- 不安全人工接管

### Implementation Notes

- 当前 `websocket_bridge` 直接发布 `ServoCommand` 是过渡态。
- 后续应先接到 `execution_manager`，再由执行层决定是否接纳。
- WebSocket 服务和调试聚合保留 Python 实现通常更合适。


## 12. simulation_bridge

### Purpose

- 负责内部运行时与仿真器之间的命令和状态互通。

### Package Ownership

- 目标 ROS 包：`simulation_bridge`
- 当前承接包：
  - `sim_servo_bridge_cpp`
  - `websocket_bridge` 中的 Isaac 相关桥接代码
- 当前问题：仿真能力散落在多个包中

### Inputs

- 运动级命令或驱动级兼容命令
- 仿真配置

### Outputs

- 仿真状态反馈
- 仿真执行结果

### Public Interfaces

- 优先对接 `motion_msgs`
- 必要时兼容 `servo_msgs`

### Dependencies

- 仿真器 API
- 关节映射和限幅配置

### Non-Responsibilities

- 不直接控制生产硬件
- 不承担任务服务入口职责
- 不做任务语义理解

### Failure Modes

- 仿真器不可用
- 状态同步不一致
- 模式不支持

### Implementation Notes

- 该模块天然适合 C++ 实现，尤其在高频桥接场景下。
- 但必须先统一对外接口和 launch 入口，再讨论实现语言统一。


## 13. 接口包职责

### task_api_msgs

Purpose：

- 定义外部任务服务面向 ROS 的正式契约。

Owns：

- 任务 Action
- 任务状态
- 对话回复
- 上下文信号

Does Not Own：

- 驱动级控制消息
- 硬件诊断

### motion_msgs

Purpose：

- 定义控制层和执行层之间的正式控制语义。

Owns：

- 运动命令
- 状态反馈
- 执行结果

Does Not Own：

- 任务对话
- 驱动寄存器细节

### speech_msgs

Purpose：

- 定义语音输入输出契约。

Owns：

- 转写结果
- 语音合成请求
- 语音状态

Does Not Own：

- 任务执行状态
- 运动命令

### perception_msgs

Purpose：

- 定义感知结构化输出。

Owns：

- 目标列表
- 场景状态
- 位姿和跟踪摘要

Does Not Own：

- 驱动状态
- 任务对话

### servo_msgs

Purpose：

- 定义执行边界上的低层驱动契约。

Owns：

- 舵机命令
- 舵机状态
- 驱动诊断和读取服务

Does Not Own：

- 任务级目标
- 感知输出
- 对话或转写内容


## 14. 跨模块规则

1. `task_service_bridge` 是唯一正式外部入口。
2. `teleoperation_bridge` 只能是人工和调试入口。
3. `execution_manager` 是唯一控制仲裁边界。
4. `motion_control` 不得直接访问串口驱动。
5. `servo_hardware` 和 `sensor_hardware` 不得暴露任务级 API。
6. 所有新增模块必须优先依赖接口包，而不是跨包直接引用内部实现。
7. Python 到 C++ 的迁移不得改变公共接口。


## 15. 推荐拆分结果

当新目录结构落地后，本文应拆分为每个模块自己的职责文档：

- `control/motion_control/docs/responsibilities.md`
- `speech/speech_interface/docs/responsibilities.md`
- `perception/vision_perception/docs/responsibilities.md`
- `execution/execution_manager/docs/responsibilities.md`
- `execution/servo_hardware/docs/responsibilities.md`
- `execution/sensor_hardware/docs/responsibilities.md`
- `bridges/task_service_bridge/docs/responsibilities.md`
- `bridges/teleoperation_bridge/docs/responsibilities.md`
- `bridges/simulation_bridge/docs/responsibilities.md`

在拆分前，本文保持为统一职责总表。
