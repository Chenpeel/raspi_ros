# ROS 仓库重构方案

## 1. 文档目的

本方案用于指导当前 ROS 2 仓库的结构重构。目标不是简单整理目录，而是建立一套脱离当前仓库上下文也能理解的公共命名、模块边界和接口约束。

本次重构采用以下总原则：

- ROS 内部承载实时链路和设备链路。
- 外部任务服务与 ROS 通过稳定桥接层交互。
- 公共接口使用职责名，不使用项目内缩写。
- 驱动层、控制层、感知层、语音层、桥接层职责必须清晰分离。


## 2. 公共术语表

以下名称为正式公共名称，后续目录、包名、文档标题、消息名、话题名均以此为准。

| 公共名称 | 含义 | 历史/兼容名称 |
| --- | --- | --- |
| `motion_control` | 运动控制层，负责逆解、轨迹、约束、状态闭环 | `robot_mpc`、`parallel_3dof_controller` |
| `speech_interface` | 语音输入输出层，负责音频采集、ASR、TTS | `stts` |
| `vision_perception` | 视觉感知层，负责检测、跟踪、位姿估计、场景摘要 | `vpti` |
| `execution_manager` | 执行协调层，负责命令落地、状态聚合、安全保护 | `exec_unit` |
| `task_service_bridge` | 外部任务服务桥接层，负责外部模块与 ROS 的任务级交互 | `robot_service_bridge` |
| `teleoperation_bridge` | 遥控与调试桥接层，负责 WebSocket 或其他人工操作入口 | `websocket_bridge` |
| `servo_hardware` | 舵机驱动层，负责总线/PWM 驱动与状态采集 | `servo_hardware` |
| `sensor_hardware` | 传感器驱动层，负责 IMU、相机等基础采集 | `sensor_hardware` |
| `simulation_bridge` | 仿真桥接层，负责仿真器与内部执行链路互通 | `sim_servo_bridge_cpp` |

命名约束：

- 公共名称只能有一个正式写法。
- 历史名称只用于迁移说明，不再作为新目录、新文档和新接口命名。
- 算法缩写可以用于实现类，不可用于系统级模块名。


## 3. 重构目标

### 3.1 目标架构

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
physical hardware or simulator
```

并行链路：

```text
microphone -> speech_interface -> task_service_bridge
camera     -> vision_perception -> task_service_bridge / motion_control
```

反馈链路：

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

### 3.2 核心目标

1. 外部任务服务不能直接控制硬件。
2. 控制层不能直接依赖驱动协议实现。
3. 驱动层不能承载任务语义。
4. 遥控入口和正式任务入口不能混用。
5. 目录名、包名、接口名脱离本仓库上下文也能理解。


## 4. 推荐目录树

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

### 4.1 现有包映射建议

- `src/parallel_3dof_controller` -> `src/control/motion_control`
- `src/hardware/servo_hardware` -> `src/execution/servo_hardware`
- `src/hardware/sensor_hardware` -> `src/execution/sensor_hardware`
- `src/websocket` -> `src/bridges/teleoperation_bridge`
- `src/sim_servo_bridge_cpp` -> `src/bridges/simulation_bridge`
- `src/servo_msgs` -> `src/interfaces/servo_msgs`
- 新增 `src/interfaces/task_api_msgs`
- 新增 `src/interfaces/motion_msgs`
- 新增 `src/interfaces/speech_msgs`
- 新增 `src/interfaces/perception_msgs`
- 新增 `src/bridges/task_service_bridge`
- 新增 `src/speech/speech_interface`
- 新增 `src/perception/vision_perception`
- 新增 `src/execution/execution_manager`

### 4.2 迁移策略

第一阶段优先调整目录层级，不强制立即修改 `package.xml` 中的包名。  
先稳定目录和职责，再决定是否统一更新 ROS 包名、Launch 名和历史脚本。


## 5. 模块职责与边界

### 5.1 motion_control

职责：

- 接收任务级目标、姿态目标、位姿目标。
- 执行逆解、轨迹生成、约束检查、控制闭环。
- 向 `execution_manager` 输出通用运动命令。
- 接收执行反馈并产出任务执行状态。

不负责：

- 不直接读写串口或总线协议。
- 不直接访问外部 LLM、RAG、记忆库。
- 不承担语音识别、视觉检测、前端遥控接入。

### 5.2 speech_interface

职责：

- 采集音频输入。
- 发布语音识别结果。
- 接收语音合成请求并输出音频。

不负责：

- 不解析任务意图。
- 不生成控制命令。
- 不直接调用硬件执行链路。

### 5.3 vision_perception

职责：

- 接入图像流、视频流、相机流。
- 输出结构化观测，如目标列表、位姿、场景状态。
- 为 `motion_control` 和 `task_service_bridge` 提供感知结果。

不负责：

- 不发布舵机控制命令。
- 不承担任务规划和对话管理。
- 不持有长期记忆或检索数据。

### 5.4 execution_manager

职责：

- 接收来自 `motion_control` 的通用运动命令。
- 将命令映射到驱动能力。
- 聚合来自驱动层的执行反馈。
- 负责急停、超时、连接状态与基础安全保护。

不负责：

- 不做轨迹规划。
- 不做语义理解。
- 不向外部任务服务暴露驱动细节。

### 5.5 servo_hardware / sensor_hardware

职责：

- 提供最低层设备驱动。
- 提供最小可用设备控制和状态采集。
- 对协议、端口、寄存器、传输错误负责。

不负责：

- 不理解任务语义。
- 不承担控制规划。
- 不直接响应外部任务请求。

### 5.6 task_service_bridge

职责：

- 作为外部任务服务的唯一正式入口。
- 将外部任务请求转换为 ROS 内部任务执行请求。
- 将内部任务状态、对话回复、感知摘要返回外部模块。

不负责：

- 不绕过 `motion_control` 直接下发执行命令。
- 不直接依赖 `servo_hardware`。
- 不承载底层协议适配。

### 5.7 teleoperation_bridge

职责：

- 提供遥控、调试、演示入口。
- 接收人工控制输入并转换为 ROS 内部可识别命令。

不负责：

- 不作为正式任务服务入口。
- 不默认与 `task_service_bridge` 共享同一控制优先级。


## 6. API 分层

### 6.1 外部 API

仅供外部任务服务使用，放在 `task_api_msgs/`。

建议接口：

- `ExecuteTask.action`
- `TaskStatus.msg`
- `DialogueReply.msg`
- `TaskContextSignal.msg`

设计约束：

- 只表达任务语义，不暴露驱动细节。
- 支持长时执行、取消、反馈。
- 必须携带任务追踪字段，如 `task_id`、`trace_id`。

### 6.2 内部 API

服务于 ROS 内部模块之间的交互。

建议接口包：

- `speech_msgs/`
- `perception_msgs/`
- `motion_msgs/`
- `servo_msgs/`

设计约束：

- 强结构化，避免歧义。
- 明确输入、输出、错误表达。
- 不把长文本上下文、向量索引、任务记忆塞进 ROS 消息。

### 6.3 驱动 API

驱动 API 是最低层契约，优先复用现有 `servo_msgs`。

适用范围：

- `execution_manager`
- `servo_hardware`
- `sensor_hardware`

禁止：

- 外部任务服务直接依赖驱动 API。
- `task_service_bridge` 直接发布 `ServoCommand`。


## 7. Topic / Service / Action 规划

### 7.1 Topic

用于高频状态流或持续数据流：

- `/speech/transcript`
- `/speech/synthesis/request`
- `/perception/object_list`
- `/perception/scene_state`
- `/execution/state_feedback`
- `/execution/result`

### 7.2 Action

用于可取消、可反馈、执行时间较长的任务：

- `/task/execute`
- `/motion/execute`

### 7.3 Service

用于同步查询和调试：

- 设备诊断
- 舵机位置读取
- 协议探测
- 校准接口

### 7.4 约束

- 任务级控制优先使用 `Action`。
- 高频反馈优先使用 `Topic`。
- 不允许用 `Service` 承载长时控制链路。
- 不允许用调试入口绕过正式任务链路。


## 8. 命名规范

### 8.1 包命名规范

- ROS package 使用小写蛇形命名：`task_service_bridge`
- 接口包统一使用 `_msgs` 结尾：`motion_msgs`
- 模块包名使用职责名，不使用模糊词：
  - 推荐：`vision_perception`
  - 禁止：`common`、`misc`、`engine2`

### 8.2 目录命名规范

- 一级职责目录必须是领域词：`control`、`speech`、`perception`、`execution`、`bridges`
- 包内配置放 `config/`
- Launch 文件放 `launch/`
- 测试放 `test/`

### 8.3 节点命名规范

- 源文件名使用 `*_node.py` 或 `*_node.cpp`
- ROS 节点运行名使用职责式命名：
  - `task_service_bridge_node`
  - `speech_transcript_node`
  - `execution_manager_node`

### 8.4 Topic 命名规范

- 统一使用小写蛇形命名。
- 必须按职责域分层：
  - `/task/...`
  - `/speech/...`
  - `/perception/...`
  - `/motion/...`
  - `/execution/...`
  - `/hardware/...`

示例：

- `/task/execute`
- `/task/status`
- `/speech/transcript`
- `/perception/object_list`
- `/motion/command`
- `/execution/state_feedback`
- `/hardware/servo/state`

### 8.5 消息命名规范

- Message 使用 `PascalCase.msg`
- Service 使用 `PascalCase.srv`
- Action 使用 `PascalCase.action`

推荐示例：

- `Transcript.msg`
- `SpeechSynthesisRequest.msg`
- `ObjectList.msg`
- `SceneState.msg`
- `MotionCommand.msg`
- `StateFeedback.msg`
- `ExecutionResult.msg`
- `ExecuteTask.action`

### 8.6 参数命名规范

- 参数使用 `snake_case`
- 布尔参数用 `enable_`、`use_`、`publish_` 前缀
- 避免项目内私有缩写进入公共参数名


## 9. 消息设计约束

### 9.1 通用字段

跨模块消息建议统一具备以下追踪字段之一或其等价字段：

- `trace_id`
- `session_id`
- `task_id`
- `source`
- `target`
- `timestamp`

### 9.2 分层隔离

- 任务层消息不包含舵机 ID、串口路径、驱动协议名。
- 控制层消息不包含长文本上下文和记忆原文。
- 驱动层消息不包含任务意图和对话语义。

### 9.3 错误表达

执行结果必须结构化表达：

- `status`
- `reason`
- `recoverable`

禁止仅使用日志字符串表达故障语义。


## 10. 架构红线

以下行为禁止进入主线设计：

1. 外部任务服务直接调用 `servo_hardware`
2. `task_service_bridge` 直接发布 `ServoCommand`
3. `motion_control` 直接访问串口或协议驱动
4. `teleoperation_bridge` 作为正式任务入口长期存在
5. 上层模块直接订阅驱动私有话题
6. 在业务包中随意新增 `msg/srv/action`，绕过接口包
7. 一个概念使用多个公共名字并行存在


## 11. 分阶段实施计划

### Phase 0：统一语言

产出：

- 本文档
- 公共术语表
- 公共命名规范

目标：

- 停止继续引入只在本仓库内部可理解的缩写
- 统一未来目录、文档、接口命名

### Phase 1：目录重组

产出：

- 新的 `src/` 分层目录
- 现有包归类完成

目标：

- 让目录直接表达职责边界
- 不急于修改包名，优先稳定结构

### Phase 2：接口包建立

产出：

- `task_api_msgs`
- `motion_msgs`
- `speech_msgs`
- `perception_msgs`

目标：

- 将公共接口从业务逻辑中抽离
- 明确外部 API 与内部 API 边界

### Phase 3：桥接层建立

产出：

- `task_service_bridge`
- 外部任务执行入口
- 状态和对话反馈出口

目标：

- 让外部任务服务只能通过桥接层进入 ROS

### Phase 4：内部功能落位

产出：

- `motion_control`
- `speech_interface`
- `vision_perception`
- `execution_manager`

目标：

- 形成完整内部实时链路

### Phase 5：入口收敛与仲裁

产出：

- 遥控入口降级为调试入口
- 正式任务入口与遥控入口的优先级规则

目标：

- 避免多入口争用同一执行链路


## 12. 当前仓库的优先改造顺序

建议按以下顺序推进：

1. `src/websocket`
   先明确其新定位为 `teleoperation_bridge`
2. `src/parallel_3dof_controller`
   作为 `motion_control` 原型继续演进
3. `src/hardware/servo_hardware`
   收敛为执行链路的底层驱动
4. `src/servo_msgs`
   固定为驱动接口包
5. 新增接口包
   搭建新 API 层
6. 新增 `task_service_bridge`
   锁定外部任务服务的唯一入口
7. 新增 `speech_interface` 与 `vision_perception`
   补齐多模态内部能力


## 13. 验收标准

重构完成后，至少应满足以下条件：

- 目录结构直接体现分层
- 任何人只看公共名称即可理解模块职责
- 外部任务服务无法绕过桥接层直接控制硬件
- 控制层不依赖驱动协议细节
- 驱动层不依赖任务语义
- 至少一条正式链路跑通：
  - `task_service_bridge -> motion_control -> execution_manager -> servo_hardware`
- 至少一条反馈链路跑通：
  - `servo_hardware -> execution_manager -> motion_control -> task_service_bridge`


## 14. 后续文档要求

后续每个正式模块都必须补充自己的职责文档，最少包含以下章节：

1. Purpose
2. Inputs
3. Outputs
4. Dependencies
5. Non-Responsibilities
6. Failure Modes

