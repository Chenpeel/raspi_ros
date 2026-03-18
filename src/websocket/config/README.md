# 舵机配置文件说明

## bus_servo_map.json - 总线舵机串口映射配置

此文件定义了每个串口对应的舵机ID列表，系统会根据此配置自动创建对应的驱动节点。

### 配置格式

```json
{
    "串口路径": [舵机ID列表],
    ...
}
```

### 配置示例

```json
{
    "/dev/ttyAMA0": [1, 2],
    "/dev/ttyAMA1": [7, 3, 9, 10, 11],
    "/dev/ttyAMA2": [4, 5],
    "/dev/ttyAMA3": [8, 6, 12, 13, 14]
}
```

### 如何修改配置

#### 1. 修改现有串口的舵机ID列表

例如，将 `/dev/ttyAMA0` 的舵机从 `[1, 2]` 改为 `[1, 2, 3]`：

```json
{
    "/dev/ttyAMA0": [1, 2, 3],  // 修改这里
    "/dev/ttyAMA1": [7, 9, 10, 11],
    "/dev/ttyAMA2": [4, 5],
    "/dev/ttyAMA3": [8, 6, 12, 13, 14]
}
```

#### 2. 添加新的串口

例如，添加 `/dev/ttyAMA4`：

```json
{
    "/dev/ttyAMA0": [1, 2],
    "/dev/ttyAMA1": [7, 3, 9, 10, 11],
    "/dev/ttyAMA2": [4, 5],
    "/dev/ttyAMA3": [8, 6, 12, 13, 14],
    "/dev/ttyAMA4": [15, 16]  // 新增
}
```

#### 3. 删除不使用的串口

直接删除对应的行即可。

### 注意事项

1. **舵机ID不能重复**：每个舵机ID只能出现在一个串口的列表中
2. **串口路径必须正确**：确保硬件上确实存在该串口设备
3. **修改后需要重启系统**：修改配置后需要重新启动 ROS 2 launch 文件才能生效
4. **开发环境 vs 安装环境**：
   - 开发环境：直接修改 `src/websocket/config/bus_servo_map.json`
   - 安装环境：修改安装后的 `share/websocket_bridge/config/bus_servo_map.json`

### 重新编译和安装

如果在开发环境中修改了配置文件，需要重新编译以更新安装目录：

```bash
colcon build --packages-select websocket_bridge
source install/setup.bash
```

### 验证配置

启动系统后，launch 文件会打印加载的配置信息：

```
✓ 已加载舵机映射配置: <bus_servo_map.json 的实际路径>
  配置内容: {'/dev/ttyAMA0': [1, 2], ...}
```

并在启动信息中显示每个串口负责的舵机ID：

```
总线舵机驱动板:
  - /dev/ttyAMA0 @ 115200 bps (舵机ID: 1, 2)
  - /dev/ttyAMA1 @ 115200 bps (舵机ID: 7, 3, 9, 10, 11)
  ...
```

### 故障排查

如果配置文件加载失败，系统会使用默认配置并打印警告：

```
⚠ 警告: 舵机映射配置文件不存在: <bus_servo_map.json 的实际路径>
  将使用默认配置
```

此时请检查：
1. 配置文件是否存在
2. 文件路径是否正确
3. JSON 格式是否正确（可以用 `cat bus_servo_map.json | jq` 验证）

## BVH 动作配置

BVH 动作配置已迁移至 `record_load_action` 包，详见：

- `src/record_load_action/README.md`
