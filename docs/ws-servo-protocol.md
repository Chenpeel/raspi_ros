# WS 舵机交接协议

## 1. 连接

- URL: `ws://192.168.0.100:9102/`
- 上行周期: `50ms`（20Hz）
- 心跳周期: `15s`

## 2. 建连握手（必须）

1. 客户端连接成功后先发：

```json
{"type":"register","name":"body"}
```

2. 服务端返回 `connected` / `presence`，客户端从
   `onlineClients` / `onlineUsers` 中找到 `name=="esp32"` 的 `id`，
   保存为 `ws_target_id`。

## 3. `servo_control`

方向：`client -> server`

```json
{
  "type": "servo_control",
  "content": "[{\"character_name\":\"jiyuan\",\"web_servo\":{\"is_bus_servo\":true,\"servo_id\":21,\"position\":173,\"speed\":100}}]"
}
```

关键点：

- `content` 是 **JSON 字符串**，反序列化后是数组。
- 数组每项结构固定：

```json
{
  "character_name": "jiyuan",
  "web_servo": {
    "is_bus_servo": true,
    "servo_id": 21,
    "position": 173,
    "speed": 100
  }
}
```

字段说明：

- `character_name`: 角色名（示例 `jiyuan`）
- `is_bus_servo`: `true`=总线舵机，`false`=PCA
- `servo_id`: 舵机ID/通道ID
- `position`: 目标位置
- `speed`: 速度（默认 `100`）

## 4. 下行处理

服务端下发使用 `type=servo_control` + 同一 `content` 结构：

```json
{"type":"servo_control","content":"[{\"character_name\":\"jiyuan\",\"web_servo\":{\"is_bus_servo\":true,\"servo_id\":21,\"position\":173,\"speed\":100}}]"}
```

处理规则（客户端侧）：

- 把 `content` 当作 JSON 字符串解析为数组。
- 逐项读取 `character_name` 与 `web_servo`。

## 5. 心跳（最小要求）

- 客户端每 `15s` 发送：`{"type":"heartbeat"}`
- 收到服务端 `{"type":"heartbeat"}` 后立即回同样内容
