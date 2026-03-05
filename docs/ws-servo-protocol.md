# WS 舵机协议

## 1) 连接地址
- ws经 server 代理: `ws://<SERVER_HOST>:9101/ros` 
    - 当前SERVER_HOST: `192.168.0.102`

## 2) 发送格式
```json
{
  "character_name": "jiyuan",
  "web_servo": {
    "is_bus_servo": true,
    "servo_id": 12,
    "position": -20,
    "speed": 100
  }
}
```

字段:
- `is_bus_servo`: `true`=总线舵机，`false`=PCA
- `servo_id`: 舵机id, 总线常用 `0~200`；PCA 必须 `0~15`
- `speed`: 舵机旋转速度，默认 `100`ms

## 3) position 范围
- 总线舵机: `[-90, 90]`
  - `-90 -> 500us`
  -  `0 -> 1500us`
  - `90 -> 2500us`
- 总线舵机: `500~2500` 可按脉宽使用
- PCA 舵机（未使用）:
  - `0~180` 角度
  - `>1000` 微秒
  - 其他按 tick

