"""
树莓派 WebSocket 客户端示例配置
用于连接到主机上的 Jiyuan WebSocket 服务器

使用方法:
1. 修改 ws_url 指向主机 IP
2. 在树莓派上运行此脚本
3. 树莓派将连接到主机并接收舵机控制命令
"""

import asyncio
import json
import time
from websockets import connect, exceptions


# ========== 配置项 ==========

# 主机地址 - 修改为实际主机 IP
MAIN_HOST_IP = "192.168.31.35"
MAIN_HOST_PORT = 9102

# WebSocket 服务器地址
WS_URL = f"ws://{MAIN_HOST_IP}:{MAIN_HOST_PORT}"

# 设备配置
DEVICE_NAME = "jiyuan_raspberry_pi"
DEVICE_TYPE = "servo_controller"

# 舵机配置
MAX_BUS_SERVO_ID = 200
BUS_MIN_US = 500
BUS_MAX_US = 2500


# ========== 模拟舵机驱动函数 ==========

def control_bus_servo(servo_id: int, position: int, speed: int = 100):
    """
    控制总线舵机

    Args:
        servo_id: 舵机 ID
        position: 角度 (0-180)
        speed: 速度 (1-255)
    """
    print(f"[Servo] 控制总线舵机 ID={servo_id}, 位置={position}°, 速度={speed}")


def control_pca_servo(channel: int, position: int):
    """
    控制 PCA9685 舵机

    Args:
        channel: 通道 (0-15)
        position: PWM 值
    """
    print(f"[Servo] 控制 PCA9685 通道 {channel}, PWM={position}")
    # 这里应该调用实际的硬件驱动接口
    # 例如: pwm.set_pwm(channel, off=position)


# ========== 消息处理 ==========

async def handle_message(message: str):
    """
    处理从服务器收到的消息

    遵循 std_jiyuan2web_stream.json 格式:
    {
      "character_name": "jiyuan",
      "bus_servos": { "id_1": angle_1, ... },
      "pwm_servos": { "id_1": angle_1, ... },
      "time": "xxxx-yy-dd-HH:MM:SS",
      "current_status": {
        "movement_active": bool,
        "listening": bool,
        "action": str,
        "led_state": str
      },
      "result_code": int
    }

    Args:
        message: JSON 字符串
    """
    try:
        data = json.loads(message)
        msg_type = data.get("type") or data.get("Type")
        character_name = data.get("character_name")

        print(f"\n[Client] 收到消息: 角色={character_name}, 类型={msg_type}")

        if character_name == "jiyuan":
            # 处理 Jiyuan 标准响应格式
            print(f"[Status] 时间: {data.get('time')}")
            print(f"[Status] 结果码: {data.get('result_code')}")

            # 处理舵机状态
            bus_servos = data.get("bus_servos", {})
            if bus_servos:
                print(f"[Servos] 总线舵机: {bus_servos}")
                for servo_id_str, angle in bus_servos.items():
                    servo_id = int(servo_id_str.split('_')[1])
                    control_bus_servo(servo_id, angle)

            pwm_servos = data.get("pwm_servos", {})
            if pwm_servos:
                print(f"[Servos] PWM舵机: {pwm_servos}")
                for servo_id_str, angle in pwm_servos.items():
                    servo_id = int(servo_id_str.split('_')[1])
                    control_pca_servo(servo_id, angle)

            # 处理系统状态
            current_status = data.get("current_status", {})
            if current_status:
                print(
                    f"[System] 运动激活: {current_status.get('movement_active')}")
                print(f"[System] 监听状态: {current_status.get('listening')}")
                print(f"[System] 动作: {current_status.get('action')}")
                print(f"[System] LED: {current_status.get('led_state')}")

        elif msg_type == "heartbeat":
            print(f"[Client] 心跳检测 - 服务器在线")

        elif msg_type == "register_ack":
            print(f"[Client] 设备注册成功")

        elif msg_type == "status_update":
            print(f"[Client] 状态更新: {data.get('data')}")

        else:
            # 尝试解析简写协议 {b, c, p}
            if 'b' in data and 'c' in data and 'p' in data:
                b_flag = int(data['b'])
                c_val = int(data['c'])
                p_val = int(data['p'])
                s_val = int(data.get('s', 100))

                if b_flag == -1:
                    # 总线舵机
                    angle = int((p_val - BUS_MIN_US) * 180.0 /
                                (BUS_MAX_US - BUS_MIN_US))
                    control_bus_servo(c_val, angle, s_val)
                elif b_flag == 0:
                    # PCA 舵机
                    control_pca_servo(c_val, p_val)
            else:
                print(f"[Client] 未识别的消息: {data}")

    except json.JSONDecodeError as e:
        print(f"[Client] JSON 解析失败: {e}")
    except Exception as e:
        print(f"[Client] 处理消息异常: {e}")


# ========== WebSocket 客户端 ==========

async def websocket_client():
    """
    WebSocket 客户端主逻辑
    """
    last_heartbeat = 0
    reconnect_interval = 5

    while True:
        try:
            async with connect(WS_URL, ping_interval=10, ping_timeout=5) as websocket:
                print(f"\n✓ [Client] 连接成功: {WS_URL}")

                # 发送注册消息
                register_msg = json.dumps({
                    "type": "register",
                    "name": DEVICE_NAME,
                    "device_type": DEVICE_TYPE
                })
                await websocket.send(register_msg)
                print(f"[Client] 发送注册消息")

                # 接收消息循环
                while True:
                    try:
                        message = await asyncio.wait_for(websocket.recv(), timeout=30.0)
                        await handle_message(message)

                    except asyncio.TimeoutError:
                        # 发送心跳
                        if time.time() - last_heartbeat > 15:
                            heartbeat_msg = json.dumps({
                                "type": "heartbeat",
                                "status": "online",
                                "timestamp": int(time.time())
                            })
                            await websocket.send(heartbeat_msg)
                            last_heartbeat = time.time()
                            print(f"[Client] 发送心跳")

                    except exceptions.ConnectionClosed:
                        print(f"✗ [Client] 连接被关闭，{reconnect_interval}秒后重连...")
                        break

        except Exception as e:
            print(f"✗ [Client] 连接失败: {e}，{reconnect_interval}秒后重试...")
            await asyncio.sleep(reconnect_interval)


# ========== 主函数 ==========

async def main():
    """主函数"""
    print("=" * 60)
    print("Jiyuan WebSocket 树莓派客户端")
    print("=" * 60)
    print(f"连接到: {WS_URL}")
    print(f"设备名: {DEVICE_NAME}")
    print()

    await websocket_client()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[Main] 退出")
