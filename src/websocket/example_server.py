#!/usr/bin/env python3
"""
简单示例：演示如何使用 Jiyuan WebSocket 系统
"""

import asyncio
import json
from jiyuan_websocket import JiyuanWebSocketServer, WebSocketHandler


async def example_servo_command_handler(servo_cmd: dict):
    """
    处理舵机命令的示例回调
    
    Args:
        servo_cmd: 舵机控制命令
    """
    print(f"[Example] 收到舵机命令: {servo_cmd}")
    print(f"  - 舵机类型: {servo_cmd.get('servo_type', 'bus')}")
    print(f"  - 舵机 ID: {servo_cmd.get('servo_id')}")
    print(f"  - 位置: {servo_cmd.get('position')}")
    print(f"  - 速度: {servo_cmd.get('speed', 'N/A')}")
    
    # 这里可以转发到 ROS 2 话题或实际硬件


async def example_status_query_handler():
    """
    处理状态查询的示例回调
    遵循 std_jiyuan2web_stream.json 格式
    """
    return {
        "character_name": "jiyuan",
        "bus_servos": {
            "id_1": 90,
            "id_2": 90,
            "id_3": 90
        },
        "pwm_servos": {
            "id_1": 90,
            "id_2": 90,
            "id_3": 90
        },
        "time": "2025-11-28-12:00:00",
        "current_status": {
            "movement_active": False,
            "listening": False,
            "action": "idle",
            "led_state": "off"
        },
        "result_code": 200
    }


async def main():
    """主函数"""
    
    print("=" * 60)
    print("Jiyuan WebSocket 服务器示例")
    print("=" * 60)
    
    # 创建服务器
    server = JiyuanWebSocketServer(
        host="0.0.0.0",
        port=9102,
        device_id="jiyuan_example",
        debug=True
    )
    
    # 注册回调
    server.set_servo_command_callback(example_servo_command_handler)
    server.set_status_query_callback(example_status_query_handler)
    
    # 启动服务器
    await server.start()
    
    print("\n[Main] 服务器已启动，等待连接...")
    print("[Main] 尝试连接: ws://localhost:9102")
    print("[Main] 发送示例命令:")
    print('  {"b": -1, "c": 1, "p": 1500}  # 总线舵机')
    print('  {"b": 0, "c": 0, "p": 300}    # PCA 舵机')
    
    try:
        # 保持运行
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("\n[Main] 收到退出信号")
    finally:
        await server.stop()


if __name__ == "__main__":
    asyncio.run(main())
