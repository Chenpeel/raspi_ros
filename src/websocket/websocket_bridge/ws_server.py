"""
WebSocket 服务器 - 主服务程序
基于 pi/server.py 改写，集成 ROS 2 功能
"""

import asyncio
import json
import time
import sys
from typing import Set, Optional
from websockets.server import serve, WebSocketServerProtocol
from websockets.exceptions import ConnectionClosed

from .websocket_handler import WebSocketHandler


class WebSocketBridgeServer:
    """
    WebSocket通信桥接服务器

    功能:
    1. 管理 WebSocket 连接
    2. 路由客户端消息到 ROS 2
    3. 广播来自 ROS 2 的状态更新
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 9102,
                 device_id: str = "default", debug: bool = False):
        """
        初始化 WebSocket 服务器

        Args:
            host: 监听地址
            port: 监听端口
            device_id: 设备 ID
            debug: 是否启用调试日志
        """
        self.host = host
        self.port = port
        self.device_id = device_id
        self.debug = debug

        self.handler = WebSocketHandler(device_id=device_id, debug=debug)
        self.clients: Set[WebSocketServerProtocol] = set()
        self.client_info = {}  # 客户端信息 {websocket: {"id": str, "name": str}}

        # 将 ROS2 服务器本身注册为一个虚拟设备
        self.robot_device = {
            "id": "ros2-robot-device",
            "name": device_id  # 使用 device_id 作为设备名（默认 "robot"）
        }

        self.server = None
        self.running = True

        # 心跳配置
        self.heartbeat_interval = 15.0  # 秒
        self.heartbeat_timeout = 5.0  # 秒

    def set_servo_command_callback(self, callback):
        """设置舵机命令回调"""
        self.handler.register_servo_command_handler(callback)

    def set_heartbeat_callback(self, callback):
        """设置心跳回调"""
        self.handler.register_heartbeat_handler(callback)

    def set_status_query_callback(self, callback):
        """设置状态查询回调"""
        self.handler.register_status_query_handler(callback)

    async def start(self):
        """启动 WebSocket 服务器"""
        try:
            self.server = await serve(
                self.handle_client,
                self.host,
                self.port,
                ping_interval=10,
                ping_timeout=5
            )
            print(f"[WebSocketServer] 启动成功: ws://{self.host}:{self.port}")

            # 启动心跳任务
            asyncio.create_task(self.heartbeat_loop())

        except Exception as e:
            print(f"[WebSocketServer] 启动失败: {e}")
            raise

    async def stop(self):
        """停止 WebSocket 服务器"""
        self.running = False

        # 关闭所有连接
        for client in list(self.clients):
            try:
                await client.close()
            except Exception as e:
                if self.debug:
                    print(f"[WebSocketServer] 关闭连接失败: {e}")

        if self.server:
            self.server.close()
            await self.server.wait_closed()

        print("[WebSocketServer] 已停止")

    async def handle_client(self, websocket: WebSocketServerProtocol, path: str):
        """
        处理客户端连接

        Args:
            websocket: WebSocket 连接
            path: 连接路径
        """
        client_addr = websocket.remote_address
        self.clients.add(websocket)

        print(f"[WebSocketServer] 客户端连接: {client_addr}")

        try:
            async for message in websocket:
                await self._process_client_message(websocket, message)

        except ConnectionClosed:
            print(f"[WebSocketServer] 客户端断开: {client_addr}")
        except Exception as e:
            print(f"[WebSocketServer] 客户端异常: {e}")
        finally:
            self.clients.discard(websocket)
            # 清理客户端信息
            if websocket in self.client_info:
                del self.client_info[websocket]
            # 广播更新后的用户列表
            await self.broadcast_user_list()

    async def _process_client_message(self, websocket: WebSocketServerProtocol,
                                      message: str):
        """
        处理来自客户端的消息

        Args:
            websocket: WebSocket 连接
            message: 消息内容
        """
        try:
            if self.debug:
                print(f"[WebSocketServer] 收到消息: {message[:100]}...")

            # 解析消息
            try:
                data = json.loads(message)
                msg_type = data.get("type", "").lower()

                # 拦截 register 消息，返回兼容前端的响应
                if msg_type == "register":
                    await self._handle_register(websocket, data)
                    return

                # 拦截发给 ROS2 机器人的私有消息（舵机控制）
                if msg_type == "private" and data.get("to") == "ros2-robot-device":
                    await self._handle_private_to_robot(websocket, data)
                    return

                # 拦截 servo_control 类型的消息
                if msg_type == "servo_control":
                    # 直接处理为舵机控制命令
                    await self._handle_servo_control_direct(data)
                    # 发送确认响应给客户端
                    ack = {
                        "type": "private_ack",
                        "status": "processed",
                        "device_id": self.device_id,
                        "timestamp": int(time.time())
                    }
                    await websocket.send(json.dumps(ack, ensure_ascii=False))
                    return

            except json.JSONDecodeError:
                pass

            response = await self.handler.handle_message(message)

            if response:
                await websocket.send(response)
                if self.debug:
                    print(f"[WebSocketServer] 发送响应: {response[:100]}...")

        except Exception as e:
            print(f"[WebSocketServer] 处理消息异常: {e}")
            try:
                error_resp = json.dumps({
                    "type": "error",
                    "status": "error",
                    "device_id": self.device_id,
                    "message": str(e),
                    "timestamp": int(time.time())
                }, ensure_ascii=False)
                await websocket.send(error_resp)
            except Exception as e2:
                if self.debug:
                    print(f"[WebSocketServer] 发送错误响应失败: {e2}")

    async def heartbeat_loop(self):
        """
        心跳循环 - 定期向所有客户端发送心跳
        """
        while self.running:
            try:
                await asyncio.sleep(self.heartbeat_interval)

                if not self.clients:
                    continue

                heartbeat_msg = json.dumps({
                    "type": "heartbeat",
                    "status": "online",
                    "device_id": self.device_id,
                    "timestamp": int(time.time())
                }, ensure_ascii=False)

                # 并发发送心跳到所有客户端
                tasks = [
                    client.send(heartbeat_msg)
                    for client in self.clients
                ]

                if tasks:
                    await asyncio.gather(*tasks, return_exceptions=True)

                # 移除心跳日志,避免刷屏
                # if self.debug:
                #     print(f"[WebSocketServer] 发送心跳到 {len(self.clients)} 个客户端")

            except Exception as e:
                print(f"[WebSocketServer] 心跳循环异常: {e}")

    async def broadcast_status(self, status_dict: dict):
        """
        广播状态更新到所有连接的客户端

        Args:
            status_dict: 状态字典
        """
        if not self.clients:
            return

        try:
            message = json.dumps({
                "type": "status_update",
                "device_id": self.device_id,
                "data": status_dict,
                "timestamp": int(time.time())
            }, ensure_ascii=False)

            tasks = [
                client.send(message)
                for client in self.clients
            ]

            await asyncio.gather(*tasks, return_exceptions=True)

            if self.debug:
                print(f"[WebSocketServer] 广播状态到 {len(self.clients)} 个客户端")

        except Exception as e:
            print(f"[WebSocketServer] 广播状态异常: {e}")

    async def send_to_client(self, websocket: WebSocketServerProtocol,
                             message: dict):
        """
        向特定客户端发送消息

        Args:
            websocket: 目标连接
            message: 消息字典
        """
        try:
            msg_str = json.dumps(message, ensure_ascii=False)
            await websocket.send(msg_str)
        except Exception as e:
            print(f"[WebSocketServer] 发送消息失败: {e}")

    def get_client_count(self) -> int:
        """获取连接的客户端数"""
        return len(self.clients)

    def update_servo_state(self, servo_id: int, servo_type: str, position: int):
        """
        更新舵机状态

        Args:
            servo_id: 舵机 ID
            servo_type: 舵机类型 ("bus" 或 "pca")
            position: 当前位置
        """
        self.handler.update_servo_state(servo_id, servo_type, position)

    def update_system_state(self, **kwargs):
        """更新系统状态"""
        self.handler.update_system_state(**kwargs)

    async def _handle_register(self, websocket: WebSocketServerProtocol, data: dict):
        """
        处理客户端注册（兼容前端协议）

        Args:
            websocket: 客户端连接
            data: 注册消息 {"type": "register", "name": "客户端名称"}
        """
        import uuid

        client_name = data.get("name", f"client_{len(self.client_info) + 1}")
        client_id = str(uuid.uuid4())

        # 保存客户端信息
        self.client_info[websocket] = {
            "id": client_id,
            "name": client_name
        }

        # 获取在线用户列表
        online_users = self.get_online_users()

        # 返回兼容前端的响应（type: "connected"）
        response = {
            "type": "connected",            # 前端期待的类型！
            "clientName": client_name,
            "onlineClients": online_users,  # 在线用户列表
            "device_id": self.device_id,
            "timestamp": int(time.time())
        }

        await websocket.send(json.dumps(response, ensure_ascii=False))
        print(f"[WebSocketServer] 客户端已注册: {client_name} (ID: {client_id})")

        # 广播更新后的用户列表给所有其他客户端
        await self.broadcast_user_list()

    def get_online_users(self) -> list:
        """
        获取在线用户列表（包括 ROS2 设备本身）

        Returns:
            list: [{"id": str, "name": str}, ...]
        """
        # 始终包含 ROS2 机器人设备
        users = [self.robot_device]

        # 添加所有连接的客户端
        users.extend([
            {"id": info["id"], "name": info["name"]}
            for info in self.client_info.values()
        ])

        return users

    async def broadcast_user_list(self):
        """广播在线用户列表给所有客户端"""
        if not self.clients:
            return

        online_users = self.get_online_users()

        message = json.dumps({
            "type": "broadcast",
            "content": "用户列表更新",
            "onlineUsers": online_users,
            "timestamp": int(time.time())
        }, ensure_ascii=False)

        tasks = [
            client.send(message)
            for client in self.clients
        ]

        await asyncio.gather(*tasks, return_exceptions=True)

        if self.debug:
            print(f"[WebSocketServer] 广播用户列表: {len(online_users)} 个在线用户")

    async def _handle_private_to_robot(self, websocket: WebSocketServerProtocol, data: dict):
        """
        处理发给 ROS2 机器人的私有消息（舵机控制）

        Args:
            websocket: 客户端连接
            data: 消息数据 {"type": "private", "to": "ros2-robot-device", "content": "..."}
        """
        content = data.get("content", "")

        if self.debug:
            print(f"[WebSocketServer] 收到发给机器人的消息: {content[:100]}")

        # 尝试解析为 JSON（舵机控制命令）
        try:
            servo_cmd = json.loads(content) if isinstance(
                content, str) else content
            await self._handle_servo_control_direct(servo_cmd)

            # 发送确认响应
            ack = {
                "type": "private_ack",
                "status": "processed",
                "device_id": self.device_id,
                "message": "舵机命令已发送",
                "timestamp": int(time.time())
            }
            await websocket.send(json.dumps(ack, ensure_ascii=False))

            if self.debug:
                print(f"[WebSocketServer] 舵机命令已处理: {servo_cmd}")

        except json.JSONDecodeError:
            # 如果不是 JSON，作为普通消息处理
            print(f"[WebSocketServer] 收到非 JSON 消息: {content}")
        except Exception as e:
            print(f"[WebSocketServer] 处理舵机命令失败: {e}")

    async def _handle_servo_control_direct(self, servo_cmd: dict):
        """
        直接处理舵机控制命令并转发到 ROS 2

        Args:
            servo_cmd: 舵机命令字典
        """
        # 使用 handler 的舵机命令回调
        if hasattr(self.handler, 'on_servo_command') and self.handler.on_servo_command:
            try:
                await self.handler.on_servo_command(servo_cmd)
                if self.debug:
                    print(f"[WebSocketServer] 舵机命令已转发到 ROS2: {servo_cmd}")
            except Exception as e:
                print(f"[WebSocketServer] ROS2 命令处理失败: {e}")
        else:
            print(f"[WebSocketServer] 警告：没有注册舵机命令处理器")


async def run_server(host: str = "0.0.0.0", port: int = 9102,
                     device_id: str = "default", debug: bool = False):
    """
    运行 WebSocket 服务器

    Args:
        host: 监听地址
        port: 监听端口
        device_id: 设备 ID
        debug: 是否启用调试
    """
    server = WebSocketBridgeServer(
        host=host,
        port=port,
        device_id=device_id,
        debug=debug
    )

    await server.start()

    try:
        # 保持运行
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("\n[WebSocketServer] 收到退出信号")
    finally:
        await server.stop()


if __name__ == "__main__":
    # 简单的命令行运行
    import argparse

    parser = argparse.ArgumentParser(description="WebSocket Bridge Server")
    parser.add_argument("--host", default="0.0.0.0", help="Server host")
    parser.add_argument("--port", type=int, default=9102, help="Server port")
    parser.add_argument("--device-id", default="default", help="Device ID")
    parser.add_argument("--debug", action="store_true",
                        help="Enable debug mode")

    args = parser.parse_args()

    asyncio.run(run_server(
        host=args.host,
        port=args.port,
        device_id=args.device_id,
        debug=args.debug
    ))
