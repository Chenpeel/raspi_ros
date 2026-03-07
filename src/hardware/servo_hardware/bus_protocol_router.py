"""总线协议路由节点。

功能：
1. 订阅 /servo/command；
2. 按 ID -> 端口 -> 协议 路由到 /bus_port_driver_x/command_{zl|lx}；
3. 聚合 /bus_port_driver_x/state 到 /servo/state；
4. 提供全局读角度服务 /servo/read_position；
5. 启动时与运行时按需探测未知协议并写入缓存文件。
"""

import json
import os
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from servo_msgs.msg import ServoCommand, ServoState
from servo_msgs.srv import ReadServoPosition

from .bus_protocol_registry import (
    ProtocolRegistry,
    load_manual_protocol_map,
)


def _to_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(int(value))
    text = str(value).strip().lower()
    return text in ("1", "true", "yes", "y", "on")


class BusProtocolRouter(Node):
    """协议路由节点。"""

    def __init__(self):
        super().__init__("bus_protocol_router")

        self.declare_parameter("bus_map_file", "")
        self.declare_parameter("protocol_cache_file", "")
        self.declare_parameter("manual_protocol_map_file", "")
        self.declare_parameter("lx_id_ranges", "21-34")
        self.declare_parameter("zl_id_ranges", "35-43")
        self.declare_parameter("probe_on_startup", True)
        self.declare_parameter("probe_on_unknown_command", True)
        self.declare_parameter("probe_timeout_sec", 0.2)
        self.declare_parameter("probe_retry_interval_sec", 3.0)
        self.declare_parameter("read_service_timeout_sec", 0.35)
        self.declare_parameter("probe_wait_service_sec", 6.0)
        self.declare_parameter("debug", False)

        self.bus_map_file = str(self.get_parameter("bus_map_file").value).strip()
        self.protocol_cache_file = str(self.get_parameter("protocol_cache_file").value).strip()
        self.manual_protocol_map_file = str(
            self.get_parameter("manual_protocol_map_file").value
        ).strip()
        self.lx_id_ranges = self.get_parameter("lx_id_ranges").value
        self.zl_id_ranges = self.get_parameter("zl_id_ranges").value
        self.probe_on_startup = _to_bool(self.get_parameter("probe_on_startup").value)
        self.probe_on_unknown_command = _to_bool(
            self.get_parameter("probe_on_unknown_command").value
        )
        self.probe_timeout_sec = float(self.get_parameter("probe_timeout_sec").value)
        self.probe_retry_interval_sec = float(
            self.get_parameter("probe_retry_interval_sec").value
        )
        self.read_service_timeout_sec = float(
            self.get_parameter("read_service_timeout_sec").value
        )
        self.probe_wait_service_sec = float(self.get_parameter("probe_wait_service_sec").value)
        self.debug = _to_bool(self.get_parameter("debug").value)
        self.last_probe_attempt: Dict[int, float] = {}

        self.port_items = self._load_bus_map(self.bus_map_file)
        self.id_to_port: Dict[int, str] = {}
        self.port_to_node_name: Dict[str, str] = {}
        for idx, (port, ids) in enumerate(self.port_items):
            self.port_to_node_name[port] = f"bus_port_driver_{idx}"
            for sid in ids:
                self.id_to_port[int(sid)] = port

        manual_map = load_manual_protocol_map(self.manual_protocol_map_file)
        self.registry = ProtocolRegistry(
            cache_file=self.protocol_cache_file,
            manual_map=manual_map,
            lx_ranges=self.lx_id_ranges,
            zl_ranges=self.zl_id_ranges,
        )

        # 全局入口和出口
        self.command_sub = self.create_subscription(
            ServoCommand,
            "/servo/command",
            self._on_command,
            10,
        )
        self.read_position_srv = self.create_service(
            ReadServoPosition,
            "/servo/read_position",
            self._handle_read_position,
        )
        self.state_pub = self.create_publisher(
            ServoState,
            "/servo/state",
            10,
        )

        # 端口级发布器/订阅器/服务客户端
        self.route_publishers: Dict[Tuple[str, str], object] = {}
        self.read_clients: Dict[str, object] = {}

        for port, _ in self.port_items:
            node_name = self.port_to_node_name[port]
            topic_zl = f"/{node_name}/command_zl"
            topic_lx = f"/{node_name}/command_lx"
            state_topic = f"/{node_name}/state"
            service_name = f"/{node_name}/read_position"

            self.route_publishers[(port, "zl")] = self.create_publisher(
                ServoCommand,
                topic_zl,
                10,
            )
            self.route_publishers[(port, "lx")] = self.create_publisher(
                ServoCommand,
                topic_lx,
                10,
            )
            self.create_subscription(
                ServoState,
                state_topic,
                self._on_state,
                10,
            )
            self.read_clients[port] = self.create_client(
                ReadServoPosition,
                service_name,
            )

        self.get_logger().info(
            "bus_protocol_router 启动完成: "
            f"ports={[p for p, _ in self.port_items]}, "
            f"cache={self.protocol_cache_file or 'disabled'}"
        )

    def _load_bus_map(self, path: str) -> List[Tuple[str, List[int]]]:
        path = str(path or "").strip()
        if not path:
            raise RuntimeError("bus_map_file 不能为空")
        if not os.path.exists(path):
            raise RuntimeError(f"bus_map_file 不存在: {path}")

        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            raise RuntimeError(f"bus_map_file 格式错误: {path}")

        result: List[Tuple[str, List[int]]] = []
        for port, ids in data.items():
            if not isinstance(ids, list):
                continue
            clean_ids = []
            for sid in ids:
                try:
                    clean_ids.append(int(sid))
                except (TypeError, ValueError):
                    continue
            result.append((str(port), clean_ids))
        return result

    def _on_state(self, msg: ServoState):
        # 驱动层状态统一汇聚到全局话题
        self.state_pub.publish(msg)

    @staticmethod
    def _is_bus_command(msg: ServoCommand) -> bool:
        servo_type = str(msg.servo_type or "").strip().lower()
        return servo_type in ("", "bus", "bus_servo", "bus_zl", "bus_lx", "zl", "lx")

    def _on_command(self, msg: ServoCommand):
        if not self._is_bus_command(msg):
            # 非总线命令忽略；PCA 仍可走原路径（后续可扩展）
            return

        servo_id = int(msg.servo_id)
        port = self.id_to_port.get(servo_id)
        if port is None:
            self.get_logger().warn(f"未找到ID={servo_id}的端口映射，忽略命令")
            return

        protocol, source = self.registry.get_protocol(servo_id)
        if protocol is None:
            protocol = self._probe_unknown_for_command(servo_id)
            source = "probe"
        if protocol is None:
            self.get_logger().warn(f"ID={servo_id}协议未知，命令未转发")
            return

        if not self._publish_to_route(port=port, protocol=protocol, msg=msg):
            return
        if self.debug:
            self.get_logger().info(
                f"[route] id={servo_id} -> port={port} protocol={protocol} source={source}"
            )

    def _publish_to_route(self, port: str, protocol: str, msg: ServoCommand) -> bool:
        publisher = self.route_publishers.get((port, protocol))
        if publisher is None:
            self.get_logger().error(
                f"未找到路由发布器: port={port}, protocol={protocol}, id={msg.servo_id}"
            )
            return False
        publisher.publish(msg)
        return True

    def _probe_unknown_for_command(self, servo_id: int) -> Optional[str]:
        if not self.probe_on_unknown_command:
            return None
        now = time.monotonic()
        last = self.last_probe_attempt.get(int(servo_id), 0.0)
        if now - last < max(0.1, self.probe_retry_interval_sec):
            return None
        self.last_probe_attempt[int(servo_id)] = now

        protocol = self._probe_servo_id(servo_id)
        if protocol is None:
            return None
        self._update_registry(servo_id=servo_id, protocol=protocol, source="probe")
        self.get_logger().info(f"ID={servo_id}运行时探测成功: {protocol}")
        return protocol

    def _update_registry(self, servo_id: int, protocol: str, source: str):
        self.registry.set_protocol(servo_id, protocol, source=source)
        try:
            self.registry.save_cache()
        except Exception as exc:
            self.get_logger().warn(f"写入协议缓存失败: {exc}")

    def run_startup_probe(self):
        """启动时探测未知协议ID，并落盘缓存。"""
        if not self.probe_on_startup:
            return

        servo_ids = sorted(self.id_to_port.keys())
        unknown_ids = self.registry.unresolved_ids(servo_ids)
        if not unknown_ids:
            self.get_logger().info("协议探测跳过：当前无未知ID")
            return

        self.get_logger().info(f"开始协议探测，未知ID列表: {unknown_ids}")
        updated = 0
        for servo_id in unknown_ids:
            protocol = self._probe_servo_id(servo_id)
            if protocol is None:
                self.get_logger().warn(f"ID={servo_id}探测失败")
                continue
            self.registry.set_protocol(servo_id, protocol, source="probe")
            updated += 1
            self.get_logger().info(f"ID={servo_id}探测成功: {protocol}")

        if updated > 0:
            try:
                self.registry.save_cache()
            except Exception as exc:
                self.get_logger().warn(f"写入协议缓存失败: {exc}")
            self.get_logger().info(
                f"协议探测完成，新增缓存{updated}条 -> {self.protocol_cache_file}"
            )
        else:
            self.get_logger().warn("协议探测完成，但没有新结果")

    def _probe_servo_id(
        self,
        servo_id: int,
        order: Optional[List[str]] = None,
    ) -> Optional[str]:
        port = self.id_to_port.get(int(servo_id))
        if port is None:
            return None
        candidates = order or ["lx", "zl"]
        for protocol in candidates:
            resp = self._call_read_position(
                port=port,
                servo_id=servo_id,
                protocol=protocol,
                timeout_sec=self.probe_timeout_sec,
            )
            if resp is None:
                continue
            if bool(resp.success):
                return protocol
        return None

    def _call_read_position(
        self,
        port: str,
        servo_id: int,
        protocol: str,
        timeout_sec: float,
    ) -> Optional[ReadServoPosition.Response]:
        client = self.read_clients.get(port)
        if client is None:
            return None
        if not client.wait_for_service(timeout_sec=max(0.1, self.probe_wait_service_sec)):
            self.get_logger().warn(f"端口{port}服务未就绪(/read_position)")
            return None

        req = ReadServoPosition.Request()
        req.servo_id = int(servo_id)
        req.protocol = str(protocol)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=max(0.05, float(timeout_sec)),
        )
        if not future.done():
            return None
        return future.result()

    def _resolve_protocol_order(self, requested: str, servo_id: int) -> List[str]:
        requested = str(requested or "").strip().lower()
        if requested in ("zl", "lx"):
            return [requested]

        protocol, _ = self.registry.get_protocol(servo_id)
        if protocol in ("zl", "lx"):
            other = "zl" if protocol == "lx" else "lx"
            return [protocol, other]
        return ["lx", "zl"]

    def _handle_read_position(
        self,
        request: ReadServoPosition.Request,
        response: ReadServoPosition.Response,
    ):
        servo_id = int(request.servo_id)
        requested = str(request.protocol or "").strip().lower()
        port = self.id_to_port.get(servo_id)
        if port is None:
            response.success = False
            response.position = 0
            response.protocol = ""
            response.error_code = 2
            response.message = f"id {servo_id} not found in bus map"
            response.stamp = self.get_clock().now().to_msg()
            return response

        protocols = self._resolve_protocol_order(requested, servo_id)
        last_message = ""
        for protocol in protocols:
            resp = self._call_read_position(
                port=port,
                servo_id=servo_id,
                protocol=protocol,
                timeout_sec=self.read_service_timeout_sec,
            )
            if resp is None:
                last_message = f"timeout protocol={protocol}"
                continue
            if not bool(resp.success):
                last_message = str(resp.message)
                continue

            self._update_registry(servo_id=servo_id, protocol=protocol, source="read_service")
            response.success = True
            response.position = int(resp.position)
            response.protocol = protocol
            response.error_code = 0
            response.message = "ok"
            response.stamp = self.get_clock().now().to_msg()
            return response

        response.success = False
        response.position = 0
        response.protocol = ""
        response.error_code = 1
        response.message = last_message or "read failed"
        response.stamp = self.get_clock().now().to_msg()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BusProtocolRouter()
    try:
        node.run_startup_probe()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
