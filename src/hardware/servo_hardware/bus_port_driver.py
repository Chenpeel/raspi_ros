"""多协议总线舵机串口驱动节点。

职责：
1. 独占一个串口端口；
2. 接收上层路由后的协议命令（众灵/幻尔）并发送；
3. 提供读角度服务接口，返回统一脉宽语义（500~2500us）。
"""

import threading
import time
from typing import Dict, Optional, Set

import rclpy
import serial
from rclpy.node import Node
from servo_msgs.msg import ServoCommand, ServoState
from servo_msgs.srv import ReadServoPosition

from .protocols import BusServoProtocol, LXBusServoProtocol, ZLBusServoProtocol


class BusPortDriver(Node):
    """多协议总线舵机串口驱动。"""

    def __init__(self):
        super().__init__("bus_port_driver")

        self.declare_parameter("port", "/dev/ttyAMA0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("timeout", 0.1)
        self.declare_parameter("read_timeout", 0.15)
        self.declare_parameter("default_speed", 100)
        self.declare_parameter("debug", False)
        self.declare_parameter("log_id", True)
        self.declare_parameter("zl_servo_ids", [0])
        self.declare_parameter("lx_servo_ids", [0])

        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.read_timeout = float(self.get_parameter("read_timeout").value)
        self.default_speed = int(self.get_parameter("default_speed").value)
        self.debug = bool(self.get_parameter("debug").value)
        self.log_id = bool(self.get_parameter("log_id").value)

        zl_ids = self.get_parameter("zl_servo_ids").value
        lx_ids = self.get_parameter("lx_servo_ids").value
        self.zl_servo_ids: Set[int] = {int(x) for x in zl_ids if int(x) > 0}
        self.lx_servo_ids: Set[int] = {int(x) for x in lx_ids if int(x) > 0}

        self.protocols: Dict[str, BusServoProtocol] = {
            "zl": ZLBusServoProtocol(),
            "lx": LXBusServoProtocol(),
        }

        self.ser: Optional[serial.Serial] = None
        self.lock = threading.Lock()
        if not self._init_serial():
            self.get_logger().error(f"串口初始化失败: {self.port}")

        self.state_pub = self.create_publisher(ServoState, "~/state", 10)
        self.zl_sub = self.create_subscription(
            ServoCommand,
            "~/command_zl",
            self._on_zl_command,
            10,
        )
        self.lx_sub = self.create_subscription(
            ServoCommand,
            "~/command_lx",
            self._on_lx_command,
            10,
        )
        self.read_position_srv = self.create_service(
            ReadServoPosition,
            "~/read_position",
            self._handle_read_position,
        )

        self.get_logger().info(
            "bus_port_driver 已启动: "
            f"{self.port}@{self.baudrate} "
            f"(zl_ids={sorted(self.zl_servo_ids) or 'ALL'}, "
            f"lx_ids={sorted(self.lx_servo_ids) or 'ALL'})"
        )

    def _init_serial(self) -> bool:
        try:
            if self.ser is None:
                self.ser = serial.Serial(
                    self.port,
                    self.baudrate,
                    timeout=self.timeout,
                )
            elif not self.ser.is_open:
                self.ser.open()
            return True
        except Exception as exc:
            self.get_logger().error(f"打开串口失败: {exc}")
            return False

    def _is_bus_command(self, msg: ServoCommand) -> bool:
        servo_type = str(msg.servo_type or "").strip().lower()
        return servo_type in ("", "bus", "bus_servo", "bus_zl", "bus_lx", "zl", "lx")

    def _is_allowed_id(self, protocol: str, servo_id: int) -> bool:
        if protocol == "zl":
            return (not self.zl_servo_ids) or (servo_id in self.zl_servo_ids)
        if protocol == "lx":
            return (not self.lx_servo_ids) or (servo_id in self.lx_servo_ids)
        return False

    def _position_to_pulse(self, position: int) -> int:
        pos = int(position)
        if 0 <= pos <= 180:
            return int(round(500 + (pos / 180.0) * 2000.0))
        if 500 <= pos <= 2500:
            return pos
        if 0 <= pos <= 1000:
            # 兼容将幻尔单位值直接传入的场景
            return int(round(500 + (pos / 1000.0) * 2000.0))

        if self.debug:
            self.get_logger().warn(f"非常规位置值: {pos}，将限幅到[500,2500]")
        return max(500, min(2500, pos))

    @staticmethod
    def _pulse_to_lx_unit(pulse: int) -> int:
        pulse = max(500, min(2500, int(pulse)))
        return int(round((pulse - 500) * 1000.0 / 2000.0))

    @staticmethod
    def _lx_unit_to_pulse(unit: int) -> int:
        unit = max(0, min(1000, int(unit)))
        return int(round(500 + unit * 2000.0 / 1000.0))

    def _publish_state(self, servo_id: int, pulse: int, error_code: int):
        msg = ServoState()
        msg.servo_type = "bus"
        msg.servo_id = int(servo_id)
        msg.position = int(max(0, min(65535, pulse)))
        msg.load = -1
        msg.temperature = -1
        msg.error_code = int(max(0, min(255, error_code)))
        msg.stamp = self.get_clock().now().to_msg()
        self.state_pub.publish(msg)

    def _write_frame(self, frame: bytes) -> bool:
        if not self._init_serial():
            return False
        with self.lock:
            try:
                self.ser.write(frame)
                if self.debug:
                    self.get_logger().debug(f"发送字节: {frame.hex(' ')}")
                return True
            except Exception as exc:
                self.get_logger().error(f"串口写入失败: {exc}")
                return False

    def _execute_move(self, protocol: str, servo_id: int, position: int, speed: int):
        if not self._is_allowed_id(protocol, servo_id):
            if self.debug:
                self.get_logger().debug(
                    f"忽略ID={servo_id}的{protocol}命令（不在本节点负责范围）"
                )
            return

        pulse = self._position_to_pulse(position)
        speed = max(0, min(30000, int(speed)))

        if protocol == "zl":
            proto_pos = pulse
        else:
            proto_pos = self._pulse_to_lx_unit(pulse)

        frame = self.protocols[protocol].encode_move_command(
            servo_id=servo_id,
            position=proto_pos,
            speed=speed,
        )
        success = self._write_frame(frame)
        self._publish_state(servo_id=servo_id, pulse=pulse, error_code=0 if success else 1)

        if self.log_id:
            self.get_logger().info(
                f"[Send/{protocol}] ID={servo_id} POS={proto_pos} SPEED={speed}"
            )

    def _on_zl_command(self, msg: ServoCommand):
        try:
            if not self._is_bus_command(msg):
                return
            speed = msg.speed if msg.speed > 0 else self.default_speed
            self._execute_move(
                protocol="zl",
                servo_id=int(msg.servo_id),
                position=int(msg.position),
                speed=int(speed),
            )
        except Exception as exc:
            self.get_logger().error(f"处理众灵命令失败: {exc}")

    def _on_lx_command(self, msg: ServoCommand):
        try:
            if not self._is_bus_command(msg):
                return
            speed = msg.speed if msg.speed > 0 else self.default_speed
            self._execute_move(
                protocol="lx",
                servo_id=int(msg.servo_id),
                position=int(msg.position),
                speed=int(speed),
            )
        except Exception as exc:
            self.get_logger().error(f"处理幻尔命令失败: {exc}")

    def _read_position_once(self, protocol: str, servo_id: int) -> Optional[int]:
        if protocol not in self.protocols:
            return None
        if not self._init_serial():
            return None

        proto = self.protocols[protocol]
        cmd = proto.encode_read_position_command(servo_id)
        deadline = time.monotonic() + max(0.02, self.read_timeout)
        buffer = bytearray()

        with self.lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(cmd)
                if self.debug:
                    self.get_logger().debug(f"[Read/{protocol}] -> {cmd.hex(' ')}")

                while time.monotonic() < deadline:
                    waiting = self.ser.in_waiting
                    if waiting > 0:
                        buffer.extend(self.ser.read(waiting))
                        pos = proto.decode_position_response(
                            bytes(buffer),
                            expected_servo_id=servo_id,
                        )
                        if pos is not None:
                            return int(pos)
                    time.sleep(0.005)
            except Exception as exc:
                self.get_logger().error(f"读取位置失败({protocol}): {exc}")
                return None
        return None

    def _pick_protocol_order(self, requested: str, servo_id: int):
        requested = (requested or "").strip().lower()
        if requested in ("zl", "lx"):
            return [requested]

        # auto: 优先根据ID归属推断
        order = []
        if self._is_allowed_id("lx", servo_id):
            order.append("lx")
        if self._is_allowed_id("zl", servo_id):
            order.append("zl")

        if not order:
            order = ["lx", "zl"]
        return order

    def _handle_read_position(self, request: ReadServoPosition.Request, response: ReadServoPosition.Response):
        servo_id = int(request.servo_id)
        protocols = self._pick_protocol_order(request.protocol, servo_id)

        for proto_name in protocols:
            pos = self._read_position_once(proto_name, servo_id)
            if pos is None:
                continue

            pulse = int(pos) if proto_name == "zl" else self._lx_unit_to_pulse(pos)
            self._publish_state(servo_id=servo_id, pulse=pulse, error_code=0)

            response.success = True
            response.position = int(max(0, min(65535, pulse)))
            response.protocol = proto_name
            response.error_code = 0
            response.message = "ok"
            response.stamp = self.get_clock().now().to_msg()

            if self.log_id:
                self.get_logger().info(
                    f"[Read/{proto_name}] ID={servo_id} RAW={pos} PULSE={pulse}"
                )
            return response

        response.success = False
        response.position = 0
        response.protocol = ""
        response.error_code = 1
        response.message = f"no response from protocols={protocols}"
        response.stamp = self.get_clock().now().to_msg()
        return response

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                self.get_logger().info("串口已关闭")
            except Exception as exc:
                self.get_logger().error(f"关闭串口失败: {exc}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BusPortDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
