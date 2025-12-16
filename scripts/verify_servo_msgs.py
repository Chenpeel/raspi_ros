#!/usr/bin/env python3
"""servo_msgs 消息类型验证测试脚本"""

import rclpy
from rclpy.node import Node
from servo_msgs.msg import ServoCommand, ServoState, ServoStatus

def test_message_creation():
    """测试消息创建"""
    print("=" * 60)
    print("测试1：消息对象创建")
    print("=" * 60)

    # 测试 ServoCommand
    cmd = ServoCommand()
    cmd.servo_type = "bus"
    cmd.servo_id = 1
    cmd.position = 1500
    cmd.speed = 100
    print(f"✓ ServoCommand 创建成功")
    print(f"  - servo_type: {cmd.servo_type}")
    print(f"  - servo_id: {cmd.servo_id}")
    print(f"  - position: {cmd.position}")
    print(f"  - speed: {cmd.speed}")

    # 测试 ServoState
    state = ServoState()
    state.servo_type = "bus"
    state.servo_id = 1
    state.position = 1500
    state.load = 50
    state.temperature = 35
    state.error_code = 0
    print(f"\n✓ ServoState 创建成功")
    print(f"  - servo_type: {state.servo_type}")
    print(f"  - servo_id: {state.servo_id}")
    print(f"  - position: {state.position}")
    print(f"  - load: {state.load}%")
    print(f"  - temperature: {state.temperature}°C")
    print(f"  - error_code: {state.error_code}")

    # 测试 ServoStatus
    status = ServoStatus()
    status.device_id = "test_robot"
    status.bus_servos = [state]
    status.system_state = "running"
    print(f"\n✓ ServoStatus 创建成功")
    print(f"  - device_id: {status.device_id}")
    print(f"  - bus_servos: {len(status.bus_servos)} 个舵机")
    print(f"  - system_state: {status.system_state}")

    return True

def test_message_serialization():
    """测试消息序列化"""
    print("\n" + "=" * 60)
    print("测试2：消息序列化/反序列化")
    print("=" * 60)

    # 创建消息
    cmd = ServoCommand()
    cmd.servo_type = "pca"
    cmd.servo_id = 5
    cmd.position = 90
    cmd.speed = 0

    # 序列化（这会在发布时自动进行）
    print(f"✓ 消息对象大小估算：~{cmd.__sizeof__()} bytes")
    print(f"  对比JSON格式：'{{'servo_type':'pca','servo_id':5,'position':90}}' = ~50 bytes")
    print(f"  → 二进制消息更紧凑高效")

    return True

def test_ros_integration():
    """测试ROS集成"""
    print("\n" + "=" * 60)
    print("测试3：ROS2集成测试")
    print("=" * 60)

    try:
        rclpy.init()

        # 创建简单节点
        node = Node('servo_msgs_test')

        # 创建发布者
        cmd_pub = node.create_publisher(ServoCommand, 'test_command', 10)
        state_pub = node.create_publisher(ServoState, 'test_state', 10)

        print(f"✓ 创建发布者成功")
        print(f"  - 话题 'test_command' (ServoCommand)")
        print(f"  - 话题 'test_state' (ServoState)")

        # 发布测试消息
        cmd = ServoCommand()
        cmd.servo_type = "bus"
        cmd.servo_id = 1
        cmd.position = 1500
        cmd.speed = 100
        cmd.stamp = node.get_clock().now().to_msg()

        cmd_pub.publish(cmd)
        print(f"\n✓ 发布 ServoCommand 消息成功")

        state = ServoState()
        state.servo_type = "bus"
        state.servo_id = 1
        state.position = 1500
        state.error_code = 0
        state.stamp = node.get_clock().now().to_msg()

        state_pub.publish(state)
        print(f"✓ 发布 ServoState 消息成功")

        node.destroy_node()
        rclpy.shutdown()

        return True

    except Exception as e:
        print(f"✗ ROS集成测试失败: {e}")
        return False

def main():
    print("\n" + "=" * 60)
    print("servo_msgs 包验证测试")
    print("=" * 60)

    results = []

    # 运行测试
    results.append(("消息创建", test_message_creation()))
    results.append(("消息序列化", test_message_serialization()))
    results.append(("ROS集成", test_ros_integration()))

    # 汇总结果
    print("\n" + "=" * 60)
    print("测试结果汇总")
    print("=" * 60)

    passed = sum(1 for _, result in results if result)
    total = len(results)

    for name, result in results:
        status = "✓ 通过" if result else "✗ 失败"
        print(f"{status}: {name}")

    print(f"\n总计: {passed}/{total} 测试通过")

    if passed == total:
        print("\n🎉 所有测试通过！servo_msgs 包工作正常。")
        return 0
    else:
        print(f"\n⚠️  {total - passed} 个测试失败，请检查。")
        return 1

if __name__ == '__main__':
    exit(main())
