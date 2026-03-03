#!/usr/bin/env bash
# Isaac <-> ROS 联调快捷脚本（ROS 侧）
# 目标：统一命令入口，避免手敲长命令导致参数顺序或环境差异。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CMD="${1:-help}"
if [ $# -gt 0 ]; then
    shift
fi

CONTAINER_NAME="${ROS_CONTAINER_NAME:-ros2}"
COMPOSE_PROFILE="${CO_TEST_COMPOSE_PROFILE:-production}"
COMPOSE_SERVICE="${CO_TEST_COMPOSE_SERVICE:-ros2_servo_prod}"
RATE="${RATE:-20}"
HZ_TIMEOUT_SEC="${HZ_TIMEOUT_SEC:-8}"
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
WS_SETUP_IN_CONTAINER="/root/ros_ws/install/setup.bash"
WS_SETUP_LOCAL="${REPO_ROOT}/install/setup.bash"

log_info() {
    echo "[INFO] $*"
}

log_warn() {
    echo "[WARN] $*" >&2
}

log_error() {
    echo "[ERROR] $*" >&2
}

has_cmd() {
    command -v "$1" >/dev/null 2>&1
}

docker_exec() {
    local inner_cmd="$1"
    if [ -t 0 ] && [ -t 1 ]; then
        docker exec -it "${CONTAINER_NAME}" bash -lc "${inner_cmd}"
    else
        docker exec -i "${CONTAINER_NAME}" bash -lc "${inner_cmd}"
    fi
}

docker_exec_ros() {
    local escaped=""
    printf -v escaped "%q " "$@"
    docker_exec "source ${ROS_SETUP} && [ -f ${WS_SETUP_IN_CONTAINER} ] && source ${WS_SETUP_IN_CONTAINER}; ${escaped}"
}

setup_ros_env_local() {
    if [ -f "${ROS_SETUP}" ]; then
        # shellcheck disable=SC1090
        source "${ROS_SETUP}"
    else
        log_error "本机缺少 ROS 环境: ${ROS_SETUP}"
        return 1
    fi

    if [ -f "${WS_SETUP_LOCAL}" ]; then
        # shellcheck disable=SC1090
        source "${WS_SETUP_LOCAL}"
    elif [ -f "${WS_SETUP_IN_CONTAINER}" ]; then
        # 兼容在容器内直接运行
        # shellcheck disable=SC1090
        source "${WS_SETUP_IN_CONTAINER}"
    else
        log_warn "未找到工作空间 setup.bash（继续执行，仅依赖系统ROS）"
    fi

    # 固化 FastDDS 相关环境，降低跨 shell 差异
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
    export ROS_DOMAIN_ID="${DOMAIN_ID}"
    export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
    unset ROS_STATIC_PEERS || true
    export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
    export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
}

print_ros_env() {
    echo "[INFO] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
    echo "[INFO] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
    echo "[INFO] ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<unset>}"
    echo "[INFO] ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-<unset>}"
    echo "[INFO] FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-<unset>}"
    echo "[INFO] ROS_STATIC_PEERS=${ROS_STATIC_PEERS:-<unset>}"
}

run_ros_cmd() {
    if has_cmd ros2 && [ -f "${ROS_SETUP}" ]; then
        setup_ros_env_local
        print_ros_env
        "$@"
    else
        docker_exec_ros "$@"
    fi
}

run_ros_shell() {
    local shell_cmd="$1"
    if has_cmd ros2 && [ -f "${ROS_SETUP}" ]; then
        setup_ros_env_local
        print_ros_env
        eval "${shell_cmd}"
    else
        docker_exec "source ${ROS_SETUP} && [ -f ${WS_SETUP_IN_CONTAINER} ] && source ${WS_SETUP_IN_CONTAINER}; ${shell_cmd}"
    fi
}

topic_hz_once() {
    local topic="$1"
    run_ros_shell "timeout ${HZ_TIMEOUT_SEC}s ros2 topic hz ${topic} || true"
}

usage() {
    cat <<'USAGE'
用法:
  ./scripts/sim_bridge_co_test.sh up            # 启动 ROS Docker（production profile）
  ./scripts/sim_bridge_co_test.sh launch        # 启动 full_system（C++桥开，Python桥关）
  ./scripts/sim_bridge_co_test.sh down          # 停止 Docker

  ./scripts/sim_bridge_co_test.sh hz_all        # 采集四条关键话题 hz（每条默认 8s）
  ./scripts/sim_bridge_co_test.sh hz TOPIC      # 采集单条话题 hz（每条默认 8s）
  ./scripts/sim_bridge_co_test.sh diag          # 最小诊断（topic/node/info）

  ./scripts/sim_bridge_co_test.sh pub_fb        # 持续发布 /sim/joint_state_fb
  ./scripts/sim_bridge_co_test.sh echo_cmd      # 监听 /sim/joint_cmd
  ./scripts/sim_bridge_co_test.sh info_fb       # 查看 /sim/joint_state_fb 端点
  ./scripts/sim_bridge_co_test.sh info_cmd      # 查看 /sim/joint_cmd 端点
  ./scripts/sim_bridge_co_test.sh info_servo    # 查看 /servo/{command,state} 端点

环境变量:
  ROS_CONTAINER_NAME      默认: ros2
  CO_TEST_COMPOSE_PROFILE 默认: production
  CO_TEST_COMPOSE_SERVICE 默认: ros2_servo_prod
  RATE                    默认: 20（pub_fb 发布频率）
  HZ_TIMEOUT_SEC          默认: 8（hz 采样时长）
  ROS_DOMAIN_ID           默认: 0
USAGE
}

case "${CMD}" in
    up)
        (cd "${REPO_ROOT}" && docker compose --profile "${COMPOSE_PROFILE}" up -d "${COMPOSE_SERVICE}")
        ;;

    launch)
        run_ros_cmd ros2 launch websocket_bridge full_system.launch.py \
            enable_sim_cpp_bridge:=true \
            enable_isaac_bridge:=false
        ;;

    down)
        (cd "${REPO_ROOT}" && docker compose --profile "${COMPOSE_PROFILE}" down)
        ;;

    hz_all)
        log_info "检查 /sim/joint_cmd"
        topic_hz_once "/sim/joint_cmd"
        log_info "检查 /servo/command"
        topic_hz_once "/servo/command"
        log_info "检查 /servo/state"
        topic_hz_once "/servo/state"
        log_info "检查 /sim/joint_state_fb"
        topic_hz_once "/sim/joint_state_fb"
        ;;

    hz)
        if [ $# -lt 1 ]; then
            log_error "请指定 topic，例如: ./scripts/sim_bridge_co_test.sh hz /sim/joint_cmd"
            exit 1
        fi
        topic_hz_once "$1"
        ;;

    diag)
        run_ros_cmd ros2 topic list
        run_ros_cmd ros2 node list
        run_ros_cmd ros2 topic info /sim/joint_cmd -v
        run_ros_cmd ros2 topic info /sim/joint_state_fb -v
        run_ros_cmd ros2 topic info /servo/command -v
        run_ros_cmd ros2 topic info /servo/state -v
        ;;

    pub_fb)
        run_ros_cmd ros2 topic pub --qos-reliability reliable --qos-durability volatile \
            -r "${RATE}" /sim/joint_state_fb std_msgs/msg/Float32MultiArray -- \
            "{data: [0.0, 0.05, -0.05, 0.1, -0.1, 0.15, -0.15, 0.2, -0.2, 0.25, -0.25, 0.3, -0.3, 0.35, -0.35, 0.4]}"
        ;;

    echo_cmd)
        run_ros_cmd ros2 topic echo /sim/joint_cmd std_msgs/msg/Float32MultiArray
        ;;

    info_fb)
        run_ros_cmd ros2 topic info /sim/joint_state_fb -v
        ;;

    info_cmd)
        run_ros_cmd ros2 topic info /sim/joint_cmd -v
        ;;

    info_servo)
        run_ros_cmd ros2 topic info /servo/command -v
        run_ros_cmd ros2 topic info /servo/state -v
        ;;

    help|-h|--help)
        usage
        ;;

    *)
        log_error "未知命令: ${CMD}"
        usage
        exit 1
        ;;
esac
