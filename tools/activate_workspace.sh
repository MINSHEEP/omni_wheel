#!/usr/bin/env bash
# 加载当前脚本所属的 Kiwi 工作区，并确认生产包没有被其他 colcon overlay 覆盖。
#
# 用法：source /path/to/ros2_ws/tools/activate_workspace.sh
# 必须使用 source；直接执行脚本只会修改子进程环境，返回终端后不会生效。

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "错误：请使用 source tools/activate_workspace.sh 加载工作区。" >&2
  exit 2
fi

KIWI_WORKSPACE_ROOT="$({ cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd; })"
KIWI_INSTALL_SETUP="${KIWI_WORKSPACE_ROOT}/install/local_setup.bash"
# Humble 的 setup 会将未指定的 ROS_LOCALHOST_ONLY 设为 0，先保留用户选择。
KIWI_REQUESTED_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "错误：未找到 /opt/ros/humble/setup.bash。" >&2
  return 1
fi
if [[ ! -r "${KIWI_INSTALL_SETUP}" ]]; then
  echo "错误：工作区尚未构建，缺少 ${KIWI_INSTALL_SETUP}。" >&2
  return 1
fi

# 去除此前加载的其他 colcon 工作区；保留系统路径和本工作区。
# 使用 local_setup，避免 setup.bash 的构建时前缀链重新加载旧工作区。
_kiwi_clear_other_workspaces() {
  local kiwi_prefix kiwi_var kiwi_entry kiwi_kept
  local -a kiwi_old_prefixes=() kiwi_entries=()
  local IFS=:
  for kiwi_var in AMENT_PREFIX_PATH COLCON_PREFIX_PATH; do
    read -r -a kiwi_entries <<< "${!kiwi_var:-}"
    for kiwi_prefix in "${kiwi_entries[@]}"; do
      case "${kiwi_prefix}" in
        ""|/opt/ros/humble|/opt/ros/humble/*|"${KIWI_WORKSPACE_ROOT}/install"|"${KIWI_WORKSPACE_ROOT}/install/"*) ;;
        *) kiwi_old_prefixes+=("${kiwi_prefix%/}") ;;
      esac
    done
  done
  for kiwi_var in AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH \
      LD_LIBRARY_PATH PYTHONPATH PATH PKG_CONFIG_PATH ROS_PACKAGE_PATH; do
    [[ -v ${kiwi_var} ]] || continue
    read -r -a kiwi_entries <<< "${!kiwi_var}"
    kiwi_kept=""
    for kiwi_entry in "${kiwi_entries[@]}"; do
      for kiwi_prefix in "${kiwi_old_prefixes[@]}"; do
        case "${kiwi_entry}" in
          "${kiwi_prefix}"|"${kiwi_prefix}/"*) kiwi_entry=""; break ;;
        esac
      done
      [[ -n ${kiwi_entry} ]] || continue
      kiwi_kept+="${kiwi_kept:+:}${kiwi_entry}"
    done
    export "${kiwi_var}=${kiwi_kept}"
  done
}
_kiwi_clear_other_workspaces
unset -f _kiwi_clear_other_workspaces
source /opt/ros/humble/setup.bash || return 1
source "${KIWI_INSTALL_SETUP}" || return 1

# 与 kiwi_web/config/web.yaml 的默认域一致，隔离局域网内其他机器的 /tf、/scan。
# 显式设置 ROS_DOMAIN_ID 的用户可连接到自己指定的 ROS 域。
if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  export ROS_DOMAIN_ID=42
elif [[ "${ROS_DOMAIN_ID}" != 42 ]]; then
  echo "提示：当前 ROS_DOMAIN_ID=${ROS_DOMAIN_ID}，Kiwi 默认域为 42；请确认没有接入其他机器的同名 TF。" >&2
fi

# 建图、录包和查询终端默认在本机发现；跨机 ROS 通信时所有终端显式设为 0。
# 切换此值后应在新环境运行 ros2 daemon stop，清除旧发现缓存。
export ROS_LOCALHOST_ONLY="${KIWI_REQUESTED_LOCALHOST_ONLY}"

for KIWI_PACKAGE_NAME in kiwi_platform kiwi_base kiwi_slam kiwi_navigation livox_ros_driver2 fast_lio; do
  KIWI_PACKAGE_PREFIX="$(ros2 pkg prefix "${KIWI_PACKAGE_NAME}" 2>/dev/null)" || {
    echo "错误：当前 ament 索引找不到 ${KIWI_PACKAGE_NAME}。" >&2
    return 1
  }
  case "${KIWI_PACKAGE_PREFIX}" in
    "${KIWI_WORKSPACE_ROOT}/install"|"${KIWI_WORKSPACE_ROOT}/install/"*) ;;
    *)
      echo "错误：${KIWI_PACKAGE_NAME} 来自其他工作区：${KIWI_PACKAGE_PREFIX}" >&2
      echo "期望来源：${KIWI_WORKSPACE_ROOT}/install" >&2
      return 1
      ;;
  esac
done

KIWI_KEYBOARD_ENTRY="${KIWI_WORKSPACE_ROOT}/install/kiwi_base/lib/kiwi_base/kiwi_teleop_keyboard"
if [[ -e "${KIWI_KEYBOARD_ENTRY}" && ! -x "${KIWI_KEYBOARD_ENTRY}" ]]; then
  echo "警告：键盘入口不可执行；请重新构建 kiwi_base，旧版安装可运行 chmod +x ${KIWI_WORKSPACE_ROOT}/src/kiwi_base/tools/kiwi_teleop_keyboard" >&2
fi
for KIWI_SCRIPT_NAME in kiwi_mapping_scan_gate kiwi_slam_monitor; do
  KIWI_SCRIPT_ENTRY="${KIWI_WORKSPACE_ROOT}/install/kiwi_slam/lib/kiwi_slam/${KIWI_SCRIPT_NAME}"
  if [[ -e "${KIWI_SCRIPT_ENTRY}" && ! -x "${KIWI_SCRIPT_ENTRY}" ]]; then
    echo "警告：${KIWI_SCRIPT_NAME} 不可执行；请重新构建 kiwi_slam，旧版安装可运行 chmod +x ${KIWI_WORKSPACE_ROOT}/src/kiwi_slam/scripts/${KIWI_SCRIPT_NAME}" >&2
  fi
done

export KIWI_WORKSPACE_ROOT
unset KIWI_INSTALL_SETUP KIWI_REQUESTED_LOCALHOST_ONLY KIWI_PACKAGE_NAME KIWI_PACKAGE_PREFIX KIWI_KEYBOARD_ENTRY KIWI_SCRIPT_NAME KIWI_SCRIPT_ENTRY
echo "Kiwi 工作区已加载：${KIWI_WORKSPACE_ROOT}（ROS_DOMAIN_ID=${ROS_DOMAIN_ID}，ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}）"
