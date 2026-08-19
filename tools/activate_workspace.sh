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
KIWI_INSTALL_SETUP="${KIWI_WORKSPACE_ROOT}/install/setup.bash"

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "错误：未找到 /opt/ros/humble/setup.bash。" >&2
  return 1
fi
if [[ ! -r "${KIWI_INSTALL_SETUP}" ]]; then
  echo "错误：工作区尚未构建，缺少 ${KIWI_INSTALL_SETUP}。" >&2
  return 1
fi

# 每次都先加载 Humble，再加载本工作区。即使终端此前 source 过 nav_ws，
# 当前工作区的包前缀也会重新排到 AMENT_PREFIX_PATH 最前面。
source /opt/ros/humble/setup.bash
source "${KIWI_INSTALL_SETUP}"

for KIWI_PACKAGE_NAME in kiwi_base kiwi_slam kiwi_navigation; do
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

export KIWI_WORKSPACE_ROOT
unset KIWI_INSTALL_SETUP KIWI_PACKAGE_NAME KIWI_PACKAGE_PREFIX
echo "Kiwi 工作区已加载：${KIWI_WORKSPACE_ROOT}"
