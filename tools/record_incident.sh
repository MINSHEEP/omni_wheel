#!/usr/bin/env bash
# Kiwi ROS 2 故障现场采集脚本。
# 录制话题（含隐藏话题）、保存节点文件日志和运行时快照，结束后生成 tar.gz。

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

DURATION_SEC=0
OUTPUT_ROOT="$WORKSPACE_ROOT/bags"
LAUNCH_CMD=""
RUN_DIR=""
BAG_DIR=""
ROS_LOG_DIR_LOCAL=""
ARCHIVE=""
BAG_PID=""
LAUNCH_PID=""
MONITOR_PID=""
END_SNAPSHOT_PID=""
STOP_REQUESTED=0
CLEANUP_STARTED=0
RECORDER_PID=$$
MIN_FREE_GIB=10
MIN_FREE_KB=$((MIN_FREE_GIB * 1024 * 1024))

usage() {
  cat <<'EOF'
用法：
  tools/record_incident.sh [选项]

选项：
  --launch-cmd CMD       设置 ROS_LOG_DIR 后启动 CMD，例如：
                         --launch-cmd 'ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=false'
  --duration SEC         SEC 秒后自动结束；默认 0，按 Ctrl+C 结束
  --output DIR           保存根目录；默认当前工作区的 bags 目录
  -h, --help             显示帮助

没有 --launch-cmd 时，脚本只负责录制当前已经运行的 ROS 系统。
服务调用不会自动成为 rosbag 普通话题；请把 ros2 service call 命令通过 tee 保存。
EOF
}

die() {
  printf '错误：%s\n' "$*" >&2
  exit 2
}

while (($# > 0)); do
  case "$1" in
    --launch-cmd)
      (($# >= 2)) || die "--launch-cmd 缺少命令"
      LAUNCH_CMD="$2"
      shift 2
      ;;
    --duration)
      (($# >= 2)) || die "--duration 缺少秒数"
      DURATION_SEC="$2"
      shift 2
      ;;
    --output)
      (($# >= 2)) || die "--output 缺少目录"
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "未知参数：$1"
      ;;
  esac
done

[[ "$DURATION_SEC" =~ ^[0-9]+$ ]] || die "--duration 必须是非负整数"

# 脚本可从任意目录执行，但优先加载本工作区的 ROS 环境。
set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [[ -f "$WORKSPACE_ROOT/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$WORKSPACE_ROOT/install/setup.bash"
fi
set -u
command -v ros2 >/dev/null 2>&1 || die "找不到 ros2，请先安装或 source ROS 2 Humble"
command -v tar >/dev/null 2>&1 || die "找不到 tar"
command -v df >/dev/null 2>&1 || die "找不到 df（coreutils）"
command -v realpath >/dev/null 2>&1 || die "找不到 realpath（coreutils）"
command -v setsid >/dev/null 2>&1 || die "找不到 setsid"
command -v timeout >/dev/null 2>&1 || die "找不到 timeout（coreutils）"
command -v ps >/dev/null 2>&1 || die "找不到 ps（procps）"

# 先规范化输出根目录并检查容量，避免录制开始后才发现磁盘空间不足。
mkdir -p "$OUTPUT_ROOT"
OUTPUT_ROOT="$(realpath -e -- "$OUTPUT_ROOT")"
AVAILABLE_KB="$(df -Pk -- "$OUTPUT_ROOT" | awk 'NR == 2 {print $4}')"
[[ "$AVAILABLE_KB" =~ ^[0-9]+$ ]] || die "无法读取输出目录的磁盘可用容量：$OUTPUT_ROOT"
if ((AVAILABLE_KB < MIN_FREE_KB)); then
  AVAILABLE_GIB=$((AVAILABLE_KB / 1024 / 1024))
  die "磁盘可用空间仅 ${AVAILABLE_GIB} GiB，至少需要 ${MIN_FREE_GIB} GiB：$OUTPUT_ROOT"
fi

STAMP="$(date +%Y%m%d_%H%M%S)_$$"
RUN_DIR="$OUTPUT_ROOT/incident_$STAMP"
BAG_DIR="$RUN_DIR/bag/all_topics"
ROS_LOG_DIR_LOCAL="$RUN_DIR/ros_logs"
ARCHIVE="$OUTPUT_ROOT/incident_$STAMP.tar.gz"
mkdir -p "$RUN_DIR" "$ROS_LOG_DIR_LOCAL" \
  "$RUN_DIR/snapshots/live" "$RUN_DIR/terminal"

# 只记录 ROS 相关的非敏感环境变量，避免把密钥等无关环境内容写入归档。
{
  printf 'start_time=%s\n' "$(date -Ins)"
  printf 'workspace=%s\n' "$WORKSPACE_ROOT"
  printf 'run_dir=%s\n' "$RUN_DIR"
  printf 'launch_cmd=%s\n' "${LAUNCH_CMD:-<external-processes>}"
  printf 'duration_sec=%s\n' "$DURATION_SEC"
  printf 'hostname=%s\n' "$(hostname)"
  uname -a
  for name in ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION RMW_IMPLEMENTATION \
    ROS_DOMAIN_ID ROS_LOCALHOST_ONLY; do
    printf '%s=%s\n' "$name" "${!name-<unset>}"
  done
  git -C "$WORKSPACE_ROOT" rev-parse HEAD 2>/dev/null || true
  git -C "$WORKSPACE_ROOT" status --short 2>/dev/null || true
} > "$RUN_DIR/snapshots/manifest.txt"

export ROS_LOG_DIR="$ROS_LOG_DIR_LOCAL"
export RCUTILS_LOGGING_BUFFERED_STREAM=1

run_to_file() {
  local output="$1"
  shift
  timeout --signal=TERM --kill-after=1s 5s "$@" > "$output" 2>&1 || true
}

collect_snapshot() {
  local label="$1"
  local dir="$RUN_DIR/snapshots/$label"
  mkdir -p "$dir"
  date -Ins > "$dir/time.txt"
  run_to_file "$dir/nodes.txt" ros2 node list --all
  run_to_file "$dir/topics.txt" ros2 topic list --include-hidden-topics -t
  run_to_file "$dir/topics_verbose.txt" ros2 topic list --include-hidden-topics -v
  run_to_file "$dir/services.txt" ros2 service list --include-hidden-services -t
  run_to_file "$dir/actions.txt" ros2 action list -t
  run_to_file "$dir/doctor_report_failed.txt" ros2 doctor --report-failed

  while IFS= read -r node; do
    [[ -n "$node" ]] || continue
    local safe_node
    safe_node="$(printf '%s' "$node" | sed 's#[/ ]#_#g')"
    run_to_file "$dir/node_${safe_node}.txt" ros2 node info --include-hidden "$node"
    run_to_file "$dir/params_${safe_node}.yaml" ros2 param dump "$node"
  done < <(sed -n 's#^\(/[^ ]*\)$#\1#p' "$dir/nodes.txt")
}

monitor_loop() {
  local count=0
  collect_snapshot start
  while kill -0 "$RECORDER_PID" 2>/dev/null; do
    sleep 10
    kill -0 "$RECORDER_PID" 2>/dev/null || return 0
    count=$((count + 1))
    collect_snapshot "live/$(printf '%04d_%s' "$count" "$(date +%Y%m%d_%H%M%S)")"
  done
}

group_is_alive() {
  local pgid state
  while read -r pgid state; do
    [[ "$pgid" == "$1" ]] || continue
    [[ "$state" == Z* ]] || return 0
  done < <(ps -eo pgid=,stat= 2>/dev/null)
  return 1
}

verify_group_leader() {
  local pid="$1"
  local name="$2"
  local pgid
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null)"
  pgid="${pgid//[[:space:]]/}"
  if [[ "$pgid" != "$pid" ]]; then
    printf '错误：%s 未创建独立进程组（PID=%s, PGID=%s）。\n' \
      "$name" "$pid" "${pgid:-<none>}" >&2
    return 1
  fi
}

signal_group() {
  local pid="$1"
  local signal="$2"
  kill -s "$signal" -- -"$pid" 2>/dev/null || kill -s "$signal" "$pid" 2>/dev/null || true
}

wait_for_group() {
  local pid="$1"
  local tenths="$2"
  local i
  for ((i = 0; i < tenths; i++)); do
    group_is_alive "$pid" || return 0
    sleep 0.1 || true
  done
  ! group_is_alive "$pid"
}

begin_stop_group() {
  local pid="$1"
  local name="$2"
  local signal="$3"
  [[ -n "$pid" ]] || return 0
  if ! group_is_alive "$pid"; then
    wait "$pid" 2>/dev/null || true
    return 0
  fi
  printf '正在停止%s（PID %s）...\n' "$name" "$pid"
  signal_group "$pid" "$signal"
}

complete_stop_group() {
  local pid="$1"
  local name="$2"
  local initial_signal="$3"
  local grace_tenths="$4"
  [[ -n "$pid" ]] || return 0

  if wait_for_group "$pid" "$grace_tenths"; then
    wait "$pid" 2>/dev/null || true
    return 0
  fi

  if [[ "$initial_signal" != "TERM" ]]; then
    printf '%s 未及时退出，发送 TERM...\n' "$name"
    signal_group "$pid" TERM
    if wait_for_group "$pid" 30; then
      wait "$pid" 2>/dev/null || true
      return 0
    fi
  fi

  printf '%s 仍未退出，发送 KILL...\n' "$name"
  signal_group "$pid" KILL
  if wait_for_group "$pid" 20; then
    wait "$pid" 2>/dev/null || true
    return 0
  fi

  printf '错误：无法停止%s（进程组 %s），跳过打包以免归档仍在写入的文件。\n' \
    "$name" "$pid" >&2
  return 1
}

start_snapshot_monitor() {
  export RUN_DIR RECORDER_PID
  export -f run_to_file collect_snapshot monitor_loop
  setsid bash -c '
    trap "exit 0" INT TERM
    monitor_loop
  ' > "$RUN_DIR/terminal/snapshot_monitor.log" 2>&1 &
  MONITOR_PID=$!
  verify_group_leader "$MONITOR_PID" "快照监控" || return 1
}

start_end_snapshot() {
  export RUN_DIR
  export -f run_to_file collect_snapshot
  setsid bash -c '
    trap "exit 0" INT TERM
    collect_snapshot end
  ' > "$RUN_DIR/terminal/snapshot_end.log" 2>&1 &
  END_SNAPSHOT_PID=$!
  verify_group_leader "$END_SNAPSHOT_PID" "结束快照" || return 1
}

request_stop() {
  if ((STOP_REQUESTED == 0)); then
    printf '\n收到停止请求，正在结束录制...\n'
  fi
  STOP_REQUESTED=1
  return 0
}

finish() {
  ((CLEANUP_STARTED == 1)) && return 0
  CLEANUP_STARTED=1
  STOP_REQUESTED=1
  local cleanup_failed=0

  # 结束快照与进程关停并行进行，避免 ROS 图查询阻塞 rosbag 退出。
  if [[ -n "$BAG_PID" ]] && group_is_alive "$BAG_PID"; then
    if ! start_end_snapshot; then
      printf '警告：结束快照启动失败，继续停止录制。\n' >&2
      END_SNAPSHOT_PID=""
    fi
  fi
  begin_stop_group "$MONITOR_PID" "快照监控" TERM
  begin_stop_group "$BAG_PID" "rosbag" INT
  begin_stop_group "$LAUNCH_PID" "launch" INT

  complete_stop_group "$MONITOR_PID" "快照监控" TERM 10 || cleanup_failed=1
  complete_stop_group "$BAG_PID" "rosbag" INT 80 || cleanup_failed=1
  complete_stop_group "$LAUNCH_PID" "launch" INT 20 || cleanup_failed=1
  begin_stop_group "$END_SNAPSHOT_PID" "结束快照" TERM
  complete_stop_group "$END_SNAPSHOT_PID" "结束快照" TERM 10 || cleanup_failed=1

  if ((cleanup_failed != 0)); then
    return 1
  fi

  date -Ins > "$RUN_DIR/snapshots/end_time.txt"
  run_to_file "$RUN_DIR/snapshots/bag_info.txt" ros2 bag info "$BAG_DIR"
  find "$RUN_DIR" -type f -printf '%p %k KB\n' \
    > "$RUN_DIR/snapshots/file_manifest.txt" 2>/dev/null || true
  du -sh "$RUN_DIR" > "$RUN_DIR/snapshots/size.txt" 2>/dev/null || true

  printf '正在打包：%s\n' "$ARCHIVE"
  mkdir -p "$OUTPUT_ROOT"
  tar -czf "$ARCHIVE" -C "$OUTPUT_ROOT" "$(basename "$RUN_DIR")"

  # 只有完整归档校验成功后才清理本次源目录；失败时保留现场供人工恢复。
  if [[ ! -s "$ARCHIVE" ]] || ! tar -tzf "$ARCHIVE" >/dev/null; then
    printf '错误：归档完整性校验失败，保留源目录：%s\n' "$RUN_DIR" >&2
    return 1
  fi

  local output_root_real run_dir_real expected_run_dir
  output_root_real="$(realpath -e -- "$OUTPUT_ROOT")"
  run_dir_real="$(realpath -e -- "$RUN_DIR")"
  expected_run_dir="$output_root_real/incident_$STAMP"
  if [[ -L "$RUN_DIR" || "$run_dir_real" != "$expected_run_dir" ]]; then
    printf '错误：源目录安全校验失败，拒绝清理：%s\n' "$RUN_DIR" >&2
    return 1
  fi

  if ! find "$run_dir_real" -xdev -depth -delete; then
    printf '错误：归档已生成，但源目录未能完整清理：%s\n' "$run_dir_real" >&2
    return 1
  fi
  printf '故障现场包已生成：%s\n' "$ARCHIVE"
  printf '已清理本次采集源目录：%s\n' "$run_dir_real"
}

on_exit() {
  local status=$?
  trap - EXIT
  if [[ -n "$BAG_PID" ]] && ((CLEANUP_STARTED == 0)); then
    request_stop
    finish || status=1
  fi
  return "$status"
}

trap request_stop INT TERM
trap on_exit EXIT

printf '输出目录：%s\n' "$RUN_DIR"
printf '开始录制所有（含隐藏）话题...\n'
setsid ros2 bag record \
  --output "$BAG_DIR" \
  --all \
  --include-hidden-topics \
  --include-unpublished-topics \
  --storage sqlite3 \
  --compression-mode file \
  --compression-format zstd \
  --max-bag-duration 600 \
  --max-bag-size 2147483648 \
  > "$RUN_DIR/terminal/rosbag_record.log" 2>&1 &
BAG_PID=$!
verify_group_leader "$BAG_PID" "rosbag" || exit 1

sleep 2 || true
if ((STOP_REQUESTED == 1)); then
  finish
  exit 0
fi
if ! group_is_alive "$BAG_PID"; then
  printf 'rosbag 启动失败，详见：%s\n' "$RUN_DIR/terminal/rosbag_record.log" >&2
  finish
  exit 1
fi

if [[ -n "$LAUNCH_CMD" ]]; then
  printf '启动系统：%s\n' "$LAUNCH_CMD"
  setsid bash -lc "$LAUNCH_CMD" \
    > "$RUN_DIR/terminal/launch.log" 2>&1 &
  LAUNCH_PID=$!
  verify_group_leader "$LAUNCH_PID" "launch" || exit 1
fi

printf '采集进行中，按 Ctrl+C 结束并打包。\n'
start_snapshot_monitor || exit 1

DEADLINE=0
if ((DURATION_SEC > 0)); then
  DEADLINE=$((SECONDS + DURATION_SEC))
fi
while ((STOP_REQUESTED == 0)); do
  if ((DEADLINE > 0 && SECONDS >= DEADLINE)); then
    STOP_REQUESTED=1
    break
  fi
  sleep 1 || true
done

finish
