#!/usr/bin/env bash
# 为固定版本 RF2O 应用本项目的启动/退出修复；重复执行不会重复修改。
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
dependency_dir="$workspace_dir/src/rf2o_laser_odometry"
patch_file="$workspace_dir/tools/patches/rf2o-startup-shutdown.patch"
expected_revision=313bb4c4123bcc0cc2e042f278312b19a3c46f31
if [[ ! -d "$dependency_dir/.git" ]]; then
  echo '缺少 RF2O：请先运行 vcs import src < tools/platform_dependencies.repos' >&2
  exit 1
fi
if [[ "$(git -C "$dependency_dir" rev-parse HEAD)" != "$expected_revision" ]]; then
  echo 'RF2O 版本与固定依赖不符，停止应用补丁，请先核对源码版本。' >&2
  exit 1
fi
if git -C "$dependency_dir" apply --reverse --check "$patch_file" 2>/dev/null; then
  echo 'RF2O 修复已应用。'
elif git -C "$dependency_dir" apply --check "$patch_file"; then
  git -C "$dependency_dir" apply "$patch_file"
  echo 'RF2O 修复已应用，请重新构建 rf2o_laser_odometry。'
else
  echo 'RF2O 存在冲突修改，补丁未应用；请检查依赖目录中的差异。' >&2
  exit 1
fi
