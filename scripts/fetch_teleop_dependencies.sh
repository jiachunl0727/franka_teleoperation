#!/usr/bin/env bash
set -euo pipefail
task_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$task_repo_root"
mkdir -p src/third_party
# Exact mounted paths only, confined to this process and its Git children.
export GIT_CONFIG_COUNT=2
export GIT_CONFIG_KEY_0=safe.directory
export GIT_CONFIG_VALUE_0="$task_repo_root/src/third_party/libfranka_0_9_2"
export GIT_CONFIG_KEY_1=safe.directory
export GIT_CONFIG_VALUE_1="$task_repo_root/src/third_party/franka_arm_ros2"
vcs import src/third_party < third_party.repos
git -C src/third_party/libfranka_0_9_2 submodule update --init --recursive
bash scripts/apply_teleop_dependency_patches.sh
