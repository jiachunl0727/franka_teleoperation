#!/usr/bin/env bash
set -euo pipefail
task_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$task_repo_root"
mkdir -p src/third_party
# Exact mounted paths only, confined to this process and its Git children.
export GIT_CONFIG_COUNT=5
export GIT_CONFIG_KEY_0=safe.directory
export GIT_CONFIG_VALUE_0="$task_repo_root/src/third_party/libfranka_0_9_2"
export GIT_CONFIG_KEY_1=safe.directory
export GIT_CONFIG_VALUE_1="$task_repo_root/src/third_party/franka_arm_ros2"
export GIT_CONFIG_KEY_2=safe.directory
export GIT_CONFIG_VALUE_2="$task_repo_root/src/third_party/osqp_0_6_3"
export GIT_CONFIG_KEY_3=safe.directory
export GIT_CONFIG_VALUE_3="$task_repo_root/src/third_party/qpoases_3_2_1"
export GIT_CONFIG_KEY_4=safe.directory
export GIT_CONFIG_VALUE_4="$task_repo_root/src/third_party/piqp_0_6_3"
vcs import src/third_party < third_party.repos
git -C src/third_party/libfranka_0_9_2 submodule update --init --recursive
bash scripts/apply_teleop_dependency_patches.sh

git -C src/third_party/osqp_0_6_3 submodule update --init --recursive
for task_solver in osqp_0_6_3 qpoases_3_2_1 piqp_0_6_3; do
  touch "src/third_party/$task_solver/COLCON_IGNORE"
done
