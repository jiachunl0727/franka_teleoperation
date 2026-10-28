#!/usr/bin/env bash
set -euo pipefail
task_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
task_sdk_root="$task_repo_root/src/third_party/libfranka_0_9_2"
task_patch="$task_repo_root/patches/libfranka-0.9.2-hand-stop.patch"

if git -c safe.directory="$task_sdk_root" -C "$task_sdk_root" apply --reverse --check "$task_patch" 2>/dev/null; then
  echo "libfranka Hand stop patch is already applied."
elif git -c safe.directory="$task_sdk_root" -C "$task_sdk_root" apply --check "$task_patch"; then
  git -c safe.directory="$task_sdk_root" -C "$task_sdk_root" apply "$task_patch"
  echo "Applied libfranka Hand stop transport patch. Rebuild libfranka and its consumers."
else
  echo "Cannot apply the Hand stop patch; inspect local SDK changes before building." >&2
  exit 1
fi
