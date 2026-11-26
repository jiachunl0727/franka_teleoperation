#!/usr/bin/env bash
set -euo pipefail
task_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
task_sdk_root="$task_repo_root/src/third_party/libfranka_0_9_2"
task_patch_names=(libfranka-0.9.2-hand-stop.patch libfranka-0.9.2-startup-observer.patch libfranka-0.9.2-receive-freshness.patch)
# Later patches can change earlier patches' context. Validate the complete
# reverse/forward chain on copies before touching the dependency checkout.
task_check_root="$(mktemp -d)"
trap 'rm -rf -- "$task_check_root"' EXIT
for task_patch_name in "${task_patch_names[@]}"; do
  while IFS=$'\t' read -r task_added task_removed task_path; do
    mkdir -p "$task_check_root/$(dirname "$task_path")"
    if [[ -f "$task_sdk_root/$task_path" ]]; then
      cp "$task_sdk_root/$task_path" "$task_check_root/$task_path"
    fi
  done < <(git apply --numstat "$task_repo_root/patches/$task_patch_name")
done
declare -a task_applied
for ((task_i=${#task_patch_names[@]}-1; task_i>=0; task_i--)); do
  task_patch="$task_repo_root/patches/${task_patch_names[task_i]}"
  task_applied[task_i]=0
  if git -C "$task_check_root" apply --reverse --check "$task_patch" 2>/dev/null; then
    git -C "$task_check_root" apply --reverse "$task_patch"
    task_applied[task_i]=1
  fi
done
for task_patch_name in "${task_patch_names[@]}"; do
  task_patch="$task_repo_root/patches/$task_patch_name"
  if ! git -C "$task_check_root" apply --check "$task_patch"; then
    echo "Cannot validate $task_patch_name; SDK files were not changed. Inspect local edits." >&2
    exit 1
  fi
  git -C "$task_check_root" apply "$task_patch"
done
for ((task_i=0; task_i<${#task_patch_names[@]}; task_i++)); do
  task_patch_name="${task_patch_names[task_i]}"
  if [[ ${task_applied[task_i]} == 1 ]]; then
    echo "$task_patch_name is already applied."
  else
    task_patch="$task_repo_root/patches/$task_patch_name"
    git -c safe.directory="$task_sdk_root" -C "$task_sdk_root" apply --check "$task_patch"
    git -c safe.directory="$task_sdk_root" -C "$task_sdk_root" apply "$task_patch"
    echo "Applied $task_patch_name. Rebuild libfranka and its consumers."
  fi
done
