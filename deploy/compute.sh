#!/usr/bin/env bash
# Shared COMPUTE (cpu/gpu) resolution for setup.sh, build.sh, and run-loop.sh.
# Source this file, then call resolve_compute. Resolution order:
#   1. --cpu / --gpu CLI flag (each script translates it to COMPUTE=cpu|gpu)
#   2. COMPUTE environment variable (auto, cpu, or gpu)
#   3. auto: gpu when nvidia-smi works, otherwise cpu
# After the call, COMPUTE is exactly "cpu" or "gpu".
resolve_compute() {
  local mode="${COMPUTE:-auto}"
  case "$mode" in
    auto)
      if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
        mode="gpu"
      else
        mode="cpu"
      fi
      ;;
    cpu|gpu) ;;
    *)
      echo "Unknown COMPUTE mode: '$COMPUTE' (use auto, cpu, or gpu)" >&2
      return 2
      ;;
  esac
  COMPUTE="$mode"
}
