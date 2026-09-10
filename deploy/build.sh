#!/usr/bin/env bash
# Rebuild every host-side artifact needed by the CARLA + VisionPilot + SI loop.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEPLOY="$ROOT/deploy"
VP="$ROOT/upstream/vision_pilot/VisionPilot"
SI="$ROOT/upstream/autoware-safety-island"
# shellcheck source=config.env
source "$DEPLOY/config.env"
COMPOSE=(docker compose --env-file "$DEPLOY/config.env" --file "$DEPLOY/docker-compose.yaml" --profile vp)

CARLA_WHEEL="/tmp/carla-0.9.16-cp310-cp310-manylinux_2_31_x86_64.whl"
CARLA_WHEEL_URL="${CARLA_WHEEL_URL:-https://files.pythonhosted.org/packages/67/be/cea470d588566ce532addc8c414c0ed53fe4d54af75b8eb7b092591279a5/carla-0.9.16-cp310-cp310-manylinux_2_31_x86_64.whl}"
CARLA_WHEEL_SHA256="${CARLA_WHEEL_SHA256:-52b1f2fafb0655e25954f9f6d1e97c211a4da404217fd1d0094b4b7350737c95}"
DDS_INTERFACE="${DDS_INTERFACE:-}"
RUN_AFTER=false

usage() {
  cat <<'EOF'
Usage: ./deploy/build.sh --dds-interface <nic> [--cpu|--gpu] [--run]

Builds the pinned VisionPilot GPU/ROS2 image and FreeRTOS POSIX Safety Island,
downloads the verified CARLA Python wheel, pulls runtime images, and builds the
domain bridge. --run starts the end-to-end loop after the build.
Compute mode can also be set with COMPUTE=cpu|gpu (default: auto, which uses
the GPU when nvidia-smi works). CPU mode builds visionpilot:cpu-ros2 and
expects the loop to run with VISIONPILOT_IMAGE=visionpilot:cpu-ros2
VISIONPILOT_RUNTIME=runc.

On a fresh Ubuntu GPU host, run ./deploy/setup.sh once first.
EOF
}

while (($#)); do
  case "$1" in
    --dds-interface)
      [[ $# -ge 2 ]] || { echo "$1 requires a value" >&2; exit 2; }
      DDS_INTERFACE="$2"
      shift 2
      ;;
    --cpu)
      COMPUTE="cpu"
      shift
      ;;
    --gpu)
      COMPUTE="gpu"
      shift
      ;;
    --run)
      RUN_AFTER=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ -n "$DDS_INTERFACE" ]] || {
  echo "--dds-interface is required (for example: ens3)" >&2
  exit 2
}

# shellcheck source=compute.sh
. "$DEPLOY/compute.sh"
resolve_compute
export COMPUTE
echo "Compute mode: $COMPUTE"
VP_IMAGE="visionpilot:${COMPUTE}-ros2"

require_command() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Required command not found: $1" >&2
    echo "On Ubuntu, run: $DEPLOY/setup.sh" >&2
    exit 1
  }
}

for command in awk curl docker git ip python3 sha256sum; do
  require_command "$command"
done

docker compose version >/dev/null
if [[ "$COMPUTE" == "gpu" ]]; then
  require_command nvidia-smi
  nvidia-smi >/dev/null
fi
required_gib=2
docker image inspect "$CARLA_IMAGE" >/dev/null 2>&1 || ((required_gib += 20))
docker image inspect "$AUTOWARE_IMAGE" >/dev/null 2>&1 || ((required_gib += 15))
docker image inspect "$VP_IMAGE" >/dev/null 2>&1 || ((required_gib += 15))
docker image inspect "$SI_BUILD_IMAGE" >/dev/null 2>&1 || ((required_gib += 15))
free_kib="$(df -Pk "$ROOT" | awk 'NR == 2 {print $4}')"
if ((free_kib < required_gib * 1024 * 1024)); then
  echo "At least $required_gib GiB of free disk space is required; found $((free_kib / 1024 / 1024)) GiB" >&2
  exit 1
fi
ip link show "$DDS_INTERFACE" >/dev/null 2>&1 || {
  echo "DDS interface does not exist: $DDS_INTERFACE" >&2
  exit 1
}
if [[ "$COMPUTE" == "gpu" ]]; then
  docker info --format '{{json .Runtimes}}' | grep -q '"nvidia"' || {
    echo "Docker NVIDIA runtime is not installed" >&2
    echo "On Ubuntu, run: $DEPLOY/setup.sh" >&2
    exit 1
  }
fi

echo "Initializing pinned submodules..."
git -C "$ROOT" submodule update --init \
  upstream/autoware-safety-island \
  upstream/vision_pilot
git -C "$SI" submodule update --init cyclonedds freertos-kernel zephyr

if ! printf '%s  %s\n' "$CARLA_WHEEL_SHA256" "$CARLA_WHEEL" | sha256sum -c - --status 2>/dev/null; then
  wheel_tmp="${CARLA_WHEEL}.download.$$"
  trap 'rm -f "$wheel_tmp"' EXIT
  echo "Downloading CARLA 0.9.16 Python wheel..."
  curl --fail --location --retry 3 --output "$wheel_tmp" "$CARLA_WHEEL_URL"
  printf '%s  %s\n' "$CARLA_WHEEL_SHA256" "$wheel_tmp" | sha256sum -c -
  if [[ -e "$CARLA_WHEEL" && ! -O "$CARLA_WHEEL" ]]; then
    require_command sudo
    sudo install --mode 0644 "$wheel_tmp" "$CARLA_WHEEL"
    rm -f "$wheel_tmp"
  else
    mv "$wheel_tmp" "$CARLA_WHEEL"
  fi
  trap - EXIT
else
  echo "CARLA wheel checksum OK."
fi

echo "Installing host CARLA Python venv..."
if [[ ! -x /tmp/carla-venv/bin/python ]]; then
  python3 -m venv /tmp/carla-venv || {
    echo "python3-venv is required to create /tmp/carla-venv" >&2
    exit 1
  }
fi
/tmp/carla-venv/bin/pip install --disable-pip-version-check "$CARLA_WHEEL"

echo "Building $VP_IMAGE..."
(
  cd "$VP/docker"
  ./build.sh "--$COMPUTE" --ros2
)
docker image inspect "$VP_IMAGE" >/dev/null

echo "Building Safety Island for $DDS_INTERFACE..."
docker pull "$SI_BUILD_IMAGE"
docker run --rm \
  --network host \
  --user "$(id -u):$(id -g)" \
  --env HOME=/tmp \
  --volume "$SI:/workspace" \
  --workdir /workspace \
  --entrypoint /bin/bash \
  "$SI_BUILD_IMAGE" \
  -lc './build.sh \
    --platform freertos-posix \
    -d build/freertos-posix \
    --dds-interface "$1" \
    --control-output DDS_ONLY' \
  bootstrap-si "$DDS_INTERFACE"
test -x "$SI/build/freertos-posix/actuation_freertos"

echo "Pulling runtime images and building the domain bridge..."
"${COMPOSE[@]}" pull carla adapter
"${COMPOSE[@]}" build domain-bridge
"${COMPOSE[@]}" config -q

echo "Build complete ($COMPUTE mode)."
if $RUN_AFTER; then
  exec "$DEPLOY/run-loop.sh"
fi
if [[ "$COMPUTE" == "cpu" ]]; then
  echo "Start the CPU loop with:"
  echo "  VISIONPILOT_IMAGE=visionpilot:cpu-ros2 VISIONPILOT_RUNTIME=runc $DEPLOY/run-loop.sh"
  echo "(run-loop.sh applies these automatically when no GPU is detected)"
else
  echo "Start with: $DEPLOY/run-loop.sh"
fi
