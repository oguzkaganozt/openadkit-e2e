#!/usr/bin/env bash
# One-time Ubuntu GPU host setup: Docker, Compose, python3-venv, NVIDIA runtime.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: ./deploy/setup.sh [--cpu|--gpu]

Installs Docker Engine, Compose, python3-venv, and (on GPU hosts) the NVIDIA
container runtime. Mode can also be set with COMPUTE=cpu|gpu (default: auto,
which uses the GPU when nvidia-smi works). Requires Ubuntu and sudo; GPU mode
additionally requires a working NVIDIA driver (nvidia-smi).
EOF
}

COMPUTE="${COMPUTE:-auto}"

while (($#)); do
  case "$1" in
    --cpu)
      COMPUTE="cpu"
      shift
      ;;
    --gpu)
      COMPUTE="gpu"
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

# shellcheck source=compute.sh
. "$(cd "$(dirname "$0")" && pwd)/compute.sh"
resolve_compute

require_command() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Required command not found: $1" >&2
    exit 1
  }
}

require_command curl
require_command sudo
if [[ "$COMPUTE" == "gpu" ]]; then
  require_command nvidia-smi
  nvidia-smi >/dev/null
fi

# shellcheck source=/dev/null
. /etc/os-release
[[ "${ID:-}" == "ubuntu" ]] || {
  echo "This script supports Ubuntu only (found: ${ID:-unknown})" >&2
  exit 1
}
codename="${VERSION_CODENAME:-}"
[[ -n "$codename" ]] || {
  echo "Could not determine Ubuntu codename" >&2
  exit 1
}
arch="$(dpkg --print-architecture)"

echo "Installing host packages on Ubuntu $codename ($arch)..."
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ca-certificates \
  curl \
  gnupg \
  python3-venv

sudo install -m 0755 -d /etc/apt/keyrings
if [[ ! -f /etc/apt/keyrings/docker.asc ]]; then
  sudo curl --fail --silent --show-error --location \
    https://download.docker.com/linux/ubuntu/gpg \
    --output /etc/apt/keyrings/docker.asc
  sudo chmod a+r /etc/apt/keyrings/docker.asc
fi
printf 'deb [arch=%s signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu %s stable\n' \
  "$arch" "$codename" | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null

if [[ "$COMPUTE" == "gpu" ]]; then
  if [[ ! -f /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg ]]; then
    curl --fail --silent --show-error --location \
      https://nvidia.github.io/libnvidia-container/gpgkey \
      | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  fi
  curl --fail --silent --show-error --location \
    https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null
fi

sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  docker-ce \
  docker-ce-cli \
  containerd.io \
  docker-buildx-plugin \
  docker-compose-plugin
if [[ "$COMPUTE" == "gpu" ]]; then
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y nvidia-container-toolkit
  sudo nvidia-ctk runtime configure --runtime=docker
fi
sudo systemctl enable --now docker
sudo systemctl restart docker
sudo usermod -aG docker "$USER"

sudo docker compose version >/dev/null
if [[ "$COMPUTE" == "gpu" ]]; then
  sudo docker info 2>/dev/null | grep -q nvidia || {
    echo "Docker NVIDIA runtime is not installed" >&2
    exit 1
  }
fi

echo "Host setup complete ($COMPUTE mode)."
if ! docker info >/dev/null 2>&1; then
  echo "Log out and back in so group 'docker' applies, then run ./deploy/build.sh"
  exit 0
fi
echo "Continue with: ./deploy/build.sh --dds-interface <nic>"
