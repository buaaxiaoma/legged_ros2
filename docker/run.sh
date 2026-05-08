#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE_NAME="${IMAGE_NAME:-legged-ros2:humble}"
CONTAINER_NAME="${CONTAINER_NAME:-legged-ros2-humble}"
ONNX_VERSION="${ONNX_VERSION:-1.22.0}"
ONNX_ARCHIVE="onnxruntime-linux-x64-${ONNX_VERSION}.tgz"
ONNX_DIR="onnxruntime-linux-x64-${ONNX_VERSION}"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is not installed or not in PATH" >&2
  exit 1
fi

for artifact_dir in build install log; do
  if [[ -e "${REPO_ROOT}/${artifact_dir}" ]]; then
    echo "Found ${REPO_ROOT}/${artifact_dir} inside the mounted source tree." >&2
    echo "Remove it first; Docker builds should create build/install/log under /root/legged_ws, not under src/legged_ros2." >&2
    echo "Suggested cleanup on the host: rm -rf ${REPO_ROOT}/build ${REPO_ROOT}/install ${REPO_ROOT}/log" >&2
    exit 1
  fi
done

if docker ps -a --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "Removing existing container: ${CONTAINER_NAME}"
  docker rm -f "${CONTAINER_NAME}" >/dev/null
fi

docker_args=(
  -d
  --name "${CONTAINER_NAME}"
  --network host
  -v "${REPO_ROOT}:/root/legged_ws/src/legged_ros2"
)

if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
  docker_args+=(
    -e "DISPLAY=${DISPLAY}"
    -e "QT_X11_NO_MITSHM=1"
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  )
fi

if [[ -d /dev/dri ]]; then
  docker_args+=(--device /dev/dri)
fi

echo "Starting container ${CONTAINER_NAME} from image ${IMAGE_NAME}"
docker run "${docker_args[@]}" \
  "${IMAGE_NAME}" \
  -lc "trap : TERM INT; sleep infinity & wait"

echo "Initializing workspace inside ${CONTAINER_NAME}"
docker exec "${CONTAINER_NAME}" /bin/bash -lc "
set -eo pipefail

cd /root/legged_ws/src/legged_ros2/third_party
if [[ ! -d \"${ONNX_DIR}\" ]]; then
  if [[ ! -f \"${ONNX_ARCHIVE}\" ]]; then
    wget \"https://github.com/microsoft/onnxruntime/releases/download/v${ONNX_VERSION}/${ONNX_ARCHIVE}\"
  fi
  tar -xzf \"${ONNX_ARCHIVE}\"
fi

source /root/unitree_ros2/setup_local.sh
cd /root/legged_ws

rosdep install --from-paths src --ignore-src -r -y

if [[ ! -f /root/legged_ws/install/setup.bash ]]; then
  colcon build --symlink-install
fi
"

echo "Container is ready and workspace is initialized."
echo "Opening an interactive shell in ${CONTAINER_NAME}"
exec docker exec -it "${CONTAINER_NAME}" /bin/bash
