#!/usr/bin/env bash
source /root/unitree_ros2/setup.sh

# unitree_ros2/setup.sh hard-codes NetworkInterface name.
# Prefer an explicit NET_IF, otherwise auto-detect the default route interface.
if [[ -z "${NET_IF:-}" ]]; then
  if command -v ip >/dev/null 2>&1; then
    NET_IF="$(ip route | awk '/default/ {print $5; exit}')"
  fi
  NET_IF="${NET_IF:-eth0}"
fi

# Re-export CYCLONEDDS_URI so the runtime interface is selected dynamically.
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${NET_IF}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
echo "Set CycloneDDS NetworkInterface to ${NET_IF}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/install/setup.bash"
