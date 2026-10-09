# Shared helpers for scripts/deps/<name>.sh. Sourced, not executed.
# Same contract as CPSL_ROS2_Sensors/scripts/deps/_common.sh and
# CPSL_ROS2_Create3/scripts/deps/_common.sh, kept in step with them on purpose:
# console runs all three repos' scripts the same way.
#
# The contract every deps script keeps:
#   - system dependencies only (apt packages, SDKs built under /opt and
#     installed to /usr/local). Nothing in the workspace, the user's shell rc,
#     the static IP on the robot subnet or chrony -- scripts/bootstrap_host.sh owns
#     those.
#   - run as root, no sudo inside: the Dockerfile and a container run it
#     directly; on a host, run it with `sudo bash`.
#   - idempotent and quick when already satisfied, so a container can re-run
#     it on every start.
#   - a non-zero exit means the dependencies are NOT in place.

set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: $(basename "$0") must run as root (e.g. sudo bash $0)." >&2
    exit 1
fi

_APT_UPDATED=false

# apt_ensure pkg... — installs whichever of the named packages are missing.
# Skips apt-get update entirely when everything is already installed.
apt_ensure() {
    local missing=()
    local pkg
    for pkg in "$@"; do
        if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
            missing+=("$pkg")
        fi
    done
    if [ ${#missing[@]} -eq 0 ]; then
        echo "apt: already installed: $*"
        return 0
    fi
    if [ "$_APT_UPDATED" = false ]; then
        apt-get update
        _APT_UPDATED=true
    fi
    echo "apt: installing ${missing[*]}"
    apt-get install -y --no-install-recommends "${missing[@]}"
}
