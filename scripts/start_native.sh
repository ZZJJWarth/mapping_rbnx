#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# mapping_rbnx native (no-docker) launcher. Mirrors docker/entrypoint.sh
# but runs directly on the host ROS 2 install. RTAB-Map can come from
# the vendored native overlay or from host apt packages.
# Picked by scripts/start.sh when ROBONIX_MAPPING_FORCE=native (or
# ROBONIX_MAPPING_PLATFORM matches the native whitelist — jetson_orin).
#
# Same sibling-process flow as the container path:
#   1. atlas_bridge   — registers cap, resolves sensors via atlas,
#                       writes /tmp/<algo>_resolved.yaml, declares outputs.
#   2. SLAM engine    — start_engine.sh → ros2 launch rtabmap_2d.launch.py
#                       (MAPPING_LAUNCH_DIR points at the package's launch/).
#
# SIGTERM tears down both children.
set -eo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

# ── ROS 2 host base + optional package overlay ────────────────────────
if [[ -z "${ROS_DISTRO:-}" || -z "${AMENT_PREFIX_PATH:-}" ]] || ! command -v ros2 >/dev/null 2>&1; then
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        set +u; source /opt/ros/humble/setup.bash; set -u
    else
        echo "[mapping-native] ERR: ROS 2 not sourced and /opt/ros/humble missing" >&2
        echo "[mapping-native]      set ROBONIX_MAPPING_FORCE=docker to use the container path" >&2
        exit 2
    fi
fi
RTABMAP_BUILD="${ROBONIX_MAPPING_RTABMAP_BUILD:-}"
if [[ -z "$RTABMAP_BUILD" && -f "$PKG/rbnx-build/rtabmap_build" ]]; then
    RTABMAP_BUILD="$(<"$PKG/rbnx-build/rtabmap_build")"
fi
RTABMAP_BUILD="${RTABMAP_BUILD:-source}"
export ROBONIX_MAPPING_RTABMAP_BUILD="$RTABMAP_BUILD"
NATIVE_OVERLAY="$PKG/rbnx-build/native_ws/install/setup.bash"
if [[ "$RTABMAP_BUILD" == "source" ]]; then
    export ROBONIX_MAPPING_RGB_ZC="${ROBONIX_MAPPING_RGB_ZC:-1}"
    export ROBONIX_MAPPING_DEPTH_ZC="${ROBONIX_MAPPING_DEPTH_ZC:-1}"
    export ROBONIX_MAPPING_SCAN_CLOUD_ZC="${ROBONIX_MAPPING_SCAN_CLOUD_ZC:-1}"
    if [[ -f "$NATIVE_OVERLAY" ]]; then
        set +u; source "$NATIVE_OVERLAY"; set -u
    else
        echo "[mapping-native] ERR: native overlay missing: $NATIVE_OVERLAY" >&2
        echo "[mapping-native]      run \`RBNX_BUILD_TARGET=jetson-native rbnx build\` first" >&2
        exit 2
    fi
elif [[ "$RTABMAP_BUILD" == "apt" ]]; then
    export ROBONIX_MAPPING_RGB_ZC="${ROBONIX_MAPPING_RGB_ZC:-0}"
    export ROBONIX_MAPPING_DEPTH_ZC="${ROBONIX_MAPPING_DEPTH_ZC:-0}"
    export ROBONIX_MAPPING_SCAN_CLOUD_ZC="${ROBONIX_MAPPING_SCAN_CLOUD_ZC:-0}"
else
    echo "[mapping-native] ERR: ROBONIX_MAPPING_RTABMAP_BUILD=$RTABMAP_BUILD not in {source,apt}" >&2
    exit 2
fi

# Fail loud if neither the overlay nor the host apt package provides rtabmap.
if ! ros2 pkg prefix rtabmap_slam >/dev/null 2>&1; then
    echo "[mapping-native] ERR: rtabmap_slam not found for RTABMAP_BUILD=$RTABMAP_BUILD." >&2
    echo "[mapping-native]      use source overlay or install ros-humble-rtabmap-ros" >&2
    exit 2
fi

# ── PYTHONPATH: pkg src + codegen stubs + robonix-api ──────────────────
CODEGEN="$PKG/rbnx-build/codegen"
if [[ ! -d "$CODEGEN/proto_gen" ]]; then
    echo "[mapping-native] ERR: $CODEGEN/proto_gen missing — run \`rbnx codegen -p $PKG\` first" >&2
    exit 2
fi
export PYTHONPATH="$PKG/src:$CODEGEN/proto_gen:$CODEGEN/robonix_mcp_types:${PYTHONPATH:-}"
if command -v rbnx >/dev/null 2>&1; then
    if API="$(rbnx path robonix-api 2>/dev/null)" && [[ -d "$API" ]]; then
        export PYTHONPATH="$API:$PYTHONPATH"
    fi
fi

mkdir -p "$PKG/rbnx-build/data"

# ── Env (mirror the docker -e block) ───────────────────────────────────
export ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}"
export ROBONIX_CAPABILITY_ID="${ROBONIX_CAPABILITY_ID:-mapping}"
export ROBONIX_PKG_HOST_DIR="$PKG"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export MAPPING_RESOLVED_DIR="${MAPPING_RESOLVED_DIR:-/tmp}"
# start_engine.sh reads the launch from here (container used /mapping/launch).
export MAPPING_LAUNCH_DIR="$PKG/launch"
export MAPPING_ENABLE_VIZ="${MAPPING_ENABLE_VIZ:-false}"
# Persistent map store. Container default is /mapping/maps (bind-mounted);
# natively there is no /mapping, so anchor it under the package dir so
# saved maps survive restarts. Override with MAPPING_MAPS_DIR.
export MAPPING_MAPS_DIR="${MAPPING_MAPS_DIR:-$PKG/maps}"

PYBIN="${MAPPING_NATIVE_PYTHON:-python3}"

ATLAS_PID=
ENGINE_PID=
cleanup() {
    [ -n "$ENGINE_PID" ] && kill -TERM "$ENGINE_PID" 2>/dev/null || true
    [ -n "$ATLAS_PID" ]  && kill -TERM "$ATLAS_PID"  2>/dev/null || true
    pkill -TERM -P $$ 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Clear stale gate files from a prior aborted boot (rbnx 2026-05-23 patch).
# Without this, start_native.sh bypasses the bridge-write gate, runs engine
# on a stale resolved.yaml from a previous run, fails fast, and trap kills
# the bridge BEFORE rbnx delivers CMD_INIT — Cancelling all calls error.
rm -f /tmp/mapping_algo /tmp/*_resolved.yaml

# ── 1. atlas_bridge (the cap) ──────────────────────────────────────────
"$PYBIN" -u -m mapping_rbnx.atlas_bridge 2>&1 | sed 's/^/[bridge] /' &
ATLAS_PID=$!

# Gate on atlas_bridge writing /tmp/mapping_algo + /tmp/<algo>_resolved.yaml.
for _ in $(seq 1 60); do
    [ -f /tmp/mapping_algo ] && break
    sleep 0.5
done
ALGO="$(cat /tmp/mapping_algo 2>/dev/null || echo rtabmap)"
export MAPPING_ALGO="$ALGO"
for _ in $(seq 1 60); do
    [ -f "/tmp/${ALGO}_resolved.yaml" ] && break
    sleep 0.5
done

# ── 2. SLAM engine ─────────────────────────────────────────────────────
bash "$PKG/scripts/start_engine.sh" 2>&1 | sed 's/^/[engine] /' &
ENGINE_PID=$!

wait "$ENGINE_PID"
