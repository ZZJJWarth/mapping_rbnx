#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# mapping_rbnx build phase — runs `rbnx codegen`, then builds for the
# selected deployment target.
#
# Target is chosen by the per-target package manifest's `build:` line
# (see package_manifest*.yaml). Add a target by adding a case branch
# below plus its Dockerfile / native step — nothing else changes.
#   x86-docker     x86_64 + docker, ROS2 in image (docker/Dockerfile)   [default]
#   jetson-docker  arm64 Jetson + docker, L4T base (docker/Dockerfile.jetson)
#   jetson-native  arm64 Jetson + host ROS2 — no docker; builds the vendored
#                  cpp_pubsub + rtabmap + rtabmap_ros overlay under
#                  rbnx-build/native_ws so ZC patches are used at runtime.
#
# RBNX_BUILD_CLEAN=1     nuke rbnx-build/ and rebuild without docker cache.
# RBNX_BUILD_VARIANT=fastlio2_full  (x86-docker only) heavy FASTLIO2 image.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
# shellcheck disable=SC1091
source "$PKG/scripts/docker_base_image.sh"
cd "$PKG"

BUILD="rbnx-build"
CLEAN="${RBNX_BUILD_CLEAN:-}"
VARIANT="${RBNX_BUILD_VARIANT:-light}"
IMG="${ROBONIX_MAPPING_IMAGE:-robonix-mapping}"
if [[ -n "${RBNX_BUILD_TARGET:-}" ]]; then
    TARGET="$RBNX_BUILD_TARGET"
elif [[ "$(uname -m)" == "aarch64" ]]; then
    TARGET="jetson-native"
else
    TARGET="x86-docker"
fi
ROS_BASE_IMAGE="${ROBONIX_MAPPING_ROS_BASE_IMAGE:-robonix-ros:humble-ros-base}"
UPSTREAM_ROS_BASE_IMAGE="ros:humble-ros-base"
JETSON_ROS_BASE_IMAGE="${ROBONIX_MAPPING_JETSON_ROS_BASE_IMAGE:-dustynv/ros:humble-ros-base-l4t-r36.4.0}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[build] clean: removing $BUILD"
    rm -rf "$BUILD"
fi
mkdir -p "$BUILD/data"

# ── 1. Codegen (proto stubs for atlas + IDL types + MCP types) — every target ─
# --mcp is REQUIRED: atlas_bridge.py imports `map_mcp` (SaveMap/LoadMap/
# PoseEstimate/SwitchMode request/response dataclasses for the MCP tools).
# Without it codegen emits only proto stubs, `map_mcp` is missing, and the
# bridge dies at import with ModuleNotFoundError → the service never registers.
if command -v rbnx >/dev/null 2>&1; then
    FLAGS=(--mcp)
    [[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
    echo "[build] rbnx codegen ${FLAGS[*]}"
    rbnx codegen -p "$PKG" "${FLAGS[@]}"
else
    echo "[build] WARNING: rbnx not in PATH — skipping proto codegen"
    echo "[build]   install robonix-cli + run \`rbnx setup\` once from the robonix source root"
fi

echo "[build] target=$TARGET"

# ── 2. Per-target build ─────────────────────────────────────────────────────
case "$TARGET" in
    x86-docker|jetson-docker)
        if ! command -v docker >/dev/null 2>&1; then
            echo "[build] error: target $TARGET needs docker on PATH" >&2
            exit 1
        fi
        DOCKER_BUILD_FLAGS=(--network=host --pull=false)
        [[ "$CLEAN" == "1" ]] && DOCKER_BUILD_FLAGS+=(--no-cache)
        if [[ "$TARGET" == "jetson-docker" ]]; then
            DF=docker/Dockerfile.jetson
            DOCKER_BUILD_FLAGS+=(--build-arg "JETSON_ROS_BASE_IMAGE=${JETSON_ROS_BASE_IMAGE}")
        else
            robonix_ensure_local_base_image "$ROS_BASE_IMAGE" "$UPSTREAM_ROS_BASE_IMAGE"
            DOCKER_BUILD_FLAGS+=(--build-arg "ROS_BASE_IMAGE=${ROS_BASE_IMAGE}")
            case "$VARIANT" in
                light)         DF=docker/Dockerfile ;;
                fastlio2_full) DF=docker/Dockerfile.fastlio2_full ;;
                *) echo "[build] unknown RBNX_BUILD_VARIANT: $VARIANT (light|fastlio2_full)" >&2; exit 2 ;;
            esac
        fi
        if [[ "$CLEAN" != "1" ]] && docker image inspect "$IMG" >/dev/null 2>&1; then
            echo "[build] image $IMG present; rebuilding incrementally"
        fi
        if [[ -f .gitmodules ]]; then
            echo "[build] syncing git submodules for docker build context"
            git submodule sync --recursive
            git submodule update --init --recursive
        fi
        echo "[build] docker build -f $DF -t $IMG"
        docker build "${DOCKER_BUILD_FLAGS[@]}" -f "$DF" -t "$IMG" .
        ;;

    jetson-native)
        # No docker: build a host overlay from the vendored sources, then
        # start_native.sh sources it before launching rtabmap. This is required
        # for Robonix ZC because apt's ros-humble-rtabmap-ros does not contain
        # our rtabmap_sync zero-copy subscriber patches.
        echo "[build] native target — building vendored ROS2 overlay"
        missing=0
        if [[ -z "${ROS_DISTRO:-}" || -z "${AMENT_PREFIX_PATH:-}" ]] || ! command -v ros2 >/dev/null 2>&1; then
            if [[ -f /opt/ros/humble/setup.bash ]]; then
                set +u; source /opt/ros/humble/setup.bash; set -u
            fi
        fi
        if ! command -v ros2 >/dev/null 2>&1; then
            echo "[build] ERROR: ros2 not on PATH and /opt/ros/humble/setup.bash was not usable" >&2
            missing=1
        fi
        if ! command -v colcon >/dev/null 2>&1; then
            echo "[build] ERROR: colcon not on PATH. Install python3-colcon-common-extensions." >&2
            missing=1
        fi
        [[ "$missing" == "1" ]] && exit 1

        if [[ -f .gitmodules ]]; then
            echo "[build] syncing git submodules for native build"
            git submodule sync --recursive
            git submodule update --init --recursive
        fi

        NATIVE_WS="$PKG/$BUILD/native_ws"
        mkdir -p "$NATIVE_WS/src"
        ln -sfn "$PKG/third_party/cpp_pubsub" "$NATIVE_WS/src/cpp_pubsub"
        ln -sfn "$PKG/third_party/rtabmap" "$NATIVE_WS/src/rtabmap"
        ln -sfn "$PKG/third_party/rtabmap_ros" "$NATIVE_WS/src/rtabmap_ros"

        echo "[build] colcon build native overlay -> $NATIVE_WS/install"
        (
            cd "$NATIVE_WS"
            colcon build --event-handlers console_direct+ \
                --packages-up-to cpp_pubsub rtabmap_slam rtabmap_odom rtabmap_viz \
                --cmake-args \
                    -DCMAKE_BUILD_TYPE=Release \
                    -DROBONIX_ZC_BUILD_EXAMPLES=OFF \
                    -DBUILD_APP=OFF \
                    -DBUILD_TOOLS=OFF \
                    -DBUILD_EXAMPLES=OFF
        )
        echo "[build] native overlay OK: $NATIVE_WS/install/setup.bash"
        ;;

    *)
        echo "[build] unknown RBNX_BUILD_TARGET: $TARGET" >&2
        echo "[build]   supported: x86-docker | jetson-docker | jetson-native" >&2
        exit 2
        ;;
esac

echo "[build] done (target=$TARGET)."
