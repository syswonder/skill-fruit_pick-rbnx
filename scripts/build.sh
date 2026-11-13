#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -eo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
FLAGS=(--mcp --ros2)
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)
# MCP types for this skill's contracts, gRPC stubs for the gripper's, and the
# gripper ROS 2 interfaces (GripperState is not a stock ROS 2 message).
rbnx codegen -p "$PKG" "${FLAGS[@]}"
source "${RBNX_ROS_SETUP:-/opt/ros/humble/setup.bash}"
set -u
cd "$PKG/rbnx-build/codegen/ros2_idl"
HOME="${HOME:-/tmp}" colcon build --packages-select gripper
python3 -c "import cv2, numpy" || { echo "fruit_pick needs python3 cv2 and numpy" >&2; exit 1; }
python3 -m compileall -q "$PKG/fruit_pick"
