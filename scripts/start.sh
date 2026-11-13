#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -eo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
source "${RBNX_ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash"
set -u
cd "$PKG"
export PYTHONPATH="$(rbnx path robonix-api):$PKG:$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"
exec python3 -m fruit_pick.main
