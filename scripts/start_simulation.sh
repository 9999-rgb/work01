#!/usr/bin/env bash
# =============================================================================
# XCZS Inspection Robot – Startup Script
# =============================================================================
# Launches the complete ROS 2 + Gazebo simulation with the Qt GUI controller.
#
# Usage:
#   ./scripts/start_simulation.sh         # default: Gazebo + GUI
#   ./scripts/start_simulation.sh build   # colcon build first, then start
# =============================================================================

set -eo pipefail

# -----------------------------------------------------------------------------
# Paths
# -----------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="$WORKSPACE/install/setup.bash"
LAUNCH_PACKAGE="xczs_inspection_robot_control"
LAUNCH_FILE="inspection_robot.launch.py"

# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------
info()  { echo -e "\033[1;32m[INFO]\033[0m  $*"; }
warn()  { echo -e "\033[1;33m[WARN]\033[0m  $*"; }
error() { echo -e "\033[1;31m[ERROR]\033[0m $*"; }

# -----------------------------------------------------------------------------
# Parse arguments
# -----------------------------------------------------------------------------
DO_BUILD=false

for arg in "$@"; do
    case "$arg" in
        build|--build|-b)
            DO_BUILD=true
            ;;
        *)
            error "Unknown argument: $arg"
            echo "Usage: $0 [build]"
            exit 1
            ;;
    esac
done

# -----------------------------------------------------------------------------
# Deactivate conda if it shadows the system Python
# -----------------------------------------------------------------------------
if command -v conda &>/dev/null && [[ -n "${CONDA_PREFIX:-}" ]]; then
    warn "Conda is active – deactivating to avoid Python conflicts …"
    set +e
    conda deactivate
    set -e
fi

# Sanity-checks – make sure we're using the system Python.
PYTHON_PATH="$(command -v python3 || true)"
if [[ "$PYTHON_PATH" != "/usr/bin/python3" ]]; then
    warn "python3 resolves to $PYTHON_PATH (expected /usr/bin/python3)."
    warn "If ROS 2 launch fails, deactivate conda/venv and re-run."
fi

# -----------------------------------------------------------------------------
# Source ROS 2
# -----------------------------------------------------------------------------
if [[ ! -f "$ROS_SETUP" ]]; then
    error "ROS 2 Humble setup not found at $ROS_SETUP"
    error "Is ros-humble-ros-base installed?"
    exit 1
fi
info "Sourcing ROS 2 Humble …"
# shellcheck disable=SC1090
source "$ROS_SETUP"

# -----------------------------------------------------------------------------
# Build (optional)
# -----------------------------------------------------------------------------
if $DO_BUILD; then
    info "Building workspace …"
    cd "$WORKSPACE"
    colcon build --symlink-install
    info "Build complete."
fi

# -----------------------------------------------------------------------------
# Source workspace
# -----------------------------------------------------------------------------
if [[ ! -f "$WORKSPACE_SETUP" ]]; then
    error "Workspace setup not found at $WORKSPACE_SETUP"
    error "Run '$0 build' first to compile the workspace."
    exit 1
fi
info "Sourcing workspace overlay …"
# shellcheck disable=SC1090
source "$WORKSPACE_SETUP"

# -----------------------------------------------------------------------------
# Launch – Gazebo + GUI controller (integrated, no separate teleop)
# -----------------------------------------------------------------------------
info "=============================================="
info "  XCZS Inspection Robot Simulation"
info "  Mode:    Gazebo + GUI Controller"
info "=============================================="
info "Launching $LAUNCH_FILE …"

exec ros2 launch "$LAUNCH_PACKAGE" "$LAUNCH_FILE" \
    gui:=true \
    control_gui:=true \
    teleop:=false
