#!/bin/bash
# =============================================================================
# MIE443 Contest 2 — Full System Launcher
# =============================================================================
#
# This script launches ALL required nodes for the contest in separate terminals.
#
# PREREQUISITES:
#   1. The TurtleBot 4 is powered on and connected.
#   2. The SO-ARM101 is plugged in (USB + battery).
#   3. The workspace is built: colcon build --packages-select mie443_contest2
#   4. The SLAM map files (.yaml + .pgm) are in mie443_contest2/maps/
#   5. The box coordinates are updated in mie443_contest2/boxes_database/coords.xml
#
# USAGE:
#   chmod +x launch_all.sh
#   ./launch_all.sh                  # Normal contest mode
#   ./launch_all.sh --debug          # Debug mode (interactive hardware testing)
#   ./launch_all.sh --map <path>     # Specify a custom map file
#
# =============================================================================

set -e

# ---- Configuration ----
TURTLEBOT_IP="${TURTLEBOT_IP:-192.168.185.3}"
DEBUG_FLAG=""

# ---- Resolve paths ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# The maps/ folder is right next to this script in the source tree
MAPS_DIR="$SCRIPT_DIR/maps"
MAP_FILE="${MAP_FILE:-$MAPS_DIR/Contest2MapPractice.yaml}"

# ---- Parse Arguments ----
while [[ $# -gt 0 ]]; do
    case $1 in
        --debug)
            DEBUG_FLAG="--debug"
            shift
            ;;
        --map)
            MAP_FILE="$2"
            shift 2
            ;;
        --ip)
            TURTLEBOT_IP="$2"
            shift 2
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Usage: ./launch_all.sh [--debug] [--map <path>] [--ip <turtlebot_ip>]"
            exit 1
            ;;
    esac
done

# ---- Source workspace ----
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
    echo "[OK] Sourced workspace: $WS_DIR/install/setup.bash"
else
    echo "[ERROR] Could not find $WS_DIR/install/setup.bash"
    echo "        Did you run: colcon build --packages-select mie443_contest2 ?"
    exit 1
fi

echo ""
echo "============================================"
echo "  MIE443 Contest 2 — System Launcher"
echo "============================================"
echo "  TurtleBot IP:  $TURTLEBOT_IP"
echo "  Map file:      $MAP_FILE"
echo "  Mode:          $([ -z "$DEBUG_FLAG" ] && echo "CONTEST" || echo "DEBUG")"
echo "============================================"
echo ""

# ---- Check map file exists ----
if [ ! -f "$MAP_FILE" ]; then
    echo "[ERROR] Map file not found: $MAP_FILE"
    echo "        Place your .yaml and .pgm files in mie443_contest2/maps/"
    exit 1
fi

# ---- Function to open a new terminal with a command ----
# Supports gnome-terminal (Ubuntu default) and xterm as fallback
launch_terminal() {
    local title="$1"
    local cmd="$2"

    # Source the workspace in each new terminal
    local full_cmd="source $WS_DIR/install/setup.bash && echo '[$title] Starting...' && $cmd"

    if command -v gnome-terminal &> /dev/null; then
        gnome-terminal --title="$title" -- bash -c "$full_cmd; echo ''; echo '[$title] Process ended. Press Enter to close.'; read" &
    elif command -v xterm &> /dev/null; then
        xterm -T "$title" -e bash -c "$full_cmd; echo ''; echo '[$title] Process ended. Press Enter to close.'; read" &
    else
        echo "[WARN] No terminal emulator found (gnome-terminal or xterm)."
        echo "       Please run manually: $cmd"
    fi

    sleep 1  # Give each terminal a moment to start
}

echo "=== Step 1/7: Launching AMCL Localization ==="
launch_terminal "AMCL Localization" \
    "ros2 launch turtlebot4_navigation localization.launch.py map:=$MAP_FILE"

echo "=== Step 2/7: Launching Nav2 Navigation ==="
launch_terminal "Nav2 Navigation" \
    "ros2 launch turtlebot4_navigation nav2.launch.py"

echo "=== Step 3/7: Launching RViz ==="
launch_terminal "RViz" \
    "ros2 launch turtlebot4_viz view_navigation.launch.py"

echo ""
echo "============================================"
echo "  IMPORTANT: Waiting for you to localize!"
echo "============================================"
echo "  1. In the RViz window that just opened,"
echo "     click the '2D Pose Estimate' button."
echo "  2. Click and drag on the map to indicate"
echo "     the robot's current position and heading."
echo "  3. The robot should now show on the map."
echo ""
read -p "  Press ENTER once the robot is localized in RViz..."
echo ""

echo "=== Step 4/7: Launching MoveIt (Laptop side) ==="
launch_terminal "MoveIt Laptop" \
    "ros2 launch lerobot_moveit so101_laptop.launch.py"

echo "=== Step 5/7: Launching AprilTag Detector ==="
launch_terminal "AprilTag Detector" \
    "ros2 launch mie443_contest2 apriltag_oakd.launch.py"

echo "=== Step 6/7: Launching YOLO Detector ==="
launch_terminal "YOLO Detector" \
    "ros2 run mie443_contest2 yolo_detector"

echo "=== Step 7/7: Launching Contest 2 Node ==="
sleep 3  # Give AprilTag, YOLO, and MoveIt a moment to initialize

if [ -z "$DEBUG_FLAG" ]; then
    echo "[INFO] Starting CONTEST mode. Timer begins when node starts!"
    launch_terminal "Contest2 Main" \
        "ros2 run mie443_contest2 contest2"
else
    echo "[INFO] Starting DEBUG mode. Interactive hardware testing."
    launch_terminal "Contest2 Debug" \
        "ros2 run mie443_contest2 contest2 --debug"
fi

echo ""
echo "============================================"
echo "  All LAPTOP nodes launched!"
echo "============================================"
echo ""
echo "  REMINDER: The following must ALREADY be"
echo "  running on the TurtleBot via SSH."
echo ""
echo "  If not, open two SSH terminals now:"
echo "    ssh ubuntu@$TURTLEBOT_IP"
echo ""
echo "    SSH Terminal 1 — Image Capture Server:"
echo "      ros2 run mie443_contest2 image_capture_server"
echo ""
echo "    SSH Terminal 2 — SO-ARM101 Hardware Bridge:"
echo "      ros2 launch lerobot_moveit so101_turtlebot.launch.py"
echo ""
echo "============================================"
echo "  To stop everything: Ctrl+C in each terminal"
echo "============================================"
