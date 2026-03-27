#!/bin/bash
# =============================================================================
# MIE443 Contest 2 — Laptop-Only Launcher
# =============================================================================
#
# Usage:
#   ./launch_all.sh                  # Normal contest mode with arm
#   ./launch_all.sh --debug          # Debug mode
#   ./launch_all.sh --no-arm         # Skip all arm/camera services
#   ./launch_all.sh --map <path>     # Custom map file
#   ./launch_all.sh --debug --no-arm # Debug mode without arm
#
# If --no-arm is used, Pi services are NOT needed.
# If arm IS used, start these on the Pi BEFORE running this script:
#
#   Terminal 1 (Pi — MoveIt):
#     ssh ubuntu@100.69.127.175
#     source contest2/bin/activate
#     ros2 launch lerobot_moveit so101_turtlebot.launch.py
#
#   Terminal 2 (Pi — Image Capture Server):
#     ssh ubuntu@100.69.127.175
#     cd ~/ros2_ws && colcon build --packages-select mie443_contest2
#     source install/setup.bash
#     source contest2/bin/activate
#     ros2 run mie443_contest2 image_capture_server
#
# =============================================================================

set -e

# ---- Configuration ----
DEBUG_FLAG=""
NO_ARM=false

# ---- Resolve paths ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPS_DIR="$SCRIPT_DIR/maps"
MAP_FILE="${MAP_FILE:-$MAPS_DIR/Contest2MapPractice.yaml}"

# ---- Parse Arguments ----
while [[ $# -gt 0 ]]; do
    case $1 in
        --debug)
            DEBUG_FLAG="--debug"
            shift
            ;;
        --no-arm)
            NO_ARM=true
            shift
            ;;
        --map)
            MAP_FILE="$2"
            shift 2
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Usage: ./launch_all.sh [--debug] [--no-arm] [--map <path>]"
            exit 1
            ;;
    esac
done

# ---- Source workspace ----
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
    echo "[OK] Sourced workspace: $WS_DIR/install/setup.bash"
else
    echo "[ERROR] Could not find $WS_DIR/install/setup.bash"
    echo "        Did you run: cd ~/ros2_ws && colcon build --packages-select mie443_contest2 ?"
    exit 1
fi

# ---- Check map file exists ----
if [ ! -f "$MAP_FILE" ]; then
    echo "[ERROR] Map file not found: $MAP_FILE"
    echo "        Place your .yaml and .pgm files in mie443_contest2/maps/"
    exit 1
fi

# ---- If --no-arm not passed, ask the user ----
if [ "$NO_ARM" = false ]; then
    echo ""
    echo "============================================"
    echo "  Do you want to enable the robot arm?"
    echo "============================================"
    echo "  [Y] Yes — full contest with arm pick-up"
    echo "      (requires Pi MoveIt + Image Capture Server running)"
    echo ""
    echo "  [N] No  — navigation + recognition only"
    echo "      (backup mode, no arm, no wrist camera)"
    echo "============================================"
    read -p "  Enable arm? [Y/n]: " arm_choice
    case "$arm_choice" in
        [nN]|[nN][oO])
            NO_ARM=true
            echo "[INFO] Arm DISABLED. Running in navigation-only mode."
            ;;
        *)
            NO_ARM=false
            echo "[INFO] Arm ENABLED."
            ;;
    esac
fi

ARM_MODE="with-arm"
if [ "$NO_ARM" = true ]; then
    ARM_MODE="no-arm"
fi

echo ""
echo "============================================"
echo "  MIE443 Contest 2 — Laptop-Only Launcher"
echo "============================================"
echo "  Map file:      $MAP_FILE"
echo "  Mode:          $([ -z "$DEBUG_FLAG" ] && echo "CONTEST" || echo "DEBUG")"
echo "  Arm:           $([ "$NO_ARM" = true ] && echo "DISABLED (nav+recognize only)" || echo "ENABLED")"
echo "============================================"
echo ""

# ---- Collect PIDs for cleanup ----
PIDS=()

cleanup() {
    echo ""
    echo "[INFO] Shutting down all launched processes..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    if [ "$NO_ARM" = false ]; then
        echo "[INFO] Remember to manually stop Pi services (MoveIt + Image Capture Server)."
    fi
    echo "[INFO] Done."
    exit 0
}
trap cleanup SIGINT SIGTERM

# ---- Pi SSH configuration ----
PI_HOST="ubuntu@100.69.127.175"
PI_WS="/home/ubuntu/ros2_ws"

# ---- Utility: auto-start image_capture_server on Pi via SSH ----
auto_start_pi_image_capture() {
    echo "[INFO] Attempting to auto-start image_capture_server on Pi ($PI_HOST)..."
    # Check SSH connectivity first (2s timeout)
    if ! ssh -o ConnectTimeout=3 -o BatchMode=yes "$PI_HOST" "echo ok" &>/dev/null; then
        echo "[WARN] Cannot SSH into Pi ($PI_HOST). Is it powered on?"
        echo "       Check: ssh $PI_HOST"
        return 1
    fi
    # Kill any existing image_capture_server (stale process)
    ssh -o ConnectTimeout=3 "$PI_HOST" "pkill -f image_capture_server" 2>/dev/null || true
    sleep 1
    # Start image_capture_server in background via SSH
    ssh -o ConnectTimeout=3 "$PI_HOST" "
        source $PI_WS/install/setup.bash && \
        source ~/contest2/bin/activate 2>/dev/null; \
        nohup ros2 run mie443_contest2 image_capture_server > /tmp/image_capture_server.log 2>&1 &
        echo \"[Pi] image_capture_server started (PID: \$!)\"
    "
    echo "[INFO] Waiting for image_capture_server to initialize..."
    sleep 5
    return 0
}

# ---- Utility: ensure image_capture_server is running, auto-start if not ----
ensure_image_capture_server() {
    local max_attempts=3
    local attempt=1
    while [ $attempt -le $max_attempts ]; do
        echo "=== Checking Image Capture Server (attempt $attempt/$max_attempts) ==="
        if check_local_service "/oakd_camera/capture_image" 10; then
            return 0
        fi
        echo "[INFO] Service not found. Trying to auto-start on Pi..."
        auto_start_pi_image_capture
        if check_local_service "/oakd_camera/capture_image" 15; then
            return 0
        fi
        attempt=$((attempt + 1))
    done
    echo "[WARN] Image Capture Server not available after $max_attempts attempts."
    echo "       Manually start on Pi: ros2 run mie443_contest2 image_capture_server"
    echo "       Pi log: ssh $PI_HOST cat /tmp/image_capture_server.log"
    read -p "  Press ENTER to continue anyway, or Ctrl+C to abort..."
    return 1
}

# ---- Utility: wait for a local ROS2 service ----
check_local_service() {
    local service="$1"
    local timeout="$2"
    local interval=2
    local elapsed=0
    echo "[INFO] Waiting up to ${timeout}s for service: $service"
    while [ $elapsed -lt $timeout ]; do
        if ros2 service list 2>/dev/null | grep -q "$service"; then
            echo "[OK] Service $service is available."
            return 0
        fi
        sleep $interval
        elapsed=$((elapsed + interval))
    done
    echo "[WARN] Service $service NOT found after ${timeout}s!"
    return 1
}

# ---- Utility: wait for a local ROS2 action ----
check_local_action() {
    local action="$1"
    local timeout="$2"
    local interval=2
    local elapsed=0
    echo "[INFO] Waiting up to ${timeout}s for action: $action"
    while [ $elapsed -lt $timeout ]; do
        if ros2 action list 2>/dev/null | grep -q "$action"; then
            echo "[OK] Action $action is available."
            return 0
        fi
        sleep $interval
        elapsed=$((elapsed + interval))
    done
    echo "[WARN] Action $action NOT found after ${timeout}s!"
    return 1
}

# ---- Utility: start OAK-D camera with retries ----
start_and_verify_camera() {
    local max_attempts=3
    local attempt=1
    while [ $attempt -le $max_attempts ]; do
        echo "[INFO] Starting OAK-D camera (attempt $attempt/$max_attempts)..."
        ros2 service call /oakd/start_camera std_srvs/srv/Trigger 2>/dev/null || true
        local wait_time=$((attempt * 3))
        echo "[INFO] Waiting ${wait_time}s for camera..."
        sleep $wait_time
        if ros2 topic list 2>/dev/null | grep -q "/oakd/rgb/preview/image_raw"; then
            echo "[OK] OAK-D camera topic is available."
            return 0
        fi
        echo "[WARN] Camera not ready yet. Retrying..."
        attempt=$((attempt + 1))
        sleep 2
    done
    echo "[WARN] Camera may not be streaming. image_capture_server will retry on demand."
    return 1
}

# ---- Utility: open a new terminal with a command ----
launch_terminal() {
    local title="$1"
    local cmd="$2"
    # The inner bash script:
    #  - Sources the workspace
    #  - Runs the command, capturing the exit code
    #  - On ANY exit (normal, error, signal), prints status and waits for Enter
    local inner_script="
        trap '' INT TERM  # ignore signals during cleanup
        source $WS_DIR/install/setup.bash
        echo '[$title] Starting...'
        set +e
        $cmd
        EXIT_CODE=\$?
        echo ''
        echo '============================================'
        if [ \$EXIT_CODE -eq 0 ]; then
            echo '[$title] Process ended normally (exit code 0).'
        elif [ \$EXIT_CODE -gt 128 ]; then
            SIG=\$((EXIT_CODE - 128))
            echo '[$title] CRASHED with signal '\$SIG' (exit code '\$EXIT_CODE')'
            echo '  Signal 6=SIGABRT  Signal 11=SIGSEGV  Signal 15=SIGTERM'
        else
            echo '[$title] Process FAILED with exit code '\$EXIT_CODE
        fi
        echo '============================================'
        echo 'Press Enter to close this window...'
        read
    "
    if command -v gnome-terminal &> /dev/null; then
        gnome-terminal --title="$title" -- bash -c "$inner_script" &
    elif command -v xterm &> /dev/null; then
        xterm -T "$title" -e bash -c "$inner_script" &
    else
        echo "[WARN] No terminal emulator found. Run manually: $cmd"
    fi
    PIDS+=($!)
    sleep 1
}

# =============================================================================
# PHASE 1: Navigation + Localization (ALWAYS runs first)
# =============================================================================
echo "=========================================="
echo "  PHASE 1: Navigation & Localization"
echo "=========================================="

echo "=== Launching AMCL Localization ==="
launch_terminal "AMCL Localization" \
    "ros2 launch turtlebot4_navigation localization.launch.py map:=$MAP_FILE"

echo "=== Launching Nav2 Navigation (custom params) ==="
NAV2_PARAMS="$SCRIPT_DIR/config/nav2.yaml"
if [ ! -f "$NAV2_PARAMS" ]; then
    echo "[WARN] Custom nav2.yaml not found at $NAV2_PARAMS"
    echo "       Falling back to default turtlebot4_navigation params."
    NAV2_PARAMS=""
fi
if [ -n "$NAV2_PARAMS" ]; then
    launch_terminal "Nav2 Navigation" \
        "ros2 launch turtlebot4_navigation nav2.launch.py params_file:=$NAV2_PARAMS"
else
    launch_terminal "Nav2 Navigation" \
        "ros2 launch turtlebot4_navigation nav2.launch.py"
fi

echo "=== Launching RViz ==="
launch_terminal "RViz" \
    "ros2 launch turtlebot4_viz view_navigation.launch.py"

echo ""
echo "============================================"
echo "  LOCALIZE THE ROBOT NOW"
echo "============================================"
echo "  1. In the RViz window, click '2D Pose Estimate'."
echo "  2. Click and drag on the map to set the robot's"
echo "     current position and heading."
echo "  3. Verify the robot shows correctly on the map."
echo ""
read -p "  Press ENTER once the robot is localized in RViz..."
echo ""

# =============================================================================
# PHASE 2: Arm + Perception (ONLY if arm is enabled)
# =============================================================================
if [ "$NO_ARM" = false ]; then
    echo "=========================================="
    echo "  PHASE 2: Arm & Perception Services"
    echo "=========================================="
    echo ""
    echo "  Confirm that these are running on the Pi:"
    echo "    1. ros2 launch lerobot_moveit so101_turtlebot.launch.py  (add port:=/dev/ttyACM1 if needed)"
    echo "    2. ros2 run mie443_contest2 image_capture_server"
    echo ""
    read -p "  Press ENTER to confirm Pi services are running..."
    echo ""

    # Verify Pi services are visible from laptop
    echo "=== Verifying Pi MoveIt ==="
    if ! check_local_action "/move_group" 3; then
        echo "[WARN] MoveIt action not detected. Check Pi terminal."
        read -p "  Press ENTER to continue anyway, or Ctrl+C to abort..."
    fi

    echo "=== Verifying Pi Image Capture Server ==="
    ensure_image_capture_server

    echo "=== Launching MoveIt (Laptop side) ==="
    launch_terminal "MoveIt Laptop" \
        "ros2 launch lerobot_moveit so101_laptop.launch.py"
    sleep 5

    echo "=== Camera Warm-Up (before AprilTag so images are flowing) ==="
    start_and_verify_camera
    echo ""

    echo "=== Launching AprilTag Detector ==="
    launch_terminal "AprilTag Detector" \
        "ros2 launch mie443_contest2 apriltag_oakd.launch.py"
    sleep 3

    echo "=== Launching YOLO Detector ==="
    launch_terminal "YOLO Detector" \
        "ros2 run mie443_contest2 yolo_detector"
    sleep 3


else
    echo "=========================================="
    echo "  PHASE 2: Perception Only (no arm)"
    echo "=========================================="
    echo ""
    echo "  The OAK-D camera is still needed for object detection."
    echo "  Will auto-start image_capture_server on Pi via SSH..."
    echo ""

    echo "=== Verifying/Starting Pi Image Capture Server ==="
    ensure_image_capture_server

    echo "=== Camera Warm-Up (before AprilTag so images are flowing) ==="
    start_and_verify_camera
    echo ""

    echo "=== Launching AprilTag Detector ==="
    launch_terminal "AprilTag Detector" \
        "ros2 launch mie443_contest2 apriltag_oakd.launch.py"
    sleep 3

    echo "=== Launching YOLO Detector ==="
    launch_terminal "YOLO Detector" \
        "ros2 run mie443_contest2 yolo_detector"
    sleep 3

fi

# =============================================================================
# PHASE 3: Main contest node
# =============================================================================
echo "=========================================="
echo "  PHASE 3: Contest 2 Main Node"
echo "=========================================="

CONTEST_ARGS=""
CONTEST_ENV=""
if [ -n "$DEBUG_FLAG" ]; then
    CONTEST_ARGS="$CONTEST_ARGS --debug"
fi
if [ "$NO_ARM" = true ]; then
    CONTEST_ARGS="$CONTEST_ARGS --no-arm"
    CONTEST_ENV="export MIE443_NO_ARM=1; "
fi

if [ -z "$DEBUG_FLAG" ]; then
    echo "[INFO] Starting CONTEST mode."
else
    echo "[INFO] Starting DEBUG mode."
fi
if [ "$NO_ARM" = true ]; then
    echo "[INFO] ARM DISABLED — navigate, recognize, return only."
    echo "[INFO] Environment: MIE443_NO_ARM=1"
fi

launch_terminal "Contest2 Main" \
    "${CONTEST_ENV}echo '[Contest2 Main] Binary: '\$(which contest2); ros2 run mie443_contest2 contest2 $CONTEST_ARGS"

echo ""
echo "============================================"
echo "  All nodes launched!"
echo "============================================"
echo "  Navigation:  AMCL + Nav2 + RViz"
if [ "$NO_ARM" = false ]; then
echo "  Perception:  MoveIt Laptop + AprilTag + YOLO"
echo "  Pi (manual): MoveIt TurtleBot + Image Capture Server"
else
echo "  Perception:  AprilTag + YOLO Detector (no arm)"
echo "  Pi (manual): Image Capture Server only"
fi
echo "  Contest:     contest2 $CONTEST_ARGS"
echo ""
echo "  Press Ctrl+C here to kill all LAPTOP processes."
echo "============================================"

# Wait forever so Ctrl+C triggers the cleanup trap
wait