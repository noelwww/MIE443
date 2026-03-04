#!/bin/bash
###############################################################################
# launch_all.sh — Start all services for MIE443 Contest 2
#
# Usage:
#   ./launch_all.sh           # Run contest mode
#   ./launch_all.sh --debug   # Run debug mode
#
# Press Ctrl+C to cleanly shut down ALL processes (local + remote).
###############################################################################

set -e

# ─── Configuration ───────────────────────────────────────────────────────────
PI_USER="ubuntu"
PI_IP="100.69.127.190"
PI_PASS="turtlebot4"
PI_SSH="sshpass -p ${PI_PASS} ssh -o StrictHostKeyChecking=no ${PI_USER}@${PI_IP}"
PI_WS="source ~/ros2_ws/install/setup.bash"

LAPTOP_ROS_SETUP="source /opt/ros/jazzy/setup.bash && source /etc/turtlebot4/setup.bash && source /home/turtlebot/ros2_ws/install/setup.bash"

# Pass through any arguments (e.g. --debug)
CONTEST_ARGS="$*"

# Track all background PIDs for cleanup
PIDS=()
SSH_PIDS=()

# Terminal colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

log()  { echo -e "${GREEN}[LAUNCH]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
err()  { echo -e "${RED}[ERROR]${NC} $1"; }
step() { echo -e "${CYAN}────────────────────────────────────────────────────${NC}"; echo -e "${CYAN}  $1${NC}"; echo -e "${CYAN}────────────────────────────────────────────────────${NC}"; }

# ─── Cleanup on exit ─────────────────────────────────────────────────────────
cleanup() {
    echo ""
    log "Shutting down all processes..."

    # Kill local background processes
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -INT "$pid" 2>/dev/null
            sleep 0.5
            kill -9 "$pid" 2>/dev/null
        fi
    done

    # Kill remote processes on the Pi
    log "Stopping remote processes on Pi..."
    $PI_SSH "pkill -f image_capture_server 2>/dev/null; pkill -f so101_bridge 2>/dev/null; pkill -f so101_turtlebot 2>/dev/null" 2>/dev/null || true

    # Kill SSH sessions
    for pid in "${SSH_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null
        fi
    done

    log "All processes stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# ─── Helper: wait for a ROS 2 service to appear ─────────────────────────────
wait_for_service() {
    local service_name="$1"
    local timeout="${2:-30}"
    local elapsed=0
    log "Waiting for service ${service_name} (timeout: ${timeout}s)..."
    while ! ros2 service list 2>/dev/null | grep -q "$service_name"; do
        sleep 1
        elapsed=$((elapsed + 1))
        if [ "$elapsed" -ge "$timeout" ]; then
            warn "Timed out waiting for ${service_name} after ${timeout}s"
            return 1
        fi
    done
    log "Service ${service_name} is ready (${elapsed}s)"
    return 0
}

# ─── Helper: wait for a ROS 2 node to appear ────────────────────────────────
wait_for_node() {
    local node_name="$1"
    local timeout="${2:-30}"
    local elapsed=0
    log "Waiting for node ${node_name} (timeout: ${timeout}s)..."
    while ! ros2 node list 2>/dev/null | grep -q "$node_name"; do
        sleep 1
        elapsed=$((elapsed + 1))
        if [ "$elapsed" -ge "$timeout" ]; then
            warn "Timed out waiting for ${node_name} after ${timeout}s"
            return 1
        fi
    done
    log "Node ${node_name} is ready (${elapsed}s)"
    return 0
}

###############################################################################
# STEP 1: Launch arm bridge + wrist camera on Pi
###############################################################################
step "Step 1/6: Starting arm bridge + wrist camera on Pi"

$PI_SSH "bash -lc '${PI_WS} && ros2 launch lerobot_moveit so101_turtlebot.launch.py'" &
SSH_PIDS+=($!)

# Give the arm bridge time to initialize
sleep 5
wait_for_node "/so101_bridge" 30 || warn "so101_bridge not detected (may be namespace issue — continuing)"

###############################################################################
# STEP 2: Launch image_capture_server on Pi
###############################################################################
step "Step 2/6: Starting OAK-D image capture server on Pi"

$PI_SSH "bash -lc '${PI_WS} && ros2 run mie443_contest2 image_capture_server'" &
SSH_PIDS+=($!)

sleep 3
wait_for_service "oakd_camera/capture_image" 20 || warn "image_capture_server not detected — OAK-D may not work"

###############################################################################
# STEP 3: Launch MoveIt (move_group + rviz) on laptop
###############################################################################
step "Step 3/6: Starting MoveIt move_group + RViz on laptop"

bash -c "${LAPTOP_ROS_SETUP} && ros2 launch lerobot_moveit so101_laptop.launch.py" &
PIDS+=($!)

sleep 5
wait_for_node "/move_group" 30 || warn "move_group not detected — arm planning may not work"

###############################################################################
# STEP 4: Launch YOLO detector on laptop
###############################################################################
step "Step 4/6: Starting YOLO detector on laptop"

bash -c "${LAPTOP_ROS_SETUP} && ros2 run mie443_contest2 yolo_detector.py" &
PIDS+=($!)

sleep 3
wait_for_service "detect_object" 15 || warn "yolo_detector not detected — object detection may not work"

###############################################################################
# STEP 5: Launch AprilTag detector on laptop
###############################################################################
step "Step 5/6: Starting AprilTag detector on laptop"

# Start the OAK-D camera first so apriltag_ros has images to process
ros2 service call /oakd/start_camera std_srvs/srv/Trigger "{}" 2>/dev/null || true
sleep 2

bash -c "${LAPTOP_ROS_SETUP} && ros2 launch mie443_contest2 apriltag_oakd.launch.py" &
PIDS+=($!)

sleep 3
wait_for_node "/apriltag" 10 || warn "apriltag node not detected — tag detection may not work"

###############################################################################
# STEP 6: Print summary + launch contest2
###############################################################################
step "Step 6/6: All services started — launching contest2"

echo ""
log "Service status:"
echo -e "  Arm bridge (Pi):       $(ros2 node list 2>/dev/null | grep -q so101_bridge    && echo -e "${GREEN}✓${NC}" || echo -e "${RED}✗${NC}")"
echo -e "  OAK-D capture (Pi):    $(ros2 service list 2>/dev/null | grep -q oakd_camera/capture_image && echo -e "${GREEN}✓${NC}" || echo -e "${RED}✗${NC}")"
echo -e "  Wrist capture (Pi):    $(ros2 service list 2>/dev/null | grep -q wrist_camera/capture_image && echo -e "${GREEN}✓${NC}" || echo -e "${RED}✗${NC}")"
echo -e "  MoveIt move_group:     $(ros2 node list 2>/dev/null | grep -q move_group      && echo -e "${GREEN}✓${NC}" || echo -e "${RED}✗${NC}")"
echo -e "  YOLO detector:         $(ros2 service list 2>/dev/null | grep -q detect_object && echo -e "${GREEN}✓${NC}" || echo -e "${RED}✗${NC}")"
echo -e "  AprilTag detector:     $(ros2 node list 2>/dev/null | grep -q apriltag         && echo -e "${GREEN}✓${NC}" || echo -e "${RED}✗${NC}")"
echo ""

log "Launching: ros2 run mie443_contest2 contest2 ${CONTEST_ARGS}"
echo -e "${YELLOW}Press Ctrl+C to stop everything.${NC}"
echo ""

# Run contest2 in foreground (blocking) — Ctrl+C triggers cleanup
bash -c "${LAPTOP_ROS_SETUP} && ros2 run mie443_contest2 contest2 ${CONTEST_ARGS}"
