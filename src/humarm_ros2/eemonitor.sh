#!/bin/bash

# Function to handle script termination
cleanup() {
    echo "Stopping all processes..."
    
    # Kill all background processes in this session
    kill $(jobs -p) 2>/dev/null
    
    # Try to kill processes by name in case they're still running
    pkill -f "robot_state_publisher" 2>/dev/null
    pkill -f "joint_state_publisher_gui" 2>/dev/null
    pkill -f "rviz2" 2>/dev/null
    pkill -f "eemonitor.py" 2>/dev/null
    pkill -f "joint2can.py" 2>/dev/null
    
    # Exit the script
    exit 0
}

# Set up trap for script termination
trap cleanup SIGINT SIGTERM EXIT

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Source ROS environment
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

# Path to URDF file
URDF_PATH="$SCRIPT_DIR/urdf/humarmv2.urdf.xacro"
XACRO_CMD="$(which xacro) $URDF_PATH"

# Function to open a command in xterm
open_xterm() {
    local title="$1"
    local command="$2"
    local geometry="$3"
    local colors="$4"
    
    # Default geometry if not specified
    if [ -z "$geometry" ]; then
        geometry="80x24"
    fi
    
    # Default colors if not specified
    if [ -z "$colors" ]; then
        colors="-bg black -fg green"
    fi
    
    # Launch xterm with the command
    xterm -title "$title" -geometry $geometry $colors -e "source $WORKSPACE/install/setup.bash && $command; exec bash" &
    
    # Store the PID for cleanup
    XTERM_PIDS="$XTERM_PIDS $!"
}

# Initialize PID storage
XTERM_PIDS=""

# Start robot state publisher
echo "Starting robot state publisher..."
open_xterm "Robot State Publisher" "ros2 run robot_state_publisher robot_state_publisher --ros-args -p \"robot_description:=\$($XACRO_CMD)\"" "80x15" "-bg black -fg cyan"

# Wait a moment for robot state publisher to initialize
sleep 2

# Start joint state publisher GUI
echo "Starting joint state publisher GUI..."
open_xterm "Joint State Publisher" "ros2 run joint_state_publisher_gui joint_state_publisher_gui" "80x30" "-bg black -fg white"

# Start the joint to CAN bridge
echo "Starting joint to CAN bridge..."
open_xterm "Joint2CAN Bridge" "python3 $SCRIPT_DIR/scripts/joint2can.py" "100x30" "-bg black -fg green"

# Start the end-effector monitor
echo "Starting end-effector monitor..."
open_xterm "End-Effector Monitor" "python3 $SCRIPT_DIR/scripts/eemonitor.py" "100x20" "-bg black -fg yellow"

# Start RViz
echo "Starting RViz..."
open_xterm "RViz" "ros2 run rviz2 rviz2" "100x30" "-bg black -fg blue"

echo "All components started. Press Ctrl+C to shut down."
echo "Running xterm processes: $XTERM_PIDS"

# Keep this script running to allow the trap to catch Ctrl+C
while true; do
    sleep 1
done