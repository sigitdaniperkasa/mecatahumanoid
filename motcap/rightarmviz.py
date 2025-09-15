import pybullet as p
import pybullet_data
import serial
import time

# --- 1. PyBullet Setup ---
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

# --- 2. Load Models ---
planeId = p.loadURDF("plane.urdf")
urdf_file = "humarm_right.urdf"
try:
    robotId = p.loadURDF(
        urdf_file,
        basePosition=[0, 0, 0],
        useFixedBase=True,
        flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
    )
except p.error as e:
    print(f"ERROR: Could not load the URDF file: {e}")
    exit()

# --- 3. Get Joint Information ---
num_joints = p.getNumJoints(robotId)
controllable_joints = []
for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    if joint_info[2] == p.JOINT_REVOLUTE:
        controllable_joints.append(i)

print(f"Controlling {len(controllable_joints)} joints.")

# --- 4. Serial Connection Setup (NEW) ---
# IMPORTANT: Change this to your ESP32's serial port
SERIAL_PORT = 'COM8'  # For Windows
# SERIAL_PORT = '/dev/ttyUSB0' # For Linux
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to ESP32 on {SERIAL_PORT}")
except serial.SerialException as e:
    print(f"Error: Could not open serial port {SERIAL_PORT}. {e}")
    p.disconnect()
    exit()

# --- 5. Main Simulation Loop ---
try:
    while True:
        # --- Read from ESP32 (REPLACES TRAJECTORY) ---
        target_positions = None
        if ser.in_waiting > 0:
            try:
                # Read one line of data from the serial port
                line = ser.readline().decode('utf-8').strip()
                
                # Parse the line into a list of floats (these are our radians)
                joint_angles_rad = [float(val) for val in line.split(',')]

                # Check if we received the correct number of angles
                if len(joint_angles_rad) == len(controllable_joints):
                    target_positions = joint_angles_rad
                else:
                    print(f"Warning: Received {len(joint_angles_rad)} angles, expected {len(controllable_joints)}")

            except (ValueError, UnicodeDecodeError):
                # This handles cases where the data is not a valid float or is garbled
                print(f"Warning: Could not parse serial data: '{line}'")

        # --- Apply the Control ---
        # If we successfully received new target positions, apply them
        if target_positions:
            p.setJointMotorControlArray(
                bodyIndex=robotId,
                jointIndices=controllable_joints,
                controlMode=p.POSITION_CONTROL,
                targetPositions=target_positions
            )

        # Step the simulation
        p.stepSimulation()
        time.sleep(1./240.)

except KeyboardInterrupt:
    print("Simulation stopped by user.")
except p.error:
    print("Simulation window closed.")
finally:
    # Always make sure to close the serial port and disconnect
    ser.close()
    if p.isConnected():
        p.disconnect()
    print("Serial port closed and simulation disconnected.")
