import serial
import json
import time
import threading
from typing import Any
from flask import Flask, request, jsonify, render_template

app = Flask(__name__)

# Update to your specific Teensy Bluetooth COM port
PORT = "COM7" 
BAUD = 9600
PWM_MIN_US = 1000
PWM_MAX_US = 2000
UI_THROTTLE_MAX = 1000

# Global state
ser = None
latest_drone_status: dict[str, Any] = {
    "status": "idle",
    "last_update": None
}


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def throttle_value_to_us(value: float) -> int:
    clamped_value = clamp(float(value), 0.0, float(UI_THROTTLE_MAX))
    return int(round(PWM_MIN_US + clamped_value))


def build_drone_command(action: str, value: float) -> dict[str, Any]:
    command: dict[str, Any] = {}

    if action == "takeoff":
        command["action"] = "takeoff"
    elif action == "land":
        command["action"] = "land"
    elif action == "cancel":
        command["action"] = "cancel"
    elif action == "program_control":
        command["action"] = "program_control"
    elif action == "user_control":
        command["action"] = "user_control"
    elif action == "forward":
        command["action"] = "user_control"
        command["pitch"] = float(value)
    elif action == "backward":
        command["action"] = "user_control"
        command["pitch"] = -float(value)
    elif action == "stop_pitch":
        command["action"] = "user_control"
        command["pitch"] = 0.0
    elif action == "right":
        command["action"] = "user_control"
        command["roll"] = float(value)
    elif action == "left":
        command["action"] = "user_control"
        command["roll"] = -float(value)
    elif action == "stop_roll":
        command["action"] = "user_control"
        command["roll"] = 0.0
    elif action == "yaw_right":
        command["action"] = "user_control"
        command["yaw"] = float(value)
    elif action == "yaw_left":
        command["action"] = "user_control"
        command["yaw"] = -float(value)
    elif action == "stop_yaw":
        command["action"] = "user_control"
        command["yaw"] = 0.0
    elif action == "throttle":
        command["action"] = "user_control"
        command["throttle_us"] = throttle_value_to_us(float(value))
    else:
        raise ValueError(f"Unsupported action: {action}")

    return {"commands": [command]}


def update_status_from_telemetry(data: dict[str, Any]) -> None:
    global latest_drone_status

    setpoint = data.get("setpoint", {})
    imu = data.get("imu", {})
    motors = data.get("motors", {})
    loop = data.get("loop", {})

    latest_drone_status["status"] = data.get("state", latest_drone_status.get("status", "idle"))
    latest_drone_status["bluetooth_connected"] = data.get("bluetooth_connected", False)
    latest_drone_status["mission_id"] = data.get("mission_id", 0)
    latest_drone_status["last_update"] = time.time()
    latest_drone_status["raw_telemetry"] = data

    if "pitch_deg" in setpoint:
        latest_drone_status["target_pitch"] = setpoint["pitch_deg"]
    if "roll_deg" in setpoint:
        latest_drone_status["target_roll"] = setpoint["roll_deg"]
    if "yaw_deg" in setpoint:
        latest_drone_status["target_yaw"] = setpoint["yaw_deg"]
    if "throttle_us" in setpoint:
        latest_drone_status["throttle_us"] = setpoint["throttle_us"]

    if "pitch_deg" in imu:
        latest_drone_status["pitch"] = imu["pitch_deg"]
    if "roll_deg" in imu:
        latest_drone_status["roll"] = imu["roll_deg"]
    if "yaw_deg" in imu:
        latest_drone_status["yaw"] = imu["yaw_deg"]
    if "gyro_x_dps" in imu:
        latest_drone_status["gyro_x_dps"] = imu["gyro_x_dps"]
    if "gyro_y_dps" in imu:
        latest_drone_status["gyro_y_dps"] = imu["gyro_y_dps"]
    if "gyro_z_dps" in imu:
        latest_drone_status["gyro_z_dps"] = imu["gyro_z_dps"]

    if motors:
        latest_drone_status["motors"] = motors
    if loop:
        latest_drone_status["loop"] = loop

def listen_to_drone():
    """Background thread that constantly reads from the serial port"""
    global ser, latest_drone_status
    while True:
        if ser and ser.is_open:
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        try:
                            # Try to unpack JSON payload from the drone
                            data = json.loads(line)
                            update_status_from_telemetry(data)
                            print(f"[Drone Telemetry]: {data}")
                        except json.JSONDecodeError:
                            # If the drone just sent plain text, save it
                            print(f"[Drone Serial]: {line}")
                            latest_drone_status["last_message"] = line
            except Exception as e:
                print(f"[Listener Error]: {e}")
        
        # Short sleep to prevent CPU pegging
        time.sleep(0.05)

def init_serial():
    global ser
    if ser is None or not ser.is_open:
        try:
            print(f"Connecting to Drone on {PORT}...")
            ser = serial.Serial(PORT, BAUD, timeout=2)
            time.sleep(2)  # Wait for connection to stabilize
            print("Connected!")
            
            # Start background listener thread as daemon so it closes when web server closes
            listener = threading.Thread(target=listen_to_drone, daemon=True)
            listener.start()
            
        except serial.SerialException as e:
            print(f"Serial Error: {e}")
            ser = None

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/status', methods=['GET'])
def get_status():
    """Endpoint for frontend to fetch the latest telemetry data"""
    return jsonify(latest_drone_status)

@app.route('/api/command', methods=['POST'])
def send_command():
    global ser
    init_serial() # Ensure serial is connected
    
    if ser is None:
        return jsonify({"status": "error", "message": "Bluetooth not connected"}), 500

    data = request.json
    action = data.get('action')
    value = data.get('value', 0.0)

    if not action:
        return jsonify({"status": "error", "message": "No action provided"}), 400

    try:
        command_payload = build_drone_command(action, float(value))
        json_payload = json.dumps(command_payload, separators=(',', ':'))
        print(f"[Ground -> Drone]: {json_payload}")
        
        # Send payload + newline over bluetooth
        ser.write((json_payload + '\n').encode('utf-8'))
        
        # Revert 'last_message' so the UI displays success and then updates on next ping
        if "last_message" in latest_drone_status:
            latest_drone_status["last_message"] = "Waiting for response..."
            
        return jsonify({"status": "success", "message": f"Command {action} sent"})
    
    except Exception as e:
        print(f"Error sending command: {e}")
        return jsonify({"status": "error", "message": str(e)}), 500

if __name__ == '__main__':
    # Initialize serial lazily or on startup
    init_serial()
    
    # disable reloader to prevent multiple threads from being launched
    app.run(debug=True, port=5000, use_reloader=False)
