import serial
import serial.tools.list_ports
import json
import time
import threading
import os
from typing import Any
from flask import Flask, request, jsonify, render_template

app = Flask(__name__, template_folder="template")

# Update to your specific HC-05 Bluetooth SPP COM port
ENV_PORT = os.environ.get("DRONE_BT_PORT")
PORT = ENV_PORT or "COM7"
BAUD = 9600
PWM_MIN_US = 1000
PWM_MAX_US = 2000
UI_THROTTLE_MAX = 1000
TAKEOFF_THROTTLE_MIN = 250
KEEPALIVE_INTERVAL_S = 3.0
TELEMETRY_PRINT_INTERVAL_S = 1.0

# Global state
ser = None
serial_write_lock = threading.Lock()
listener_thread_started = False
keepalive_thread_started = False
last_telemetry_print_at = 0.0
latest_drone_status: dict[str, Any] = {
    "status": "idle",
    "last_update": None,
    "serial_port": PORT,
    "serial_connected": False,
    "serial_error": None,
    "available_ports": [],
}


def format_number(value: Any, digits: int = 1, default: str = "--") -> str:
    try:
        return f"{float(value):.{digits}f}"
    except (TypeError, ValueError):
        return default


def format_int(value: Any, default: str = "--") -> str:
    try:
        return str(int(round(float(value))))
    except (TypeError, ValueError):
        return default


def format_telemetry_summary(data: dict[str, Any]) -> str:
    setpoint = data.get("setpoint", {})
    imu = data.get("imu", {})
    motors = data.get("motors", {})
    loop = data.get("loop", {})

    state = data.get("state", "--")
    bt = "yes" if data.get("bluetooth_connected") else "no"
    throttle = format_int(setpoint.get("throttle_us"))
    roll = format_number(imu.get("roll_deg"))
    pitch = format_number(imu.get("pitch_deg"))
    yaw = format_number(imu.get("yaw_deg"))
    motor_values = "/".join(format_int(motors.get(name)) for name in ("m1", "m2", "m3", "m4"))
    avg_loop = format_int(loop.get("avg_us"))
    max_loop = format_int(loop.get("max_us"))

    return (
        f"[Telemetry] state={state} bt={bt} thr={throttle}us | "
        f"imu R/P/Y={roll}/{pitch}/{yaw}deg | "
        f"motors={motor_values}us | loop avg/max={avg_loop}/{max_loop}us"
    )


def get_serial_port_catalog() -> list[dict[str, str]]:
    catalog: list[dict[str, str]] = []
    for port in serial.tools.list_ports.comports():
        hwid = port.hwid or ""
        description = port.description or ""
        lower_description = description.lower()
        is_bluetooth = "bthenum" in hwid.lower() or "bluetooth" in lower_description
        if is_bluetooth and "000000000000" in hwid:
            port_kind = "bluetooth-incoming"
        elif is_bluetooth:
            port_kind = "bluetooth-outgoing"
        else:
            port_kind = "serial"

        catalog.append({
            "device": port.device,
            "name": port.name or port.device,
            "description": description,
            "hwid": hwid,
            "kind": port_kind,
        })
    return catalog


def format_port_label(port_info: dict[str, str]) -> str:
    suffix = ""
    if port_info["kind"] == "bluetooth-outgoing":
        suffix = " [preferred BT client]"
    elif port_info["kind"] == "bluetooth-incoming":
        suffix = " [incoming BT server]"
    return f"{port_info['device']} - {port_info['description']}{suffix}"


def choose_serial_port() -> tuple[str, str]:
    if ENV_PORT:
        return ENV_PORT, "Selected from DRONE_BT_PORT."

    catalog = get_serial_port_catalog()
    for port_info in catalog:
        if port_info["kind"] == "bluetooth-outgoing":
            return port_info["device"], "Auto-selected outgoing Bluetooth SPP port."

    return PORT, "Using fallback default port."


def refresh_connection_diagnostics() -> None:
    telemetry_rx_time = latest_drone_status.get("serial_last_rx")
    telemetry_seen = telemetry_rx_time is not None

    latest_drone_status["serial_connected"] = ser is not None and ser.is_open
    latest_drone_status["serial_port"] = PORT
    latest_drone_status["telemetry_seen"] = telemetry_seen
    latest_drone_status["link_verified"] = telemetry_seen or bool(latest_drone_status.get("bluetooth_connected"))

    if telemetry_seen:
        latest_drone_status["serial_last_rx_age_ms"] = int(max(0.0, (time.time() - float(telemetry_rx_time)) * 1000.0))
    else:
        latest_drone_status["serial_last_rx_age_ms"] = None

    if latest_drone_status.get("serial_error"):
        latest_drone_status["connection_note"] = f"Serial error: {latest_drone_status['serial_error']}"
    elif not latest_drone_status["serial_connected"]:
        latest_drone_status["connection_note"] = "Serial port is not open."
    elif not telemetry_seen:
        latest_drone_status["connection_note"] = (
            "COM port is open, but no drone telemetry has been received yet. "
            "Check the outgoing Bluetooth SPP port, HC-05 link LED, and TX/RX/GND wiring."
        )
    elif latest_drone_status.get("bluetooth_connected"):
        latest_drone_status["connection_note"] = "Bidirectional Bluetooth link is active."
    else:
        latest_drone_status["connection_note"] = (
            "Telemetry is arriving from the drone, but the drone has not reported recent command RX."
        )


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def throttle_value_to_us(value: float) -> int:
    clamped_value = clamp(float(value), 0.0, float(UI_THROTTLE_MAX))
    return int(round(PWM_MIN_US + clamped_value))


def list_serial_ports() -> list[str]:
    return [format_port_label(port_info) for port_info in get_serial_port_catalog()]


def build_drone_command(action: str, value: float) -> dict[str, Any]:
    command: dict[str, Any] = {}

    if action == "takeoff":
        command["action"] = "takeoff"
        command["throttle_us"] = throttle_value_to_us(max(float(value), float(TAKEOFF_THROTTLE_MIN)))
    elif action == "land":
        command["action"] = "land"
    elif action == "stop":
        command["action"] = "stop"
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
    latest_drone_status["serial_port"] = PORT
    latest_drone_status["serial_error"] = None

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
    refresh_connection_diagnostics()


def close_serial_link() -> None:
    global ser
    if ser is not None and ser.is_open:
        ser.close()
    ser = None
    latest_drone_status["serial_connected"] = False


def write_serial_payload(payload: bytes, *, description: str, update_command_time: bool = False) -> int:
    if ser is None or not ser.is_open:
        raise serial.SerialException("Serial port is not open")

    with serial_write_lock:
        bytes_written = ser.write(payload)
        ser.flush()

    if update_command_time:
        latest_drone_status["last_command_at"] = time.time()
        latest_drone_status["last_command_bytes"] = bytes_written
    else:
        latest_drone_status["last_keepalive_at"] = time.time()
        latest_drone_status["last_keepalive_bytes"] = bytes_written

    latest_drone_status["serial_error"] = None
    latest_drone_status["last_serial_activity"] = description
    refresh_connection_diagnostics()
    return bytes_written


def listen_to_drone():
    """Background thread that constantly reads from the serial port"""
    global ser, latest_drone_status, last_telemetry_print_at
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
                            latest_drone_status["serial_last_rx"] = time.time()
                            refresh_connection_diagnostics()
                            now = time.time()
                            if (now - last_telemetry_print_at) >= TELEMETRY_PRINT_INTERVAL_S:
                                print(format_telemetry_summary(data))
                                last_telemetry_print_at = now
                        except json.JSONDecodeError:
                            # If the drone just sent plain text, save it
                            print(f"[Drone Serial]: {line}")
                            latest_drone_status["last_message"] = line
                            latest_drone_status["serial_last_rx"] = time.time()
                            refresh_connection_diagnostics()
            except Exception as e:
                print(f"[Listener Error]: {e}")
                latest_drone_status["serial_error"] = str(e)
                close_serial_link()
                refresh_connection_diagnostics()
        
        # Short sleep to prevent CPU pegging
        time.sleep(0.05)


def keepalive_to_drone():
    while True:
        if ser and ser.is_open:
            last_command_at = float(latest_drone_status.get("last_command_at", 0.0) or 0.0)
            last_keepalive_at = float(latest_drone_status.get("last_keepalive_at", 0.0) or 0.0)
            now = time.time()
            most_recent_tx = max(last_command_at, last_keepalive_at)

            if (now - most_recent_tx) >= KEEPALIVE_INTERVAL_S:
                try:
                    # A blank newline refreshes the ESP32 RX timeout without creating a command packet.
                    write_serial_payload(b"\n", description="keepalive", update_command_time=False)
                except Exception as e:
                    print(f"[Keepalive Error]: {e}")
                    latest_drone_status["serial_error"] = str(e)
                    close_serial_link()
                    refresh_connection_diagnostics()

        time.sleep(0.25)


def init_serial():
    global ser, PORT, listener_thread_started, keepalive_thread_started
    available_ports = list_serial_ports()
    PORT, port_reason = choose_serial_port()
    latest_drone_status["available_ports"] = available_ports
    latest_drone_status["serial_port"] = PORT
    latest_drone_status["serial_port_reason"] = port_reason

    if ser is None or not ser.is_open:
        try:
            print(f"Available serial ports: {available_ports}")
            print(f"Connecting to drone on {PORT} at {BAUD} baud... {port_reason}")
            ser = serial.Serial(PORT, BAUD, timeout=2, write_timeout=2)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(2)  # Wait for connection to stabilize
            print("Connected!")
            latest_drone_status["serial_error"] = None
            latest_drone_status["last_keepalive_at"] = 0.0
            refresh_connection_diagnostics()
            
            if not listener_thread_started:
                listener = threading.Thread(target=listen_to_drone, daemon=True)
                listener.start()
                listener_thread_started = True
            if not keepalive_thread_started:
                keepalive = threading.Thread(target=keepalive_to_drone, daemon=True)
                keepalive.start()
                keepalive_thread_started = True
            
        except serial.SerialException as e:
            print(f"Serial Error: {e}")
            print("Hint: on Windows HC-05 usually exposes two COM ports; use the outgoing/client SPP port.")
            close_serial_link()
            latest_drone_status["serial_error"] = str(e)
            refresh_connection_diagnostics()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/status', methods=['GET'])
def get_status():
    """Endpoint for frontend to fetch the latest telemetry data"""
    refresh_connection_diagnostics()
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
        wire_payload = (json_payload + '\n').encode('utf-8')
        latest_drone_status["last_command"] = command_payload
        latest_drone_status["last_command_wire"] = wire_payload.decode('utf-8', errors='ignore')
        print(f"[Ground -> Drone on {PORT}]: {json_payload}")
        print(f"[Ground -> Drone bytes]: {wire_payload!r}")
        
        # Send payload + newline over bluetooth
        bytes_written = write_serial_payload(wire_payload, description=f"command:{action}", update_command_time=True)
        latest_drone_status["serial_connected"] = True
        
        # Revert 'last_message' so the UI displays success and then updates on next ping
        if "last_message" in latest_drone_status:
            latest_drone_status["last_message"] = "Waiting for response..."
            
        return jsonify({"status": "success", "message": f"Command {action} sent"})
    
    except serial.SerialTimeoutException as e:
        latest_drone_status["serial_error"] = str(e)
        latest_drone_status["connection_note"] = (
            f"Write timed out on {PORT}. This usually means Windows opened the COM port, "
            "but the Bluetooth SPP link is not actually established. Try the outgoing/client port."
        )
        close_serial_link()
        refresh_connection_diagnostics()
        print(f"Error sending command: {e}")
        return jsonify({"status": "error", "message": str(e)}), 500

    except Exception as e:
        latest_drone_status["serial_error"] = str(e)
        refresh_connection_diagnostics()
        print(f"Error sending command: {e}")
        return jsonify({"status": "error", "message": str(e)}), 500

if __name__ == '__main__':
    # Initialize serial lazily or on startup
    init_serial()
    
    # disable reloader to prevent multiple threads from being launched
    app.run(debug=True, port=5000, use_reloader=False)
