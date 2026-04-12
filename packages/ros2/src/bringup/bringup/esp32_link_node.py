"""
ESP32 Link Node — Orin ↔ ESP32 UART bridge using orin_link_protocol.

Manages two serial connections:
  - Safety ESP32:  TX planner heartbeat (0x02), RX safety heartbeat (0x04)
  - Control ESP32: TX planner commands (0x01), RX control heartbeat (0x03)

Wire format per frame: [0xAA] [type] [len] [payload...]
"""

import struct
import threading
import time

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from msgs.msg import DriveCommand, ESP32Heartbeat

# orin_link_protocol constants
SYNC_BYTE = 0xAA
MSG_PLANNER_COMMAND = 0x01
MSG_PLANNER_HEARTBEAT = 0x02
MSG_CONTROL_HEARTBEAT = 0x03
MSG_SAFETY_HEARTBEAT = 0x04

# Node states
STATE_INIT = 0
STATE_NOT_READY = 1
STATE_READY = 2
STATE_ENABLE = 3
STATE_ACTIVE = 4

# Status flags
STATUS_FLAG_ENABLE_COMPLETE = 0x01
STATUS_FLAG_AUTONOMY_REQUEST = 0x02

# Actuator ranges (must match ESP32 control_config.h)
THROTTLE_MIN = 800   # DAC levels below this produce no wheel movement
THROTTLE_MAX = 4095  # 12-bit DAC full scale
STEERING_MIN = 0
STEERING_MAX = 720   # 360 = centered/straight
BRAKING_MIN = 0
BRAKING_MAX = 3      # 0=released, 3=full brake

def encode_frame(msg_type: int, payload: bytes) -> bytes:
    """Build an orin_link wire frame: [sync][type][len][payload]."""
    return bytes([SYNC_BYTE, msg_type, len(payload)]) + payload

def encode_heartbeat(seq: int, state: int, fault: int, status: int,
                     stop: int, soc: int = 0) -> bytes:
    """Encode an 8-byte heartbeat payload."""
    return struct.pack("BBBBBBBB", seq, state, fault, status, stop, soc, 0, 0)

def encode_planner_command(seq: int, throttle: int, steering: int,
                           braking: int) -> bytes:
    """Encode a 6-byte planner command payload (big-endian u16 fields)."""
    throttle = max(0, min(THROTTLE_MAX, throttle))
    steering = max(STEERING_MIN, min(STEERING_MAX, steering))
    braking = max(BRAKING_MIN, min(BRAKING_MAX, braking))
    return struct.pack(">BHHB", seq, throttle, steering, braking)

def decode_heartbeat(payload: bytes) -> dict | None:
    """Decode an 8-byte heartbeat payload. Returns None if too short."""
    if len(payload) < 6:
        return None
    return {
        "sequence": payload[0],
        "state": payload[1],
        "fault_flags": payload[2],
        "status_flags": payload[3],
        "stop_flags": payload[4],
        "soc_pct": payload[5],
    }

def read_frame(port: serial.Serial, timeout: float = 0.1) -> tuple[int, bytes] | None:
    """
    Read one orin_link frame from the serial port.
    Returns (type, payload) or None on timeout/error.
    """
    # Find sync byte
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        b = port.read(1)
        if not b:
            return None
        if b[0] == SYNC_BYTE:
            break
    else:
        return None

    # Read type + length
    header = port.read(2)
    if len(header) < 2:
        return None

    msg_type = header[0]
    payload_len = header[1]
    if payload_len > 8:
        return None

    # Read payload
    if payload_len > 0:
        payload = port.read(payload_len)
        if len(payload) < payload_len:
            return None
    else:
        payload = b""

    return (msg_type, payload)

class ESP32LinkNode(Node):
    def __init__(self):
        super().__init__("esp32_link")

        # Parameters
        self.declare_parameter("safety_port", "")
        self.declare_parameter("control_port", "")
        self.declare_parameter("baud_rate", 1_000_000)
        self.declare_parameter("heartbeat_hz", 10.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("drive_cmd_topic", "/act/drive_cmd")

        safety_port = self.get_parameter("safety_port").value
        control_port = self.get_parameter("control_port").value
        baud_rate = self.get_parameter("baud_rate").value
        heartbeat_hz = self.get_parameter("heartbeat_hz").value
        cmd_timeout = self.get_parameter("command_timeout_s").value
        drive_cmd_topic = self.get_parameter("drive_cmd_topic").value

        self._cmd_timeout = cmd_timeout
        self._last_cmd_time = 0.0
        self._hb_seq = 0
        self._cmd_seq = 0

        # Latest drive command and Safety target state (protected by lock)
        self._lock = threading.Lock()
        self._latest_cmd: DriveCommand | None = None
        self._safety_target_state: int = STATE_INIT

        # Open serial ports
        self._safety_port = None
        self._control_port = None

        if safety_port:
            try:
                self._safety_port = serial.Serial(
                    port=safety_port, baudrate=baud_rate, timeout=0.05)
                self.get_logger().info(f"Safety port opened: {safety_port}")
            except serial.SerialException as e:
                self.get_logger().error(f"Cannot open safety port: {e}")

        if control_port:
            try:
                self._control_port = serial.Serial(
                    port=control_port, baudrate=baud_rate, timeout=0.05)
                self.get_logger().info(f"Control port opened: {control_port}")
            except serial.SerialException as e:
                self.get_logger().error(f"Cannot open control port: {e}")

        # Publishers
        self._safety_hb_pub = self.create_publisher(ESP32Heartbeat, "/esp32/safety_heartbeat", 10)
        self._control_hb_pub = self.create_publisher(ESP32Heartbeat, "/esp32/control_heartbeat", 10)

        # Subscriber for actuator commands
        self._cmd_sub = self.create_subscription(
            DriveCommand, drive_cmd_topic, self._on_drive_cmd, 10)

        # RX threads
        self._running = True

        if self._safety_port:
            self._safety_rx_thread = threading.Thread(
                target=self._rx_loop, args=(self._safety_port, "safety"),
                daemon=True)
            self._safety_rx_thread.start()

        if self._control_port:
            self._control_rx_thread = threading.Thread(
                target=self._rx_loop, args=(self._control_port, "control"),
                daemon=True)
            self._control_rx_thread.start()

        # TX timer — sends heartbeat to Safety, commands to Control
        period = 1.0 / max(0.1, heartbeat_hz)
        self._tx_timer = self.create_timer(period, self._tx_tick)

        self.get_logger().info("ESP32 link node started")

    def _on_drive_cmd(self, msg: DriveCommand):
        with self._lock:
            self._latest_cmd = msg
            self._last_cmd_time = time.monotonic()

    def _tx_tick(self):
        now = time.monotonic()

        # Determine if we have active drive commands and Safety's current state
        with self._lock:
            cmd = self._latest_cmd
            cmd_age = now - self._last_cmd_time
            safety_state = self._safety_target_state

        cmd_active = cmd is not None and cmd_age < self._cmd_timeout

        # Mirror Safety's target state in planner heartbeat.
        # Safety advances the system: when it says ENABLE, we say ENABLE.
        # When it says ACTIVE, we say ACTIVE. This lets Safety see us
        # as cooperative and complete the ENABLE → ACTIVE transition.
        planner_state = safety_state if safety_state >= STATE_READY else STATE_READY

        status_flags = STATUS_FLAG_AUTONOMY_REQUEST
        if planner_state == STATE_ENABLE:
            status_flags |= STATUS_FLAG_ENABLE_COMPLETE

        hb_payload = encode_heartbeat(
            seq=self._hb_seq,
            state=planner_state,
            fault=0,
            status=status_flags,
            stop=0,
        )
        self._hb_seq = (self._hb_seq + 1) & 0xFF

        # TX planner heartbeat to both ESP32s
        hb_frame = encode_frame(MSG_PLANNER_HEARTBEAT, hb_payload)
        if self._safety_port and self._safety_port.is_open:
            try:
                self._safety_port.write(hb_frame)
            except serial.SerialException as e:
                self.get_logger().error(f"Safety heartbeat TX error: {e}")
        if self._control_port and self._control_port.is_open:
            try:
                self._control_port.write(hb_frame)
            except serial.SerialException as e:
                self.get_logger().error(f"Control heartbeat TX error: {e}")

        # TX to Control: planner command (only when Safety says ACTIVE)
        if self._control_port and self._control_port.is_open and cmd_active and safety_state == STATE_ACTIVE:
            throttle = max(0, min(THROTTLE_MAX, int(cmd.throttle)))
            steering = max(STEERING_MIN, min(STEERING_MAX, int(cmd.steering_position)))
            braking = max(BRAKING_MIN, min(BRAKING_MAX, int(cmd.braking_level)))

            cmd_payload = encode_planner_command(
                seq=self._cmd_seq,
                throttle=throttle,
                steering=steering,
                braking=braking,
            )
            self._cmd_seq = (self._cmd_seq + 1) & 0xFF

            try:
                self._control_port.write(
                    encode_frame(MSG_PLANNER_COMMAND, cmd_payload))
            except serial.SerialException as e:
                self.get_logger().error(f"Control command TX error: {e}")

    def _rx_loop(self, port: serial.Serial, label: str):
        """Background thread that reads heartbeat frames from an ESP32."""
        while self._running:
            try:
                result = read_frame(port, timeout=0.15)
            except serial.SerialException as e:
                self.get_logger().error(f"{label} RX error: {e}")
                time.sleep(0.1)
                continue

            if result is None:
                continue

            msg_type, payload = result
            hb = decode_heartbeat(payload)
            if hb is None:
                continue

            # Publish to appropriate topic
            if label == "safety" and msg_type == MSG_SAFETY_HEARTBEAT:
                self._publish_heartbeat(self._safety_hb_pub, msg_type, hb)
                # Track Safety's target state so we can mirror it in our heartbeat
                with self._lock:
                    self._safety_target_state = hb["state"]
            elif label == "control" and msg_type == MSG_CONTROL_HEARTBEAT:
                self._publish_heartbeat(self._control_hb_pub, msg_type, hb)

    def _publish_heartbeat(self, pub, source: int, hb: dict):
        msg = ESP32Heartbeat()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "esp32"
        msg.source = source
        msg.sequence = hb["sequence"]
        msg.state = hb["state"]
        msg.fault_flags = hb["fault_flags"]
        msg.status_flags = hb["status_flags"]
        msg.stop_flags = hb["stop_flags"]
        msg.soc_pct = hb["soc_pct"]
        pub.publish(msg)

    def destroy_node(self):
        self._running = False
        for port in (self._safety_port, self._control_port):
            if port and port.is_open:
                port.close()
        self.get_logger().info("ESP32 link node stopped")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ESP32LinkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
