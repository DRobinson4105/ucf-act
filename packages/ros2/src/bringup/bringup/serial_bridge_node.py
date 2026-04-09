import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from serial_bridge.msg import SerialFrame

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__("serial_bridge")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 1_000_000)
        self.declare_parameter("read_size", 64)
        self.declare_parameter("read_timeout", 0.01)
        self.declare_parameter("poll_period", 0.01)

        port = self.get_parameter("port").value
        baud_rate = self.get_parameter("baud_rate").value
        self.read_size = self.get_parameter("read_size").value
        read_timeout = self.get_parameter("read_timeout").value
        poll_period = self.get_parameter("poll_period").value

        self.get_logger().info(f"Opening serial port '{port}' at {baud_rate} baud rate")

        try:
            self._port = serial.Serial(
                port=port,
                baudrate=baud_rate,
                timeout=read_timeout,
            )
        except serial.SerialException as e:
            self.get_logger().fatal(f"Cannot open serial port: {e}")
            raise SystemExit(1) from e

        self._pub = self.create_publisher(SerialFrame, "/from_serial", 10)

        self._sub = self.create_subscription(
            SerialFrame,
            "/to_serial",
            self._tx_callback,
            10,
        )

        self._rx_timer = self.create_timer(poll_period, self._rx_callback)
        self.get_logger().info("serial_bridge node ready")

    def _tx_callback(self, msg: SerialFrame):
        if not msg.data: return

        payload = bytes(msg.data)
        
        try:
            written = self._port.write(payload)
            self.get_logger().debug(f"TX {written}")
        except serial.SerialException as e:
            self.get_logger().error(f"TX error: {e}")

    def _rx_callback(self):
        try:
            waiting = self._port.in_waiting
        except serial.SerialException as exc:
            self.get_logger().error(f"RX in_waiting error: {exc}")
            return

        if waiting == 0:
            return

        to_read = min(waiting, self.read_size)
        try:
            raw = self._port.read(to_read)
        except serial.SerialException as exc:
            self.get_logger().error(f"RX read error: {exc}")
            return

        if not raw:
            return

        msg = SerialFrame()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "serial"
        msg.data = list(raw)
        self._pub.publish(msg)
        self.get_logger().debug(f"RX {msg.data}")

    def destroy_node(self):
        if self._port.is_open:
            self._port.close()
            self.get_logger().info("Serial port closed")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()