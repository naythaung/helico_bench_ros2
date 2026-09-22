import time

import serial

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class HelicoSensorBridge(Node):

    def __init__(self):

        super().__init__(
            "helico_sensor_bridge"
        )

        self.declare_parameter(
            "port",
            "/dev/ttyUSB0",
        )

        self.declare_parameter(
            "baud_rate",
            115200,
        )

        self.port_name = (
            self.get_parameter("port")
            .get_parameter_value()
            .string_value
        )

        self.baud_rate = (
            self.get_parameter("baud_rate")
            .get_parameter_value()
            .integer_value
        )
        self.baud_rate = 115200

        self.serial_port = None
        self.last_connect_attempt = 0.0

        self.laser_publisher = self.create_publisher(
            Float64,
            "/helico/sensors/laser",
            10,
        )

        self.force_publisher = self.create_publisher(
            Float64,
            "/helico/sensors/force",
            10,
        )

        self.timer = self.create_timer(
            0.01,
            self.update,
        )

        self.get_logger().info(
            "Sensor ESP32 bridge started."
        )

        self.connect_serial()

    def connect_serial(self):

        now = time.monotonic()

        if (
            now - self.last_connect_attempt
            < 1.0
        ):

            return

        self.last_connect_attempt = now

        try:

            self.serial_port = serial.Serial(
                self.port_name,
                self.baud_rate,
                timeout=0.05,
            )

            self.serial_port.reset_input_buffer()

            self.get_logger().info(
                f"Sensor ESP32 connected: {self.port_name}"
            )

        except (
            serial.SerialException,
            OSError,
        ):

            self.serial_port = None

    def disconnect_serial(self):

        if self.serial_port is not None:

            try:

                if self.serial_port.is_open:
                    self.serial_port.close()

            except Exception:
                pass

        self.serial_port = None

        # Prevent immediate reconnect loops.
        self.last_connect_attempt = time.monotonic()

        self.get_logger().warning(
            "Sensor ESP32 disconnected. "
            "Waiting for reconnection..."
        )

    def update(self):

        if self.serial_port is None:

            self.connect_serial()

            return

        try:

            line = (
                self.serial_port
                .readline()
                .decode(
                    "utf-8",
                    errors="ignore",
                )
                .strip()
            )

            if not line:
                return

            self.parse_line(
                line
            )

        except serial.SerialException as error:

            self.get_logger().warning(
                f"Serial connection lost: {error}"
            )

            self.disconnect_serial()

        except OSError as error:

            self.get_logger().warning(
                f"USB connection lost: {error}"
            )

            self.disconnect_serial()

    def parse_line(
        self,
        line,
    ):

        try:

            items = line.split(",")

            for item in items:

                item = item.strip()

                if "=" not in item:
                    continue

                key, value = item.split(
                    "=",
                    1,
                )

                key = key.strip().lower()

                value = float(
                    value.strip()
                )

                if key in [
                    "laser",
                    "force",
                ]:

                    self.publish_sensor(
                        key,
                        value,
                    )

        except ValueError:

            return

    def publish_sensor(
        self,
        name,
        value,
    ):

        message = Float64()
        message.data = value

        if name == "laser":

            self.laser_publisher.publish(
                message
            )

        elif name == "force":

            self.force_publisher.publish(
                message
            )

    def destroy_node(self):

        if self.serial_port is not None:

            try:

                if self.serial_port.is_open:
                    self.serial_port.close()

            except Exception:

                pass

        super().destroy_node()


def main():

    rclpy.init()

    node = HelicoSensorBridge()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()