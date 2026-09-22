import time

import serial

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class HelicoActuatorBridge(Node):

    def __init__(self):

        super().__init__(
            "helico_actuator_bridge"
        )

        # Hardware configuration is exposed as ROS parameters.
        # The defaults can later be overridden by hardware.yaml.
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

        self.serial_port = None
        self.last_connect_attempt = 0.0

        self.pressure_publisher = self.create_publisher(
            Float64,
            "/helico/actuator/pressure",
            10,
        )

        self.timer = self.create_timer(
            0.01,
            self.update,
        )

        self.get_logger().info(
            "Helico actuator bridge started."
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
                f"Helico controller connected: "
                f"{self.port_name}"
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
        self.last_connect_attempt = time.monotonic()

        self.get_logger().warning(
            "Helico controller disconnected. "
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

        except (
            serial.SerialException,
            OSError,
        ) as error:

            self.get_logger().warning(
                f"Serial connection lost: {error}"
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

                if key != "pressure":
                    continue

                pressure = float(
                    value.strip()
                )

                message = Float64()
                message.data = pressure

                self.pressure_publisher.publish(
                    message
                )

        except ValueError:
            return

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

    node = HelicoActuatorBridge()

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