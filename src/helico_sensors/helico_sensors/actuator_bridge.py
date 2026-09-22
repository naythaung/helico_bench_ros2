import os
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

        self.declare_parameter(
            "port",
            "/dev/ttyUSB0",
        )

        self.declare_parameter(
            "baud_rate",
            115200,
        )

        self.declare_parameter(
            "reconnect_interval",
            1.0,
        )

        self.declare_parameter(
            "data_timeout",
            3.0,
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

        self.reconnect_interval = (
            self.get_parameter("reconnect_interval")
            .get_parameter_value()
            .double_value
        )

        self.data_timeout = (
            self.get_parameter("data_timeout")
            .get_parameter_value()
            .double_value
        )

        self.serial_port = None

        self.last_connect_attempt = 0.0
        self.last_valid_data_time = None

        self.pressure_publisher = (
            self.create_publisher(
                Float64,
                "/helico/actuator/pressure",
                10,
            )
        )

        self.timer = self.create_timer(
            0.01,
            self.update,
        )

        self.get_logger().info(
            f"Helico controller bridge started. "
            f"Port: {self.port_name}"
        )

        self.connect_serial()

    def port_exists(self):

        return os.path.exists(
            self.port_name
        )

    def connect_serial(self):

        if self.serial_port is not None:
            return

        now = time.monotonic()

        if (
            now - self.last_connect_attempt
            <
            self.reconnect_interval
        ):
            return

        self.last_connect_attempt = now

        if not self.port_exists():
            return

        try:

            serial_port = serial.Serial(
                self.port_name,
                self.baud_rate,
                timeout=0.05,
            )

            serial_port.reset_input_buffer()

            self.serial_port = serial_port

            # Gives the ESP32 a few seconds after opening
            # the connection to begin streaming data.
            self.last_valid_data_time = (
                time.monotonic()
            )

            self.get_logger().info(
                f"Helico controller connected: "
                f"{self.port_name}"
            )

        except (
            serial.SerialException,
            OSError,
        ) as error:

            self.serial_port = None

            self.get_logger().warning(
                f"Could not connect to Helico "
                f"controller on {self.port_name}: "
                f"{error}"
            )

    def disconnect_serial(
        self,
        reason=None,
    ):

        if self.serial_port is not None:

            try:

                if self.serial_port.is_open:
                    self.serial_port.close()

            except Exception:
                pass

        self.serial_port = None
        self.last_valid_data_time = None

        self.last_connect_attempt = (
            time.monotonic()
        )

        if reason:

            self.get_logger().warning(
                f"Helico controller disconnected: "
                f"{reason}. "
                f"Waiting for reconnection..."
            )

        else:

            self.get_logger().warning(
                "Helico controller disconnected. "
                "Waiting for reconnection..."
            )

    def update(self):

        if self.serial_port is None:

            self.connect_serial()
            return

        if not self.port_exists():

            self.disconnect_serial(
                "USB device disappeared"
            )

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

            if line:

                valid_data = self.parse_line(
                    line
                )

                if valid_data:

                    self.last_valid_data_time = (
                        time.monotonic()
                    )

            self.check_data_watchdog()

        except (
            serial.SerialException,
            OSError,
        ) as error:

            self.disconnect_serial(
                str(error)
            )

    def check_data_watchdog(self):

        if self.last_valid_data_time is None:
            return

        age = (
            time.monotonic()
            -
            self.last_valid_data_time
        )

        if age > self.data_timeout:

            self.get_logger().warning(
                f"No valid pressure data for "
                f"{age:.1f}s. "
                f"Restarting serial connection."
            )

            self.disconnect_serial(
                "pressure data timeout"
            )

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

                key = (
                    key
                    .strip()
                    .lower()
                )

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

                return True

        except ValueError:
            pass

        return False

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