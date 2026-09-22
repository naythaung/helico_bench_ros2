import os
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

        self.laser_publisher = (
            self.create_publisher(
                Float64,
                "/helico/sensors/laser",
                10,
            )
        )

        self.force_publisher = (
            self.create_publisher(
                Float64,
                "/helico/sensors/force",
                10,
            )
        )

        self.timer = self.create_timer(
            0.01,
            self.update,
        )

        self.get_logger().info(
            f"Bench sensor bridge started. "
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

            self.last_valid_data_time = (
                time.monotonic()
            )

            self.get_logger().info(
                f"Bench sensor controller connected: "
                f"{self.port_name}"
            )

        except (
            serial.SerialException,
            OSError,
        ) as error:

            self.serial_port = None

            self.get_logger().warning(
                f"Could not connect to bench "
                f"sensor controller on "
                f"{self.port_name}: "
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
                f"Bench sensor controller "
                f"disconnected: {reason}. "
                f"Waiting for reconnection..."
            )

        else:

            self.get_logger().warning(
                "Bench sensor controller "
                "disconnected. "
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
                f"No valid bench sensor data "
                f"for {age:.1f}s. "
                f"Restarting serial connection."
            )

            self.disconnect_serial(
                "sensor data timeout"
            )

    def parse_line(
        self,
        line,
    ):

        valid_data = False

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

                value = float(
                    value.strip()
                )

                message = Float64()
                message.data = value

                if key == "laser":

                    self.laser_publisher.publish(
                        message
                    )

                    valid_data = True

                elif key == "force":

                    self.force_publisher.publish(
                        message
                    )

                    valid_data = True

        except ValueError:
            return False

        return valid_data

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