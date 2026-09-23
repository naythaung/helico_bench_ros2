import time

import serial

import rclpy

from rclpy.node import Node

from std_msgs.msg import Float64

from helico_sensors.serial_discovery import (
    discover_serial_device,
    open_serial,
)


class HelicoSensorBridge(Node):

    def __init__(self):

        super().__init__(
            "helico_sensor_bridge"
        )

        self.declare_parameter(
            "port",
            "auto",
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

        self.configured_port = (
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
            self.get_parameter(
                "reconnect_interval"
            )
            .get_parameter_value()
            .double_value
        )

        self.data_timeout = (
            self.get_parameter(
                "data_timeout"
            )
            .get_parameter_value()
            .double_value
        )

        self.serial_port = None

        self.connected_port = None

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
            "Bench sensor bridge started. "
            f"Port: {self.configured_port}"
        )

        self.connect_serial()

    # ============================================================
    # CONNECTION
    # ============================================================

    def connect_serial(self):

        if self.serial_port is not None:

            return

        now = time.monotonic()

        if (
            now
            -
            self.last_connect_attempt
            <
            self.reconnect_interval
        ):

            return

        self.last_connect_attempt = now

        if (
            self.configured_port
            ==
            "auto"
        ):

            self.get_logger().info(
                "Searching for bench sensor "
                "controller..."
            )

            (
                port_name,
                serial_port,
            ) = discover_serial_device(
                baud_rate=self.baud_rate,
                role="bench",
            )

            if serial_port is None:

                return

            self.connected_port = (
                port_name
            )

            self.serial_port = (
                serial_port
            )

        else:

            try:

                self.serial_port = (
                    open_serial(
                        self.configured_port,
                        self.baud_rate,
                        timeout=0.05,
                    )
                )

                self.connected_port = (
                    self.configured_port
                )

                time.sleep(
                    1.0
                )

                self.serial_port.reset_input_buffer()

            except (
                serial.SerialException,
                OSError,
            ) as error:

                self.serial_port = None
                self.connected_port = None

                self.get_logger().warning(
                    "Could not connect to "
                    "bench sensor controller "
                    f"on {self.configured_port}: "
                    f"{error}"
                )

                return

        self.last_valid_data_time = (
            time.monotonic()
        )

        self.get_logger().info(
            "Bench sensor controller "
            "connected: "
            f"{self.connected_port}"
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

        old_port = (
            self.connected_port
        )

        self.connected_port = None

        self.last_connect_attempt = (
            time.monotonic()
        )

        if reason:

            self.get_logger().warning(
                "Bench sensor controller "
                f"disconnected from "
                f"{old_port}: "
                f"{reason}. "
                "Searching for reconnection..."
            )

        else:

            self.get_logger().warning(
                "Bench sensor controller "
                "disconnected. "
                "Searching for reconnection..."
            )

    # ============================================================
    # UPDATE
    # ============================================================

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

            if line:

                valid_data = (
                    self.parse_line(
                        line
                    )
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

        if (
            self.last_valid_data_time
            is None
        ):

            return

        age = (
            time.monotonic()
            -
            self.last_valid_data_time
        )

        if (
            age
            >
            self.data_timeout
        ):

            self.get_logger().warning(
                "No valid bench sensor "
                f"data for {age:.1f}s. "
                "Restarting discovery."
            )

            self.disconnect_serial(
                "sensor data timeout"
            )

    # ============================================================
    # PARSING
    # ============================================================

    def parse_line(
        self,
        line,
    ):

        values = {}

        try:

            for item in line.split(","):

                item = item.strip()

                if "=" not in item:

                    continue

                key, value = (
                    item.split(
                        "=",
                        1,
                    )
                )

                key = (
                    key
                    .strip()
                    .lower()
                )

                if key not in [
                    "laser",
                    "force",
                ]:

                    continue

                values[key] = float(
                    value.strip()
                )

        except ValueError:

            return False

        if (
            "laser" not in values
            or
            "force" not in values
        ):

            return False

        laser_message = Float64()

        laser_message.data = (
            values["laser"]
        )

        self.laser_publisher.publish(
            laser_message
        )

        force_message = Float64()

        force_message.data = (
            values["force"]
        )

        self.force_publisher.publish(
            force_message
        )

        return True

    # ============================================================
    # SHUTDOWN
    # ============================================================

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