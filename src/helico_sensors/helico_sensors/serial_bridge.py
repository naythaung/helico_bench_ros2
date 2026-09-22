import serial

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class HelicoSensorBridge(Node):

    def __init__(self):

        super().__init__(
            "helico_sensor_bridge"
        )

        self.serial_port = serial.Serial(
            "/dev/ttyUSB0",
            115200,
            timeout=0.05,
        )

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

        self.pressure_publisher = self.create_publisher(
            Float64,
            "/helico/sensors/pressure",
            10,
        )

        self.timer = self.create_timer(
            0.01,
            self.read_serial,
        )

        self.get_logger().info(
            "Helico sensor bridge connected to /dev/ttyUSB0"
        )

    def read_serial(self):

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

            values = {}

            for item in line.split(","):

                key, value = item.split("=")

                values[
                    key.strip()
                ] = float(
                    value.strip()
                )

            laser = Float64()
            force = Float64()
            pressure = Float64()

            laser.data = values["laser"]
            force.data = values["force"]
            pressure.data = values["pressure"]

            self.laser_publisher.publish(
                laser
            )

            self.force_publisher.publish(
                force
            )

            self.pressure_publisher.publish(
                pressure
            )

        except Exception as error:

            self.get_logger().warning(
                f"Could not parse serial data: {error}"
            )

    def destroy_node(self):

        if self.serial_port.is_open:

            self.serial_port.close()

        super().destroy_node()


def main():

    rclpy.init()

    node = HelicoSensorBridge()

    try:

        rclpy.spin(
            node
        )

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()