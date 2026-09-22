import serial

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class HelicoSensorBridge(Node):

    def __init__(self):

        super().__init__(
            "helico_sensor_bridge"
        )

        # --------------------------------------------------------
        # SERIAL CONNECTION
        # --------------------------------------------------------

        self.serial_port = serial.Serial(
            "/dev/ttyUSB0",
            115200,
            timeout=0.05,
        )

        # --------------------------------------------------------
        # ROS PUBLISHERS
        # --------------------------------------------------------

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

        # --------------------------------------------------------
        # TIMER
        # --------------------------------------------------------

        self.timer = self.create_timer(
            0.01,
            self.read_serial,
        )

        self.get_logger().info(
            "Helico sensor bridge connected to /dev/ttyUSB0"
        )

    # ============================================================
    # PUBLISH ONE SENSOR
    # ============================================================

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

        elif name == "pressure":

            self.pressure_publisher.publish(
                message
            )

    # ============================================================
    # READ SERIAL
    # ============================================================

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

            # Supports both:
            #
            # laser=40.2
            #
            # and:
            #
            # laser=40.2,force=1.4,pressure=25.0

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
                    "pressure",
                ]:

                    self.publish_sensor(
                        key,
                        value,
                    )

        except ValueError as error:

            self.get_logger().warning(
                f"Invalid sensor value: {error}"
            )

        except serial.SerialException as error:

            self.get_logger().error(
                f"Serial connection error: {error}"
            )

    # ============================================================
    # CLEANUP
    # ============================================================

    def destroy_node(self):

        if (
            hasattr(
                self,
                "serial_port",
            )
            and
            self.serial_port.is_open
        ):

            self.serial_port.close()

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