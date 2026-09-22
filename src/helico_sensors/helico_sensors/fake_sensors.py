import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class FakeSensors(Node):

    def __init__(self):
        super().__init__("fake_helico_sensors")

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

        self.time = 0.0

        self.timer = self.create_timer(
            0.1,
            self.publish_sensors,
        )

        self.get_logger().info(
            "Fake Helico sensors streaming."
        )

    def publish_sensors(self):

        self.time += 0.1

        laser = Float64()
        force = Float64()
        pressure = Float64()

        laser.data = (
            40.0
            + 3.0 * math.sin(self.time)
        )

        force.data = (
            1.5
            + 0.4 * math.sin(
                self.time * 0.7
            )
        )

        pressure.data = (
            25.0
            + 5.0 * math.sin(
                self.time * 0.4
            )
        )

        self.laser_publisher.publish(
            laser
        )

        self.force_publisher.publish(
            force
        )

        self.pressure_publisher.publish(
            pressure
        )


def main():

    rclpy.init()

    node = FakeSensors()

    try:

        rclpy.spin(node)

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()