import sys

import rclpy

from PySide6.QtWidgets import QApplication

from helico_gui.ros_node import HelicoRosNode
from helico_gui.main_window import HelicoWindow


def main():

    rclpy.init()

    node = HelicoRosNode()

    app = QApplication(
        sys.argv
    )

    window = HelicoWindow(
        node
    )

    window.show()

    try:

        exit_code = app.exec()

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()

    sys.exit(
        exit_code
    )


if __name__ == "__main__":

    main()