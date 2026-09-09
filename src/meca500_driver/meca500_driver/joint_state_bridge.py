import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import mecademicpy.robot as mdr

ROBOT_IP = "192.168.0.100"  # update if yours differs

JOINT_NAMES = [
    "meca_axis_1", "meca_axis_2", "meca_axis_3",
    "meca_axis_4", "meca_axis_5", "meca_axis_6",
]


class MecaJointStateBridge(Node):
    def __init__(self):
        super().__init__('meca_joint_state_bridge')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        self.robot = mdr.Robot()
        self.get_logger().info(f"Connecting to Meca500 at {ROBOT_IP} ...")
        self.robot.Connect(address=ROBOT_IP, monitor_mode=True)
        self.get_logger().info("Connected. Publishing live joint states.")

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

    def timer_callback(self):
        try:
            joints_deg = self.robot.GetRtJointPos(synchronous_update=False)
        except Exception as e:
            self.get_logger().warn(f"Failed to read joint positions: {e}")
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [math.radians(j) for j in joints_deg]
        self.publisher_.publish(msg)

    def destroy_node(self):
        try:
            self.robot.Disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MecaJointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()