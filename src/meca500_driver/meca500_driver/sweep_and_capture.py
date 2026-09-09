import time
import threading
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import mecademicpy.robot as mdr

ROBOT_IP = "192.168.0.100"

JOINT_NAMES = [
    "meca_axis_1", "meca_axis_2", "meca_axis_3",
    "meca_axis_4", "meca_axis_5", "meca_axis_6",
]

# --- Define your sweep here: (j1, j2, j3, j4, j5, j6) in degrees ---
# Vary joints 1/2/3 - these actually TRANSLATE the tip through space,
# giving real parallax. Keep changes small and conservative for a first run.
WAYPOINTS = [
    (0,   -10, 10, 0, 0, 0),
    (5,   -10, 10, 0, 0, 0),
    (10,  -8,  8,  0, 0, 0),
    (15,  -5,  5,  0, 0, 0),
]
SETTLE_TIME_S = 0.5   # pause after reaching a waypoint before capturing (avoids motion blur)
JOINT_VEL_PERCENT = 15  # start slow for a first run - this is % of max joint speed
# ---------------------------------------------------------------------


class SweepAndCapture(Node):
    def __init__(self):
        super().__init__('sweep_and_capture')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.capture_client = self.create_client(Trigger, 'capture_frame')

        self.robot = mdr.Robot()
        self.get_logger().info(f"Connecting to Meca500 at {ROBOT_IP} ...")
        self.robot.Connect(address=ROBOT_IP)
        self.robot.ActivateRobot()
        self.robot.Home()
        self.robot.WaitHomed()
        self.robot.SetJointVel(JOINT_VEL_PERCENT)
        self.get_logger().info("Robot activated, homed, velocity set.")

        self._stop_publishing = False
        self.publish_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.publish_thread.start()

    def _publish_loop(self):
        while not self._stop_publishing and rclpy.ok():
            try:
                joints_deg = self.robot.GetRtJointPos()
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = JOINT_NAMES
                msg.position = [math.radians(j) for j in joints_deg]
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to read/publish joint state: {e}")
            time.sleep(0.05)  # 20 Hz

    def trigger_capture(self):
        if not self.capture_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("capture_frame service not available - skipping capture.")
            return
        future = self.capture_client.call_async(Trigger.Request())
        while not future.done() and rclpy.ok():
            time.sleep(0.02)
        if future.result() is not None:
            self.get_logger().info(f"Capture: {future.result().message}")
        else:
            self.get_logger().warn("Capture call returned no result.")

    def run_sweep(self):
        for i, (j1, j2, j3, j4, j5, j6) in enumerate(WAYPOINTS):
            self.get_logger().info(f"Waypoint {i+1}/{len(WAYPOINTS)}: {(j1, j2, j3, j4, j5, j6)}")
            self.robot.MoveJoints(j1, j2, j3, j4, j5, j6)
            self.robot.WaitIdle()
            time.sleep(SETTLE_TIME_S)
            self.trigger_capture()
        self.get_logger().info("Sweep complete.")

    def shutdown(self):
        self._stop_publishing = True
        self.publish_thread.join(timeout=2.0)
        try:
            self.robot.DeactivateRobot()
            self.robot.WaitDeactivated()
            self.robot.Disconnect()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SweepAndCapture()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run_sweep()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()