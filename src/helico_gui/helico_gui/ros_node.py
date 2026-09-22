import time

from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float64

from meca500_interfaces.srv import (
    ListPoses,
    CapturePose,
    DeletePose,
    ListTrajectories,
    InspectTrajectory,
    DeleteTrajectory,
)

from meca500_interfaces.action import (
    PlanTrajectory,
    GoToTrajectoryStart,
    ExecuteTrajectory,
)


class HelicoRosNode(Node):

    def __init__(self):

        super().__init__(
            "helico_qt_gui"
        )

        # ========================================================
        # POSE SERVICES
        # ========================================================

        self.list_poses_client = self.create_client(
            ListPoses,
            "/meca/list_poses",
        )

        self.capture_pose_client = self.create_client(
            CapturePose,
            "/meca/capture_pose",
        )

        self.delete_pose_client = self.create_client(
            DeletePose,
            "/meca/delete_pose",
        )

        # ========================================================
        # TRAJECTORY SERVICES
        # ========================================================

        self.list_trajectories_client = self.create_client(
            ListTrajectories,
            "/meca/list_trajectories",
        )

        self.inspect_trajectory_client = self.create_client(
            InspectTrajectory,
            "/meca/inspect_trajectory",
        )

        self.delete_trajectory_client = self.create_client(
            DeleteTrajectory,
            "/meca/delete_trajectory",
        )

        # ========================================================
        # ACTION CLIENTS
        # ========================================================

        self.plan_client = ActionClient(
            self,
            PlanTrajectory,
            "/meca/plan_trajectory",
        )

        self.go_to_start_client = ActionClient(
            self,
            GoToTrajectoryStart,
            "/meca/go_to_trajectory_start",
        )

        self.execute_client = ActionClient(
            self,
            ExecuteTrajectory,
            "/meca/execute_trajectory",
        )

        # ========================================================
        # BENCH SENSOR STATE
        # ========================================================

        self.latest_laser = None
        self.latest_force = None

        self.last_laser_update = None
        self.last_force_update = None

        # ========================================================
        # HELICO STATE
        # ========================================================

        self.latest_pressure = None
        self.last_pressure_update = None

        # ========================================================
        # BENCH SENSOR SUBSCRIPTIONS
        # ========================================================

        self.laser_subscription = self.create_subscription(
            Float64,
            "/helico/sensors/laser",
            self.laser_callback,
            10,
        )

        self.force_subscription = self.create_subscription(
            Float64,
            "/helico/sensors/force",
            self.force_callback,
            10,
        )

        # ========================================================
        # HELICO SUBSCRIPTIONS
        # ========================================================

        self.pressure_subscription = self.create_subscription(
            Float64,
            "/helico/actuator/pressure",
            self.pressure_callback,
            10,
        )

    # ============================================================
    # BENCH SENSOR CALLBACKS
    # ============================================================

    def laser_callback(
        self,
        msg,
    ):

        self.latest_laser = msg.data
        self.last_laser_update = time.monotonic()

    def force_callback(
        self,
        msg,
    ):

        self.latest_force = msg.data
        self.last_force_update = time.monotonic()

    # ============================================================
    # HELICO CALLBACKS
    # ============================================================

    def pressure_callback(
        self,
        msg,
    ):

        self.latest_pressure = msg.data
        self.last_pressure_update = time.monotonic()