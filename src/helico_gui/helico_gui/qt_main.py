import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QFormLayout,
    QLabel,
    QPushButton,
    QComboBox,
    QLineEdit,
    QSpinBox,
    QListWidget,
    QProgressBar,
    QGroupBox,
    QTabWidget,
    QMessageBox,
    QInputDialog,
)

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
        super().__init__("helico_qt_gui")

        # ========================================================
        # SERVICES
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
        # ACTIONS
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


class HelicoWindow(QMainWindow):

    def __init__(self, node):
        super().__init__()

        self.node = node

        # --------------------------------------------------------
        # CYCLE STATE
        # --------------------------------------------------------

        self.validated_cycle_names = []
        self.pending_cycle_names = []
        self.pending_cycle_details = {}

        self.cycle_current_index = 0

        # --------------------------------------------------------
        # WINDOW
        # --------------------------------------------------------

        self.setWindowTitle("Helico Bench")
        self.resize(1100, 800)

        self.build_gui()

        # ROS spin inside Qt event loop
        self.ros_timer = QTimer(self)

        self.ros_timer.timeout.connect(
            self.spin_ros
        )

        self.ros_timer.start(20)

        # Retry backend connection
        self.backend_timer = QTimer(self)

        self.backend_timer.timeout.connect(
            self.wait_for_backend
        )

        self.backend_timer.start(500)

    # ============================================================
    # GUI
    # ============================================================

    def build_gui(self):

        central = QWidget()

        self.setCentralWidget(
            central
        )

        outer = QVBoxLayout(
            central
        )

        title = QLabel(
            "HELICO BENCH"
        )

        title.setStyleSheet(
            "font-size: 26px; "
            "font-weight: bold; "
            "margin: 10px;"
        )

        outer.addWidget(
            title
        )

        self.tabs = QTabWidget()

        outer.addWidget(
            self.tabs
        )

        self.configure_tab = QWidget()
        self.operate_tab = QWidget()
        self.diagnostics_tab = QWidget()

        self.tabs.addTab(
            self.operate_tab,
            "Operate",
        )

        self.tabs.addTab(
            self.configure_tab,
            "Configure",
        )

        self.tabs.addTab(
            self.diagnostics_tab,
            "Diagnostics",
        )

        self.build_configure_tab()
        self.build_operate_tab()
        self.build_diagnostics_tab()

        self.status_label = QLabel(
            "Starting..."
        )

        self.status_label.setStyleSheet(
            "padding: 8px; "
            "font-weight: bold;"
        )

        outer.addWidget(
            self.status_label
        )

    # ============================================================
    # CONFIGURE TAB
    # ============================================================

    def build_configure_tab(self):

        layout = QHBoxLayout(
            self.configure_tab
        )

        # --------------------------------------------------------
        # LEFT SIDE
        # --------------------------------------------------------

        left = QVBoxLayout()

        planning_box = QGroupBox(
            "Trajectory Planning"
        )

        planning_form = QFormLayout(
            planning_box
        )

        self.start_pose_box = QComboBox()
        self.target_pose_box = QComboBox()

        self.start_pose_box.currentTextChanged.connect(
            self.update_default_trajectory_name
        )

        self.target_pose_box.currentTextChanged.connect(
            self.update_default_trajectory_name
        )

        self.trajectory_name_entry = QLineEdit(
            "gui_trajectory"
        )

        self.candidate_spinbox = QSpinBox()

        self.candidate_spinbox.setRange(
            1,
            50,
        )

        self.candidate_spinbox.setValue(
            3
        )

        self.speed_spinbox = QSpinBox()

        self.speed_spinbox.setRange(
            1,
            100,
        )

        self.speed_spinbox.setValue(
            20
        )

        self.speed_spinbox.setSuffix(
            " %"
        )

        self.clearance_spinbox = QSpinBox()

        self.clearance_spinbox.setRange(
            0,
            200,
        )

        self.clearance_spinbox.setValue(
            10
        )

        self.clearance_spinbox.setSuffix(
            " mm"
        )

        planning_form.addRow(
            "Start pose",
            self.start_pose_box,
        )

        planning_form.addRow(
            "Target pose",
            self.target_pose_box,
        )

        planning_form.addRow(
            "Trajectory name",
            self.trajectory_name_entry,
        )

        planning_form.addRow(
            "Candidates",
            self.candidate_spinbox,
        )

        planning_form.addRow(
            "Speed scaling",
            self.speed_spinbox,
        )

        planning_form.addRow(
            "Minimum clearance",
            self.clearance_spinbox,
        )

        self.plan_button = QPushButton(
            "PLAN TRAJECTORY"
        )

        self.plan_button.clicked.connect(
            self.plan_trajectory
        )

        planning_form.addRow(
            self.plan_button
        )

        self.progress = QProgressBar()

        self.progress.setRange(
            0,
            100,
        )

        self.progress.setValue(
            0
        )

        planning_form.addRow(
            "Progress",
            self.progress,
        )

        left.addWidget(
            planning_box
        )

        # --------------------------------------------------------
        # POSE LIBRARY
        # --------------------------------------------------------

        pose_box = QGroupBox(
            "Pose Library"
        )

        pose_layout = QVBoxLayout(
            pose_box
        )

        self.pose_list = QListWidget()

        pose_layout.addWidget(
            self.pose_list
        )

        pose_buttons = QHBoxLayout()

        save_pose_button = QPushButton(
            "Save Current Robot Pose"
        )

        save_pose_button.clicked.connect(
            self.capture_pose
        )

        delete_pose_button = QPushButton(
            "Delete Pose"
        )

        delete_pose_button.clicked.connect(
            self.delete_pose
        )

        pose_buttons.addWidget(
            save_pose_button
        )

        pose_buttons.addWidget(
            delete_pose_button
        )

        pose_layout.addLayout(
            pose_buttons
        )

        left.addWidget(
            pose_box
        )

        # --------------------------------------------------------
        # RIGHT SIDE
        # --------------------------------------------------------

        right = QVBoxLayout()

        trajectory_box = QGroupBox(
            "Saved Trajectories"
        )

        trajectory_layout = QVBoxLayout(
            trajectory_box
        )

        self.trajectory_list = QListWidget()

        self.trajectory_list.currentTextChanged.connect(
            self.inspect_trajectory
        )

        trajectory_layout.addWidget(
            self.trajectory_list
        )

        trajectory_buttons = QHBoxLayout()

        refresh_button = QPushButton(
            "Refresh"
        )

        refresh_button.clicked.connect(
            self.refresh_all
        )

        go_button = QPushButton(
            "Go To Start"
        )

        go_button.clicked.connect(
            self.go_to_start
        )

        execute_button = QPushButton(
            "Execute"
        )

        execute_button.clicked.connect(
            self.execute_trajectory
        )

        delete_button = QPushButton(
            "Delete"
        )

        delete_button.clicked.connect(
            self.delete_trajectory
        )

        trajectory_buttons.addWidget(
            refresh_button
        )

        trajectory_buttons.addWidget(
            go_button
        )

        trajectory_buttons.addWidget(
            execute_button
        )

        trajectory_buttons.addWidget(
            delete_button
        )

        trajectory_layout.addLayout(
            trajectory_buttons
        )

        right.addWidget(
            trajectory_box
        )

        # --------------------------------------------------------
        # TRAJECTORY DETAILS
        # --------------------------------------------------------

        details_box = QGroupBox(
            "Trajectory Details"
        )

        details_layout = QVBoxLayout(
            details_box
        )

        self.details_label = QLabel(
            "Select a saved trajectory."
        )

        self.details_label.setWordWrap(
            True
        )

        details_layout.addWidget(
            self.details_label
        )

        right.addWidget(
            details_box
        )

        layout.addLayout(
            left,
            1,
        )

        layout.addLayout(
            right,
            1,
        )

    # ============================================================
    # OPERATE TAB
    # ============================================================

    def build_operate_tab(self):

        layout = QVBoxLayout(
            self.operate_tab
        )

        heading = QLabel(
            "Experiment Operation"
        )

        heading.setStyleSheet(
            "font-size: 20px; "
            "font-weight: bold;"
        )

        layout.addWidget(
            heading
        )

        layout.addWidget(
            QLabel(
                "Build a cycle from validated saved trajectories."
            )
        )

        cycle_box = QGroupBox(
            "Experiment Cycle"
        )

        cycle_layout = QVBoxLayout(
            cycle_box
        )

        # --------------------------------------------------------
        # TRAJECTORY STEPS
        # --------------------------------------------------------

        self.cycle_trajectory_boxes = []

        for i in range(4):

            row = QHBoxLayout()

            label = QLabel(
                f"Step {i + 1}"
            )

            trajectory_box = QComboBox()

            trajectory_box.addItem(
                "-- select trajectory --"
            )

            trajectory_box.currentTextChanged.connect(
                self.invalidate_cycle_validation
            )

            self.cycle_trajectory_boxes.append(
                trajectory_box
            )

            row.addWidget(
                label
            )

            row.addWidget(
                trajectory_box
            )

            cycle_layout.addLayout(
                row
            )

        # --------------------------------------------------------
        # VALIDATION
        # --------------------------------------------------------

        self.validate_cycle_button = QPushButton(
            "VALIDATE CYCLE"
        )

        self.validate_cycle_button.clicked.connect(
            self.validate_cycle
        )

        cycle_layout.addWidget(
            self.validate_cycle_button
        )

        self.cycle_status_label = QLabel(
            "Cycle not validated."
        )

        self.cycle_status_label.setWordWrap(
            True
        )

        cycle_layout.addWidget(
            self.cycle_status_label
        )

        # --------------------------------------------------------
        # CYCLE PROGRESS
        # --------------------------------------------------------

        self.cycle_progress = QProgressBar()

        self.cycle_progress.setRange(
            0,
            100,
        )

        self.cycle_progress.setValue(
            0
        )

        cycle_layout.addWidget(
            self.cycle_progress
        )

        # --------------------------------------------------------
        # RUN
        # --------------------------------------------------------

        self.run_cycle_button = QPushButton(
            "RUN CYCLE"
        )

        self.run_cycle_button.clicked.connect(
            self.run_cycle
        )

        self.run_cycle_button.setEnabled(
            False
        )

        cycle_layout.addWidget(
            self.run_cycle_button
        )

        layout.addWidget(
            cycle_box
        )

        layout.addStretch()

    # ============================================================
    # CYCLE VALIDATION
    # ============================================================

    def invalidate_cycle_validation(
        self,
        *_,
    ):

        self.validated_cycle_names = []

        self.run_cycle_button.setEnabled(
            False
        )

        self.cycle_progress.setValue(
            0
        )

        self.cycle_status_label.setText(
            "Cycle not validated."
        )

    def validate_cycle(self):

        trajectory_names = [
            box.currentText().strip()
            for box in self.cycle_trajectory_boxes
        ]

        trajectory_names = [
            name
            for name in trajectory_names
            if name
            and name != "-- select trajectory --"
        ]

        if len(trajectory_names) < 2:

            self.validated_cycle_names = []

            self.cycle_status_label.setText(
                "INVALID: select at least two trajectories."
            )

            self.run_cycle_button.setEnabled(
                False
            )

            return

        if not self.node.inspect_trajectory_client.service_is_ready():

            self.validated_cycle_names = []

            self.cycle_status_label.setText(
                "Cannot validate: "
                "trajectory inspection service is unavailable."
            )

            self.run_cycle_button.setEnabled(
                False
            )

            return

        self.cycle_status_label.setText(
            "Validating cycle..."
        )

        self.run_cycle_button.setEnabled(
            False
        )

        self.validated_cycle_names = []

        self.pending_cycle_names = (
            trajectory_names
        )

        self.pending_cycle_details = {}

        for index, name in enumerate(
            trajectory_names
        ):

            request = InspectTrajectory.Request()

            request.name = name

            future = (
                self.node.inspect_trajectory_client.call_async(
                    request
                )
            )

            future.add_done_callback(
                lambda future,
                index=index,
                name=name:
                self.cycle_inspection_received(
                    future,
                    index,
                    name,
                )
            )

    def cycle_inspection_received(
        self,
        future,
        index,
        name,
    ):

        try:

            response = future.result()

            if not response.success:

                self.validated_cycle_names = []

                self.cycle_status_label.setText(
                    f"INVALID: could not inspect '{name}'."
                )

                self.run_cycle_button.setEnabled(
                    False
                )

                return

            self.pending_cycle_details[index] = {
                "name": name,
                "start_pose": response.start_pose,
                "target_pose": response.target_pose,
            }

            if (
                len(self.pending_cycle_details)
                == len(self.pending_cycle_names)
            ):

                self.finish_cycle_validation()

        except Exception as error:

            self.validated_cycle_names = []

            self.cycle_status_label.setText(
                f"Validation failed: {error}"
            )

            self.run_cycle_button.setEnabled(
                False
            )

    def finish_cycle_validation(self):

        ordered = [
            self.pending_cycle_details[i]
            for i in range(
                len(self.pending_cycle_names)
            )
        ]

        # --------------------------------------------------------
        # CHECK TRAJECTORY CONTINUITY
        # --------------------------------------------------------

        for i in range(
            len(ordered) - 1
        ):

            current = ordered[i]
            following = ordered[i + 1]

            if (
                current["target_pose"]
                != following["start_pose"]
            ):

                self.validated_cycle_names = []

                self.cycle_status_label.setText(
                    "INVALID CYCLE\n\n"
                    f"Step {i + 1}: "
                    f"{current['name']}\n"
                    f"ends at: "
                    f"{current['target_pose']}\n\n"
                    f"Step {i + 2}: "
                    f"{following['name']}\n"
                    f"starts at: "
                    f"{following['start_pose']}"
                )

                self.run_cycle_button.setEnabled(
                    False
                )

                return

        # --------------------------------------------------------
        # VALID CYCLE
        # --------------------------------------------------------

        description = []

        for i, trajectory in enumerate(
            ordered
        ):

            description.append(
                f"{i + 1}. "
                f"{trajectory['name']} "
                f"("
                f"{trajectory['start_pose']} "
                f"→ "
                f"{trajectory['target_pose']}"
                f")"
            )

        self.validated_cycle_names = [
            trajectory["name"]
            for trajectory in ordered
        ]

        self.cycle_status_label.setText(
            "VALID CYCLE\n\n"
            + "\n".join(description)
        )

        self.run_cycle_button.setEnabled(
            True
        )

        self.cycle_progress.setValue(
            0
        )

    # ============================================================
    # CYCLE EXECUTION
    # ============================================================

    def run_cycle(self):

        if not self.validated_cycle_names:

            self.set_status(
                "Validate the cycle before running."
            )

            return

        if not self.node.go_to_start_client.server_is_ready():

            self.set_status(
                "Go-to-start server is not running."
            )

            return

        if not self.node.execute_client.server_is_ready():

            self.set_status(
                "Execution server is not running."
            )

            return

        answer = QMessageBox.question(
            self,
            "Run Experiment Cycle",
            "Run the validated experiment cycle?",
            QMessageBox.Yes | QMessageBox.No,
        )

        if answer != QMessageBox.Yes:
            return

        self.run_cycle_button.setEnabled(
            False
        )

        self.validate_cycle_button.setEnabled(
            False
        )

        for box in self.cycle_trajectory_boxes:

            box.setEnabled(
                False
            )

        self.cycle_progress.setValue(
            0
        )

        self.cycle_current_index = 0

        first_trajectory = (
            self.validated_cycle_names[0]
        )

        self.cycle_status_label.setText(
            "PREPARING CYCLE\n\n"
            f"Going to start of:\n"
            f"{first_trajectory}"
        )

        self.set_status(
            f"Going to start of '{first_trajectory}'..."
        )

        goal = GoToTrajectoryStart.Goal()

        goal.trajectory_name = (
            first_trajectory
        )

        future = (
            self.node.go_to_start_client.send_goal_async(
                goal,
                feedback_callback=self.cycle_motion_feedback,
            )
        )

        future.add_done_callback(
            self.cycle_go_to_start_goal_response
        )

    def cycle_motion_feedback(
        self,
        feedback_msg,
    ):

        self.set_status(
            feedback_msg.feedback.status
        )

    def cycle_go_to_start_goal_response(
        self,
        future,
    ):

        try:

            goal_handle = future.result()

            if not goal_handle.accepted:

                self.cycle_failed(
                    "Go-to-start request was rejected."
                )

                return

            result_future = (
                goal_handle.get_result_async()
            )

            result_future.add_done_callback(
                self.cycle_go_to_start_result
            )

        except Exception as error:

            self.cycle_failed(
                f"Go-to-start failed: {error}"
            )

    def cycle_go_to_start_result(
        self,
        future,
    ):

        try:

            result = (
                future.result().result
            )

            if not result.success:

                self.cycle_failed(
                    "Could not reach cycle start: "
                    + result.message
                )

                return

            self.execute_cycle_step()

        except Exception as error:

            self.cycle_failed(
                f"Go-to-start failed: {error}"
            )

    def execute_cycle_step(self):

        if (
            self.cycle_current_index
            >= len(self.validated_cycle_names)
        ):

            self.cycle_complete()

            return

        name = self.validated_cycle_names[
            self.cycle_current_index
        ]

        step_number = (
            self.cycle_current_index + 1
        )

        total_steps = len(
            self.validated_cycle_names
        )

        self.cycle_status_label.setText(
            "RUNNING CYCLE\n\n"
            f"Step {step_number} "
            f"of {total_steps}\n"
            f"{name}"
        )

        self.set_status(
            f"Executing cycle step "
            f"{step_number}/{total_steps}: "
            f"'{name}'"
        )

        progress = int(
            self.cycle_current_index
            / total_steps
            * 100
        )

        self.cycle_progress.setValue(
            progress
        )

        goal = ExecuteTrajectory.Goal()

        goal.trajectory_name = name

        future = (
            self.node.execute_client.send_goal_async(
                goal,
                feedback_callback=self.cycle_motion_feedback,
            )
        )

        future.add_done_callback(
            self.cycle_execute_goal_response
        )

    def cycle_execute_goal_response(
        self,
        future,
    ):

        try:

            goal_handle = future.result()

            if not goal_handle.accepted:

                self.cycle_failed(
                    "Trajectory execution was rejected."
                )

                return

            result_future = (
                goal_handle.get_result_async()
            )

            result_future.add_done_callback(
                self.cycle_execute_result
            )

        except Exception as error:

            self.cycle_failed(
                f"Trajectory execution failed: {error}"
            )

    def cycle_execute_result(
        self,
        future,
    ):

        try:

            result = (
                future.result().result
            )

            if not result.success:

                self.cycle_failed(
                    "Cycle stopped: "
                    + result.message
                )

                return

            self.cycle_current_index += 1

            completed = (
                self.cycle_current_index
            )

            total = len(
                self.validated_cycle_names
            )

            progress = int(
                completed
                / total
                * 100
            )

            self.cycle_progress.setValue(
                progress
            )

            self.execute_cycle_step()

        except Exception as error:

            self.cycle_failed(
                f"Cycle execution failed: {error}"
            )

    def cycle_complete(self):

        self.cycle_progress.setValue(
            100
        )

        self.cycle_status_label.setText(
            "CYCLE COMPLETE"
        )

        self.set_status(
            "Experiment cycle completed successfully."
        )

        self.run_cycle_button.setEnabled(
            True
        )

        self.validate_cycle_button.setEnabled(
            True
        )

        for box in self.cycle_trajectory_boxes:

            box.setEnabled(
                True
            )

    def cycle_failed(
        self,
        message,
    ):

        self.cycle_status_label.setText(
            "CYCLE STOPPED\n\n"
            + message
        )

        self.set_status(
            message
        )

        self.run_cycle_button.setEnabled(
            True
        )

        self.validate_cycle_button.setEnabled(
            True
        )

        for box in self.cycle_trajectory_boxes:

            box.setEnabled(
                True
            )

    # ============================================================
    # DEFAULT TRAJECTORY NAME
    # ============================================================

    def update_default_trajectory_name(self):

        start = (
            self.start_pose_box
            .currentText()
            .strip()
        )

        target = (
            self.target_pose_box
            .currentText()
            .strip()
        )

        if start and target:

            self.trajectory_name_entry.setText(
                f"{start}_TO_{target}"
            )

    # ============================================================
    # DIAGNOSTICS TAB
    # ============================================================

    def build_diagnostics_tab(self):

        layout = QVBoxLayout(
            self.diagnostics_tab
        )

        heading = QLabel(
            "System Diagnostics"
        )

        heading.setStyleSheet(
            "font-size: 20px; "
            "font-weight: bold;"
        )

        layout.addWidget(
            heading
        )

        self.backend_status = QLabel(
            "Workbench Backend: checking..."
        )

        layout.addWidget(
            self.backend_status
        )

        layout.addWidget(
            QLabel(
                "Meca500: future status\n"
                "Laser sensor: future status\n"
                "Force sensor: future status\n"
                "Helico actuator: future status"
            )
        )

        layout.addStretch()

    # ============================================================
    # ROS LOOP
    # ============================================================

    def spin_ros(self):

        if rclpy.ok():

            rclpy.spin_once(
                self.node,
                timeout_sec=0.0,
            )

    # ============================================================
    # BACKEND
    # ============================================================

    def wait_for_backend(self):

        ready = (
            self.node.list_poses_client.service_is_ready()
            and
            self.node.list_trajectories_client.service_is_ready()
        )

        if ready:

            self.backend_timer.stop()

            self.backend_status.setText(
                "Workbench Backend: CONNECTED"
            )

            self.set_status(
                "Backend connected."
            )

            self.refresh_all()

        else:

            self.backend_status.setText(
                "Workbench Backend: WAITING"
            )

            self.set_status(
                "Waiting for workbench backend..."
            )

    # ============================================================
    # STATUS
    # ============================================================

    def set_status(
        self,
        text,
    ):

        self.status_label.setText(
            text
        )

    # ============================================================
    # REFRESH
    # ============================================================

    def refresh_all(self):

        self.refresh_poses()
        self.refresh_trajectories()

    def refresh_poses(self):

        if not self.node.list_poses_client.service_is_ready():
            return

        future = (
            self.node.list_poses_client.call_async(
                ListPoses.Request()
            )
        )

        future.add_done_callback(
            self.poses_received
        )

    def poses_received(
        self,
        future,
    ):

        try:

            response = future.result()

            names = list(
                response.names
            )

            types = list(
                response.types
            )

            joint_poses = [
                name
                for name, pose_type
                in zip(names, types)
                if pose_type == "joint"
            ]

            self.pose_list.clear()

            self.pose_list.addItems(
                names
            )

            self.start_pose_box.clear()

            self.start_pose_box.addItems(
                joint_poses
            )

            self.target_pose_box.clear()

            self.target_pose_box.addItems(
                names
            )

            if "meca_zero" in joint_poses:

                self.start_pose_box.setCurrentText(
                    "meca_zero"
                )

            if "meca_demo" in names:

                self.target_pose_box.setCurrentText(
                    "meca_demo"
                )

        except Exception as error:

            self.set_status(
                f"Pose refresh failed: {error}"
            )

    def refresh_trajectories(self):

        if not self.node.list_trajectories_client.service_is_ready():
            return

        future = (
            self.node.list_trajectories_client.call_async(
                ListTrajectories.Request()
            )
        )

        future.add_done_callback(
            self.trajectories_received
        )

    def trajectories_received(
        self,
        future,
    ):

        try:

            response = future.result()

            names = list(
                response.names
            )

            # ----------------------------------------------------
            # CONFIGURE TAB
            # ----------------------------------------------------

            self.trajectory_list.clear()

            self.trajectory_list.addItems(
                names
            )

            # ----------------------------------------------------
            # OPERATE TAB
            # ----------------------------------------------------

            for box in self.cycle_trajectory_boxes:

                current = (
                    box.currentText()
                )

                box.clear()

                box.addItem(
                    "-- select trajectory --"
                )

                box.addItems(
                    names
                )

                if current in names:

                    box.setCurrentText(
                        current
                    )

            self.set_status(
                "Ready"
            )

        except Exception as error:

            self.set_status(
                f"Trajectory refresh failed: {error}"
            )

    # ============================================================
    # PLAN
    # ============================================================

    def plan_trajectory(self):

        start_pose = (
            self.start_pose_box.currentText()
        )

        target_pose = (
            self.target_pose_box.currentText()
        )

        trajectory_name = (
            self.trajectory_name_entry
            .text()
            .strip()
        )

        if not start_pose:

            self.set_status(
                "Select a start pose."
            )

            return

        if not target_pose:

            self.set_status(
                "Select a target pose."
            )

            return

        if not trajectory_name:

            self.set_status(
                "Enter a trajectory name."
            )

            return

        if not self.node.plan_client.server_is_ready():

            self.set_status(
                "Planning server is not running."
            )

            return

        goal = PlanTrajectory.Goal()

        goal.start_pose = (
            start_pose
        )

        goal.target_pose = (
            target_pose
        )

        goal.trajectory_name = (
            trajectory_name
        )

        goal.num_candidates = (
            self.candidate_spinbox.value()
        )

        speed = (
            self.speed_spinbox.value()
            / 100.0
        )

        goal.velocity_scaling = (
            speed
        )

        goal.acceleration_scaling = (
            speed
        )

        goal.minimum_required_clearance = (
            self.clearance_spinbox.value()
            / 1000.0
        )

        goal.weight_clearance = 0.4
        goal.weight_smoothness = 0.3
        goal.weight_path_length = 0.2
        goal.weight_duration = 0.1

        self.progress.setValue(
            0
        )

        self.set_status(
            "Sending planning request..."
        )

        future = (
            self.node.plan_client.send_goal_async(
                goal,
                feedback_callback=self.plan_feedback,
            )
        )

        future.add_done_callback(
            self.plan_goal_response
        )

    def plan_feedback(
        self,
        feedback_msg,
    ):

        feedback = (
            feedback_msg.feedback
        )

        self.set_status(
            feedback.status
        )

        status = (
            feedback.status.lower()
        )

        # --------------------------------------------------------
        # INITIALISATION
        # --------------------------------------------------------

        if "initialising" in status:

            progress = 5

        # --------------------------------------------------------
        # CANDIDATE PLANNING
        # --------------------------------------------------------

        elif (
            feedback.total_candidates > 0
            and
            (
                "planning candidate" in status
                or
                "evaluating candidate" in status
            )
        ):

            total_steps = (
                feedback.total_candidates
                * 2
            )

            candidate_index = max(
                feedback.current_candidate - 1,
                0,
            )

            if "planning candidate" in status:

                completed_steps = (
                    candidate_index * 2
                )

            else:

                completed_steps = (
                    candidate_index * 2
                    + 1
                )

            fraction = (
                completed_steps
                / total_steps
            )

            progress = int(
                5
                + fraction * 80
            )

        # --------------------------------------------------------
        # FINAL PROCESSING
        # --------------------------------------------------------

        elif "scoring" in status:

            progress = 90

        elif "saving" in status:

            progress = 97

        else:

            progress = min(
                self.progress.value(),
                99,
            )

        self.progress.setValue(
            min(
                progress,
                99,
            )
        )

    def plan_goal_response(
        self,
        future,
    ):

        try:

            goal_handle = (
                future.result()
            )

            if not goal_handle.accepted:

                self.set_status(
                    "Planning goal rejected."
                )

                return

            result_future = (
                goal_handle.get_result_async()
            )

            result_future.add_done_callback(
                self.plan_result
            )

        except Exception as error:

            self.set_status(
                f"Planning request failed: {error}"
            )

    def plan_result(
        self,
        future,
    ):

        try:

            result = (
                future.result().result
            )

            if result.success:

                self.progress.setValue(
                    100
                )

                self.set_status(
                    f"Planned "
                    f"'{result.trajectory_name}' "
                    f"| candidate "
                    f"{result.selected_candidate} "
                    f"| clearance "
                    f"{result.minimum_clearance * 1000:.1f} mm"
                )

                self.refresh_trajectories()

            else:

                self.set_status(
                    "Planning failed: "
                    + result.message
                )

        except Exception as error:

            self.set_status(
                f"Planning result failed: {error}"
            )

    # ============================================================
    # TRAJECTORY DETAILS
    # ============================================================

    def selected_trajectory(self):

        item = (
            self.trajectory_list.currentItem()
        )

        if item is None:

            self.set_status(
                "Select a saved trajectory first."
            )

            return None

        return item.text()

    def inspect_trajectory(
        self,
        name,
    ):

        if not name:
            return

        if not self.node.inspect_trajectory_client.service_is_ready():
            return

        request = InspectTrajectory.Request()

        request.name = name

        future = (
            self.node.inspect_trajectory_client.call_async(
                request
            )
        )

        future.add_done_callback(
            self.trajectory_details_received
        )

    def trajectory_details_received(
        self,
        future,
    ):

        try:

            response = future.result()

            if not response.success:

                self.details_label.setText(
                    response.message
                )

                return

            clearance_pass = (
                response.minimum_clearance
                >=
                response.minimum_required_clearance
            )

            clearance_status = (
                "PASS"
                if clearance_pass
                else "FAIL"
            )

            self.details_label.setText(
                f"{response.start_pose}  →  "
                f"{response.target_pose}\n\n"

                f"PLANNING SETTINGS\n"

                f"Speed scaling: "
                f"{response.velocity_scaling * 100:.0f}%\n"

                f"Acceleration scaling: "
                f"{response.acceleration_scaling * 100:.0f}%\n"

                f"Required clearance: "
                f"{response.minimum_required_clearance * 1000:.1f} mm\n"

                f"Candidates: "
                f"{response.num_candidates}\n"

                f"Planner: "
                f"{response.planner_id}\n\n"

                f"SCORING WEIGHTS\n"

                f"Clearance: "
                f"{response.weight_clearance:.2f}\n"

                f"Smoothness: "
                f"{response.weight_smoothness:.2f}\n"

                f"Path length: "
                f"{response.weight_path_length:.2f}\n"

                f"Duration: "
                f"{response.weight_duration:.2f}\n\n"

                f"RESULT\n"

                f"Minimum clearance: "
                f"{response.minimum_clearance * 1000:.2f} mm "
                f"[{clearance_status}]\n"

                f"Closest objects: "
                f"{response.closest_object_a} ↔ "
                f"{response.closest_object_b}\n"

                f"Smoothness: "
                f"{response.smoothness:.6f}\n"

                f"Path length: "
                f"{response.path_length:.4f} rad\n"

                f"Duration: "
                f"{response.duration:.2f} s"
            )

        except Exception as error:

            self.details_label.setText(
                f"Inspection failed: {error}"
            )

    # ============================================================
    # GO TO START
    # ============================================================

    def go_to_start(self):

        name = (
            self.selected_trajectory()
        )

        if name is None:
            return

        if not self.node.go_to_start_client.server_is_ready():

            self.set_status(
                "Go-to-start server is not running."
            )

            return

        goal = GoToTrajectoryStart.Goal()

        goal.trajectory_name = (
            name
        )

        self.set_status(
            f"Going to start of '{name}'..."
        )

        future = (
            self.node.go_to_start_client.send_goal_async(
                goal,
                feedback_callback=self.motion_feedback,
            )
        )

        future.add_done_callback(
            self.motion_goal_response
        )

    # ============================================================
    # EXECUTE SINGLE TRAJECTORY
    # ============================================================

    def execute_trajectory(self):

        name = (
            self.selected_trajectory()
        )

        if name is None:
            return

        answer = QMessageBox.question(
            self,
            "Execute Trajectory",
            f"Execute trajectory '{name}'?",
            QMessageBox.Yes | QMessageBox.No,
        )

        if answer != QMessageBox.Yes:
            return

        if not self.node.execute_client.server_is_ready():

            self.set_status(
                "Execution server is not running."
            )

            return

        goal = ExecuteTrajectory.Goal()

        goal.trajectory_name = (
            name
        )

        self.set_status(
            f"Executing '{name}'..."
        )

        future = (
            self.node.execute_client.send_goal_async(
                goal,
                feedback_callback=self.motion_feedback,
            )
        )

        future.add_done_callback(
            self.motion_goal_response
        )

    def motion_feedback(
        self,
        feedback_msg,
    ):

        self.set_status(
            feedback_msg.feedback.status
        )

    def motion_goal_response(
        self,
        future,
    ):

        try:

            goal_handle = (
                future.result()
            )

            if not goal_handle.accepted:

                self.set_status(
                    "Motion goal rejected."
                )

                return

            result_future = (
                goal_handle.get_result_async()
            )

            result_future.add_done_callback(
                self.motion_result
            )

        except Exception as error:

            self.set_status(
                f"Motion request failed: {error}"
            )

    def motion_result(
        self,
        future,
    ):

        try:

            result = (
                future.result().result
            )

            self.set_status(
                result.message
            )

        except Exception as error:

            self.set_status(
                f"Motion result failed: {error}"
            )

    # ============================================================
    # SAVE POSE
    # ============================================================

    def capture_pose(self):

        name, ok = QInputDialog.getText(
            self,
            "Save Current Robot Pose",
            "Pose name:",
        )

        name = (
            name.strip()
        )

        if not ok or not name:
            return

        if not self.node.capture_pose_client.service_is_ready():

            self.set_status(
                "Capture pose service is not ready."
            )

            return

        request = CapturePose.Request()

        request.name = (
            name
        )

        future = (
            self.node.capture_pose_client.call_async(
                request
            )
        )

        future.add_done_callback(
            self.capture_pose_result
        )

    def capture_pose_result(
        self,
        future,
    ):

        try:

            response = (
                future.result()
            )

            self.set_status(
                response.message
            )

            if response.success:

                self.refresh_poses()

        except Exception as error:

            self.set_status(
                f"Save pose failed: {error}"
            )

    # ============================================================
    # DELETE POSE
    # ============================================================

    def delete_pose(self):

        item = (
            self.pose_list.currentItem()
        )

        if item is None:

            self.set_status(
                "Select a pose to delete."
            )

            return

        name = (
            item.text()
        )

        answer = QMessageBox.question(
            self,
            "Delete Pose",
            f"Delete pose '{name}'?",
            QMessageBox.Yes | QMessageBox.No,
        )

        if answer != QMessageBox.Yes:
            return

        request = DeletePose.Request()

        request.name = (
            name
        )

        future = (
            self.node.delete_pose_client.call_async(
                request
            )
        )

        future.add_done_callback(
            self.delete_pose_result
        )

    def delete_pose_result(
        self,
        future,
    ):

        try:

            response = (
                future.result()
            )

            self.set_status(
                response.message
            )

            if response.success:

                self.refresh_poses()

        except Exception as error:

            self.set_status(
                f"Delete pose failed: {error}"
            )

    # ============================================================
    # DELETE TRAJECTORY
    # ============================================================

    def delete_trajectory(self):

        name = (
            self.selected_trajectory()
        )

        if name is None:
            return

        answer = QMessageBox.question(
            self,
            "Delete Trajectory",
            f"Delete trajectory '{name}'?",
            QMessageBox.Yes | QMessageBox.No,
        )

        if answer != QMessageBox.Yes:
            return

        request = DeleteTrajectory.Request()

        request.name = (
            name
        )

        future = (
            self.node.delete_trajectory_client.call_async(
                request
            )
        )

        future.add_done_callback(
            self.delete_trajectory_result
        )

    def delete_trajectory_result(
        self,
        future,
    ):

        try:

            response = (
                future.result()
            )

            self.set_status(
                response.message
            )

            if response.success:

                self.details_label.setText(
                    "Select a saved trajectory."
                )

                self.refresh_trajectories()

        except Exception as error:

            self.set_status(
                f"Delete trajectory failed: {error}"
            )


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

        exit_code = (
            app.exec()
        )

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()

    sys.exit(
        exit_code
    )


if __name__ == "__main__":

    main()