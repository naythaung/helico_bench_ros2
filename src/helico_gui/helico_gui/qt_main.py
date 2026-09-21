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

        # Services
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

        # Actions
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

        self.setWindowTitle("Helico Bench")
        self.resize(1100, 800)

        self.build_gui()

        # ROS spin inside Qt event loop
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(20)

        # Retry backend connection
        self.backend_timer = QTimer(self)
        self.backend_timer.timeout.connect(self.wait_for_backend)
        self.backend_timer.start(500)

    # ============================================================
    # GUI
    # ============================================================

    def build_gui(self):

        central = QWidget()
        self.setCentralWidget(central)

        outer = QVBoxLayout(central)

        title = QLabel("HELICO BENCH")
        title.setStyleSheet(
            "font-size: 26px; font-weight: bold; margin: 10px;"
        )
        outer.addWidget(title)

        self.tabs = QTabWidget()
        outer.addWidget(self.tabs)

        self.configure_tab = QWidget()
        self.operate_tab = QWidget()
        self.diagnostics_tab = QWidget()

        self.tabs.addTab(self.operate_tab, "Operate")
        self.tabs.addTab(self.configure_tab, "Configure")
        self.tabs.addTab(self.diagnostics_tab, "Diagnostics")

        self.build_configure_tab()
        self.build_operate_tab()
        self.build_diagnostics_tab()

        self.status_label = QLabel("Starting...")
        self.status_label.setStyleSheet(
            "padding: 8px; font-weight: bold;"
        )

        outer.addWidget(self.status_label)

    # ============================================================
    # CONFIGURE TAB
    # ============================================================

    def build_configure_tab(self):

        layout = QHBoxLayout(self.configure_tab)

        # --------------------------------------------------------
        # LEFT SIDE
        # --------------------------------------------------------

        left = QVBoxLayout()

        planning_box = QGroupBox("Trajectory Planning")
        planning_form = QFormLayout(planning_box)

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
        self.candidate_spinbox.setRange(1, 50)
        self.candidate_spinbox.setValue(3)
        
        self.speed_spinbox = QSpinBox()
        self.speed_spinbox.setRange(1, 100)
        self.speed_spinbox.setValue(20)
        self.speed_spinbox.setSuffix(" %")

        self.clearance_spinbox = QSpinBox()
        self.clearance_spinbox.setRange(0, 200)
        self.clearance_spinbox.setValue(10)
        self.clearance_spinbox.setSuffix(" mm")
        

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
        self.progress.setRange(0, 100)

        planning_form.addRow(
            "Progress",
            self.progress,
        )
        
        planning_form.addRow(
            "Speed",
            self.speed_spinbox,
        )

        planning_form.addRow(
            "Minimum clearance",
            self.clearance_spinbox,
        )

        left.addWidget(planning_box)

        # --------------------------------------------------------
        # POSE LIBRARY
        # --------------------------------------------------------

        pose_box = QGroupBox("Pose Library")
        pose_layout = QVBoxLayout(pose_box)

        self.pose_list = QListWidget()
        pose_layout.addWidget(self.pose_list)

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

        left.addWidget(pose_box)

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
            "font-size: 20px; font-weight: bold;"
        )

        layout.addWidget(
            heading
        )

        layout.addWidget(
            QLabel(
                "Validated experiment cycles will run here."
            )
        )

        cycle_box = QGroupBox(
            "Experiment Cycle"
        )

        cycle_layout = QVBoxLayout(
            cycle_box
        )

        cycle_layout.addWidget(
            QLabel(
                "HOME\n"
                "  ↓\n"
                "APPROACH\n"
                "  ↓\n"
                "MEASUREMENT\n"
                "  ↓\n"
                "RETRACT\n"
                "  ↓\n"
                "HOME"
            )
        )

        self.run_cycle_button = QPushButton(
            "RUN CYCLE"
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

    def update_default_trajectory_name(self):

        start = self.start_pose_box.currentText().strip()
        target = self.target_pose_box.currentText().strip()

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
            "font-size: 20px; font-weight: bold;"
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

            self.trajectory_list.clear()

            self.trajectory_list.addItems(
                list(response.names)
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
            self.trajectory_name_entry.text().strip()
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

        goal.start_pose = start_pose
        goal.target_pose = target_pose
        goal.trajectory_name = trajectory_name

        goal.num_candidates = (
            self.candidate_spinbox.value()
        )

        speed = (
            self.speed_spinbox.value()
            / 100.0
        )

        goal.velocity_scaling = speed
        goal.acceleration_scaling = speed

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

        feedback = feedback_msg.feedback

        self.set_status(
            feedback.status
        )

        if feedback.total_candidates > 0:

            progress = int(
                feedback.current_candidate
                /
                feedback.total_candidates
                *
                100
            )

            self.progress.setValue(
                progress
            )

    def plan_goal_response(
        self,
        future,
    ):

        goal_handle = future.result()

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

    def plan_result(
        self,
        future,
    ):

        result = (
            future.result().result
        )

        if result.success:

            self.progress.setValue(
                100
            )

            self.set_status(
                f"Planned '{result.trajectory_name}' "
                f"| candidate {result.selected_candidate} "
                f"| clearance "
                f"{result.minimum_clearance * 1000:.1f} mm"
            )

            self.refresh_trajectories()

        else:

            self.set_status(
                "Planning failed: "
                + result.message
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

            self.details_label.setText(
                f"Start pose: {response.start_pose}\n"
                f"Target pose: {response.target_pose}\n"
                f"Candidates: {response.num_candidates}\n"
                f"Minimum clearance: "
                f"{response.minimum_clearance * 1000:.1f} mm\n"
                f"Closest objects: "
                f"{response.closest_object_a} ↔ "
                f"{response.closest_object_b}\n"
                f"Smoothness: "
                f"{response.smoothness:.6f}\n"
                f"Path length: "
                f"{response.path_length:.4f} rad\n"
                f"Duration: "
                f"{response.duration:.3f} s"
            )

        except Exception as error:

            self.details_label.setText(
                f"Inspection failed: {error}"
            )

    # ============================================================
    # GO TO START
    # ============================================================

    def go_to_start(self):

        name = self.selected_trajectory()

        if name is None:
            return

        if not self.node.go_to_start_client.server_is_ready():

            self.set_status(
                "Go-to-start server is not running."
            )

            return

        goal = GoToTrajectoryStart.Goal()
        goal.trajectory_name = name

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
    # EXECUTE
    # ============================================================

    def execute_trajectory(self):

        name = self.selected_trajectory()

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
        goal.trajectory_name = name

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

        goal_handle = future.result()

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

    def motion_result(
        self,
        future,
    ):

        result = future.result().result

        self.set_status(
            result.message
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

        name = name.strip()

        if not ok or not name:
            return

        if not self.node.capture_pose_client.service_is_ready():
            self.set_status(
                "Capture pose service is not ready."
            )
            return

        request = CapturePose.Request()
        request.name = name

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

        response = future.result()

        self.set_status(
            response.message
        )

        if response.success:
            self.refresh_poses()

    # ============================================================
    # DELETE POSE
    # ============================================================

    def delete_pose(self):

        item = self.pose_list.currentItem()

        if item is None:

            self.set_status(
                "Select a pose to delete."
            )

            return

        name = item.text()

        answer = QMessageBox.question(
            self,
            "Delete Pose",
            f"Delete pose '{name}'?",
            QMessageBox.Yes | QMessageBox.No,
        )

        if answer != QMessageBox.Yes:
            return

        request = DeletePose.Request()
        request.name = name

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

        response = future.result()

        self.set_status(
            response.message
        )

        if response.success:
            self.refresh_poses()

    # ============================================================
    # DELETE TRAJECTORY
    # ============================================================

    def delete_trajectory(self):

        name = self.selected_trajectory()

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
        request.name = name

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

        response = future.result()

        self.set_status(
            response.message
        )

        if response.success:

            self.details_label.setText(
                "Select a saved trajectory."
            )

            self.refresh_trajectories()


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
