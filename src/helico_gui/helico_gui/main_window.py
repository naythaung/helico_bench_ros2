import time

import rclpy

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
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
    QCheckBox,
)

from helico_gui.cycle_controller import CycleController
from helico_gui.experiment_logger import ExperimentLogger
from helico_gui.workflows.cycle_workflow import CycleWorkflowMixin
from helico_gui.workflows.logging_workflow import LoggingWorkflowMixin
from helico_gui.workflows.robot_workflow import RobotWorkflowMixin
from helico_gui.widgets.hardware_status import HardwareStatusWidget
from helico_gui.widgets.diagnostics_tab import DiagnosticsTab


GREEN = "#2e7d32"
ORANGE = "#ed8b00"
RED = "#c62828"
NEUTRAL = "#666666"


class HelicoWindow(
    CycleWorkflowMixin,
    LoggingWorkflowMixin,
    RobotWorkflowMixin,
    QMainWindow,
):
    """Top-level Qt window.

    This class owns layout, timers and high-level composition. Robot workflow,
    cycle workflow, ROS communication, logging and reusable status widgets are
    kept in separate modules for handover and maintenance.
    """

    def __init__(self, node):
        super().__init__()

        self.node = node
        self.cycle = CycleController()
        self.experiment_logger = ExperimentLogger()
        self.cycle_started_logging = False
        self.backend_was_ready = False

        self.setWindowTitle("Helico Bench")
        self.resize(1100, 800)

        self.build_gui()

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(20)

        self.backend_timer = QTimer(self)
        self.backend_timer.timeout.connect(self.wait_for_backend)
        self.backend_timer.start(500)

        self.sensor_display_timer = QTimer(self)
        self.sensor_display_timer.timeout.connect(self.update_sensor_display)
        self.sensor_display_timer.start(100)

        self.logging_timer = QTimer(self)
        self.logging_timer.timeout.connect(self.write_log_sample)
        self.logging_timer.setInterval(100)

    def build_gui(self):
        central = QWidget()
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)

        title = QLabel("HELICO BENCH")
        title.setStyleSheet(
            "font-size: 26px; font-weight: bold; margin: 10px;"
        )
        outer.addWidget(title)

        self.hardware_status = HardwareStatusWidget()
        outer.addWidget(self.hardware_status)

        self.tabs = QTabWidget()
        outer.addWidget(self.tabs)

        self.operate_tab = QWidget()
        self.configure_tab = QWidget()
        self.diagnostics_tab = DiagnosticsTab()

        self.tabs.addTab(self.operate_tab, "Operate")
        self.tabs.addTab(self.configure_tab, "Configure")
        self.tabs.addTab(self.diagnostics_tab, "Diagnostics")

        self.build_configure_tab()
        self.build_operate_tab()

        self.status_label = QLabel("Starting...")
        self.status_label.setStyleSheet("padding: 8px; font-weight: bold;")
        outer.addWidget(self.status_label)

    def build_configure_tab(self):
        layout = QHBoxLayout(self.configure_tab)
        left = QVBoxLayout()
        planning_box = QGroupBox('Trajectory Planning')
        planning_form = QFormLayout(planning_box)
        self.start_pose_box = QComboBox()
        self.target_pose_box = QComboBox()
        self.start_pose_box.currentTextChanged.connect(self.update_default_trajectory_name)
        self.target_pose_box.currentTextChanged.connect(self.update_default_trajectory_name)
        self.trajectory_name_entry = QLineEdit('gui_trajectory')
        self.candidate_spinbox = QSpinBox()
        self.candidate_spinbox.setRange(1, 50)
        self.candidate_spinbox.setValue(3)
        self.speed_spinbox = QSpinBox()
        self.speed_spinbox.setRange(1, 100)
        self.speed_spinbox.setValue(20)
        self.speed_spinbox.setSuffix(' %')
        self.clearance_spinbox = QSpinBox()
        self.clearance_spinbox.setRange(0, 200)
        self.clearance_spinbox.setValue(10)
        self.clearance_spinbox.setSuffix(' mm')
        planning_form.addRow('Start pose', self.start_pose_box)
        planning_form.addRow('Target pose', self.target_pose_box)
        planning_form.addRow('Trajectory name', self.trajectory_name_entry)
        planning_form.addRow('Candidates', self.candidate_spinbox)
        planning_form.addRow('Speed scaling', self.speed_spinbox)
        planning_form.addRow('Minimum clearance', self.clearance_spinbox)
        self.plan_button = QPushButton('PLAN TRAJECTORY')
        self.plan_button.clicked.connect(self.plan_trajectory)
        planning_form.addRow(self.plan_button)
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        planning_form.addRow('Progress', self.progress)
        left.addWidget(planning_box)
        pose_box = QGroupBox('Pose Library')
        pose_layout = QVBoxLayout(pose_box)
        self.pose_list = QListWidget()
        pose_layout.addWidget(self.pose_list)
        pose_buttons = QHBoxLayout()
        save_pose_button = QPushButton('Save Current Robot Pose')
        save_pose_button.clicked.connect(self.capture_pose)
        delete_pose_button = QPushButton('Delete Pose')
        delete_pose_button.clicked.connect(self.delete_pose)
        pose_buttons.addWidget(save_pose_button)
        pose_buttons.addWidget(delete_pose_button)
        pose_layout.addLayout(pose_buttons)
        left.addWidget(pose_box)
        right = QVBoxLayout()
        trajectory_box = QGroupBox('Saved Trajectories')
        trajectory_layout = QVBoxLayout(trajectory_box)
        self.trajectory_list = QListWidget()
        self.trajectory_list.currentTextChanged.connect(self.inspect_trajectory)
        trajectory_layout.addWidget(self.trajectory_list)
        trajectory_buttons = QHBoxLayout()
        refresh_button = QPushButton('Refresh')
        refresh_button.clicked.connect(self.refresh_all)
        go_button = QPushButton('Go To Start')
        go_button.clicked.connect(self.go_to_start)
        execute_button = QPushButton('Execute')
        execute_button.clicked.connect(self.execute_trajectory)
        delete_button = QPushButton('Delete')
        delete_button.clicked.connect(self.delete_trajectory)
        trajectory_buttons.addWidget(refresh_button)
        trajectory_buttons.addWidget(go_button)
        trajectory_buttons.addWidget(execute_button)
        trajectory_buttons.addWidget(delete_button)
        trajectory_layout.addLayout(trajectory_buttons)
        right.addWidget(trajectory_box)
        details_box = QGroupBox('Trajectory Details')
        details_layout = QVBoxLayout(details_box)
        self.details_label = QLabel('Select a saved trajectory.')
        self.details_label.setWordWrap(True)
        details_layout.addWidget(self.details_label)
        right.addWidget(details_box)
        layout.addLayout(left, 1)
        layout.addLayout(right, 1)

    def build_operate_tab(self):
        layout = QVBoxLayout(self.operate_tab)
        heading = QLabel('Experiment Operation')
        heading.setStyleSheet('font-size: 20px; font-weight: bold;')
        layout.addWidget(heading)
        layout.addWidget(QLabel('Build and run a validated sequence of saved trajectories.'))
        cycle_box = QGroupBox('Experiment Cycle')
        cycle_layout = QVBoxLayout(cycle_box)
        self.cycle_trajectory_boxes = []
        for i in range(4):
            row = QHBoxLayout()
            label = QLabel(f'Step {i + 1}')
            trajectory_box = QComboBox()
            trajectory_box.addItem('-- select trajectory --')
            trajectory_box.currentTextChanged.connect(self.invalidate_cycle_validation)
            self.cycle_trajectory_boxes.append(trajectory_box)
            row.addWidget(label)
            row.addWidget(trajectory_box)
            cycle_layout.addLayout(row)
        self.validate_cycle_button = QPushButton('VALIDATE CYCLE')
        self.validate_cycle_button.clicked.connect(self.validate_cycle)
        cycle_layout.addWidget(self.validate_cycle_button)
        self.pause_each_step_checkbox = QCheckBox('Pause after each step')
        self.pause_each_step_checkbox.setChecked(True)
        cycle_layout.addWidget(self.pause_each_step_checkbox)
        self.cycle_status_label = QLabel('Cycle not validated.')
        self.cycle_status_label.setWordWrap(True)
        cycle_layout.addWidget(self.cycle_status_label)
        self.cycle_progress = QProgressBar()
        self.cycle_progress.setRange(0, 100)
        self.cycle_progress.setValue(0)
        cycle_layout.addWidget(self.cycle_progress)
        controls = QHBoxLayout()
        self.run_cycle_button = QPushButton('RUN CYCLE')
        self.run_cycle_button.clicked.connect(self.run_cycle)
        self.run_cycle_button.setEnabled(False)
        self.continue_cycle_button = QPushButton('CONTINUE')
        self.continue_cycle_button.clicked.connect(self.continue_cycle)
        self.continue_cycle_button.setEnabled(False)
        self.stop_cycle_button = QPushButton('STOP CYCLE')
        self.stop_cycle_button.clicked.connect(self.stop_cycle)
        self.stop_cycle_button.setEnabled(False)
        controls.addWidget(self.run_cycle_button)
        controls.addWidget(self.continue_cycle_button)
        controls.addWidget(self.stop_cycle_button)
        cycle_layout.addLayout(controls)
        layout.addWidget(cycle_box)
        logging_box = QGroupBox('Experiment Data Logging')
        logging_layout = QVBoxLayout(logging_box)
        self.logging_status_label = QLabel('Logging: OFF')
        self.logging_status_label.setStyleSheet('font-weight: bold;')
        logging_layout.addWidget(self.logging_status_label)
        self.logging_file_label = QLabel('No log file active.')
        self.logging_file_label.setWordWrap(True)
        logging_layout.addWidget(self.logging_file_label)
        logging_buttons = QHBoxLayout()
        self.start_logging_button = QPushButton('START LOGGING')
        self.start_logging_button.clicked.connect(self.start_logging)
        self.stop_logging_button = QPushButton('STOP LOGGING')
        self.stop_logging_button.clicked.connect(self.stop_logging)
        self.stop_logging_button.setEnabled(False)
        logging_buttons.addWidget(self.start_logging_button)
        logging_buttons.addWidget(self.stop_logging_button)
        logging_layout.addLayout(logging_buttons)
        layout.addWidget(logging_box)
        layout.addStretch()

    def sensor_is_recent(self, timestamp, timeout=1.0):
        if timestamp is None:
            return False
        return time.monotonic() - timestamp <= timeout

    def update_sensor_display(self):
        laser_recent = self.sensor_is_recent(self.node.last_laser_update)
        force_recent = self.sensor_is_recent(self.node.last_force_update)
        pressure_recent = self.sensor_is_recent(self.node.last_pressure_update)

        self.hardware_status.update_values(
            laser=self.node.latest_laser,
            force=self.node.latest_force,
            pressure=self.node.latest_pressure,
            laser_recent=laser_recent,
            force_recent=force_recent,
            pressure_recent=pressure_recent,
        )

        self.diagnostics_tab.update_hardware(
            laser_recent=laser_recent,
            force_recent=force_recent,
            pressure_recent=pressure_recent,
        )

        self.update_operation_colours()

    def update_operation_colours(self):
        if self.cycle_state in ["READY", "COMPLETE"]:
            cycle_colour = GREEN
        elif self.cycle_state in [
            "PREPARING",
            "RUNNING",
            "PAUSED",
            "STOP_REQUESTED",
        ]:
            cycle_colour = ORANGE
        elif self.cycle_state in ["FAILED", "STOPPED"]:
            cycle_colour = RED
        else:
            cycle_colour = NEUTRAL

        self.cycle_status_label.setStyleSheet(
            f"font-weight: bold; color: {cycle_colour};"
        )

        logging_colour = GREEN if self.experiment_logger.active else NEUTRAL
        self.logging_status_label.setStyleSheet(
            f"font-weight: bold; color: {logging_colour};"
        )

    def spin_ros(self):
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)

    def wait_for_backend(self):
        ready = (
            self.node.list_poses_client.service_is_ready()
            and self.node.list_trajectories_client.service_is_ready()
        )

        self.diagnostics_tab.set_backend_connected(ready)

        if ready and not self.backend_was_ready:
            self.set_status("Backend connected.")
            self.refresh_all()
        elif not ready and self.backend_was_ready:
            self.set_status("Workbench backend disconnected.")
        elif not ready:
            self.set_status("Waiting for workbench backend...")

        self.backend_was_ready = ready

    def set_status(self, text):
        self.status_label.setText(text)

        status = text.lower()

        red_words = [
            "failed",
            "failure",
            "error",
            "rejected",
            "invalid",
            "unavailable",
            "could not",
            "disconnected",
        ]

        green_words = [
            "ready",
            "connected",
            "complete",
            "completed",
            "success",
            "planned",
            "saved",
        ]

        orange_words = [
            "waiting",
            "planning",
            "executing",
            "going to",
            "validating",
            "preparing",
            "paused",
            "stop requested",
            "sending",
        ]

        if any(word in status for word in red_words):
            colour = RED
        elif any(word in status for word in green_words):
            colour = GREEN
        elif any(word in status for word in orange_words):
            colour = ORANGE
        else:
            colour = NEUTRAL

        self.status_label.setStyleSheet(
            f"padding: 8px; font-weight: bold; color: {colour};"
        )

    def closeEvent(self, event):
        if self.experiment_logger.active:
            self.stop_logging()
        event.accept()
