import time

from PySide6.QtSvgWidgets import QSvgWidget
from PySide6.QtWidgets import QGridLayout

from pathlib import Path

from PySide6.QtGui import QPixmap

import rclpy

from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QDoubleSpinBox,
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
    QSlider,
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
    """Top-level Qt window."""

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

        header = QGridLayout()

        # ============================================================
        # LEFT LOGO AREA
        # ============================================================

        left_area = QWidget()
        left_area.setFixedWidth(180)

        left_layout = QHBoxLayout(left_area)
        left_layout.setContentsMargins(0, 0, 0, 0)

        logo_path = (
            Path(__file__).parent
            / "assets"
            / "InteractiveMedicalRobotics.png"
        )

        if logo_path.exists():
            logo_label = QLabel()

            logo_pixmap = QPixmap(
                str(logo_path)
            )

            logo_label.setPixmap(
                logo_pixmap.scaledToHeight(
                    55,
                    Qt.SmoothTransformation,
                )
            )

            left_layout.addWidget(
                logo_label,
                alignment=Qt.AlignLeft | Qt.AlignVCenter,
            )

        header.addWidget(
            left_area,
            0,
            0,
        )


        # ============================================================
        # CENTRE TITLE
        # ============================================================

        title_area = QWidget()

        title_layout = QVBoxLayout(title_area)
        title_layout.setContentsMargins(0, 0, 0, 0)

        title = QLabel("Helico Bench Test Rig")
        title.setStyleSheet(
            "font-size: 26px; "
            "font-weight: bold;"
        )
        title.setAlignment(Qt.AlignCenter)

        subtitle = QLabel(
            "Automated Balloon Characterisation Workbench"
        )
        subtitle.setStyleSheet(
            "font-size: 13px;"
            "font-weight: bold;"
            "color: #666666;"
        )
        subtitle.setAlignment(Qt.AlignCenter)

        title_layout.addWidget(title)
        title_layout.addWidget(subtitle)

        header.addWidget(
            title_area,
            0,
            1,
        )


        # ============================================================
        # RIGHT LOGO AREA
        # ============================================================

        right_area = QWidget()
        right_area.setFixedWidth(180)

        right_layout = QHBoxLayout(right_area)
        right_layout.setContentsMargins(0, 0, 0, 0)

        imr_logo_path = (
            Path(__file__).parent
            / "assets"
            / "Helico_logo_white.svg"
        )

        if imr_logo_path.exists():
            helico_logo = QSvgWidget(
                str(imr_logo_path)
            )

            helico_logo.setFixedSize(
                150,
                65,
            )

            right_layout.addWidget(
                helico_logo,
                alignment=Qt.AlignRight | Qt.AlignVCenter,
            )

        header.addWidget(
            right_area,
            0,
            2,
        )

        header.setColumnStretch(1, 1)

        outer.addLayout(header)

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
        self.status_label.setStyleSheet(
            "padding: 8px; font-weight: bold;"
        )
        outer.addWidget(self.status_label)

    def build_configure_tab(self):
        layout = QHBoxLayout(self.configure_tab)

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
        self.speed_spinbox.valueChanged.connect(
            self.sync_acceleration_to_speed
        )

        self.clearance_spinbox = QDoubleSpinBox()
        self.clearance_spinbox.setRange(0.0, 200.0)
        self.clearance_spinbox.setDecimals(2)
        self.clearance_spinbox.setSingleStep(0.1)
        self.clearance_spinbox.setValue(10.0)
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
        planning_form.addRow(
            "Speed scaling",
            self.speed_spinbox,
        )
        planning_form.addRow(
            "Minimum clearance",
            self.clearance_spinbox,
        )

        self.advanced_planning_button = QPushButton(
            "Advanced Planning ▼"
        )
        self.advanced_planning_button.setCheckable(
            True
        )
        self.advanced_planning_button.setChecked(
            False
        )
        self.advanced_planning_button.toggled.connect(
            self.toggle_advanced_planning
        )
        planning_form.addRow(
            self.advanced_planning_button
        )

        self.advanced_planning_widget = QWidget()
        advanced_layout = QFormLayout(
            self.advanced_planning_widget
        )
        advanced_layout.setContentsMargins(
            0,
            4,
            0,
            4,
        )

        self.separate_acceleration_checkbox = QCheckBox(
            "Set acceleration separately"
        )
        self.separate_acceleration_checkbox.setChecked(
            False
        )
        self.separate_acceleration_checkbox.toggled.connect(
            self.update_acceleration_control_state
        )
        advanced_layout.addRow(
            self.separate_acceleration_checkbox
        )

        acceleration_row = QWidget()
        acceleration_layout = QHBoxLayout(
            acceleration_row
        )
        acceleration_layout.setContentsMargins(
            0,
            0,
            0,
            0,
        )

        self.acceleration_slider = QSlider(
            Qt.Horizontal
        )
        self.acceleration_slider.setRange(
            1,
            100,
        )
        self.acceleration_slider.setValue(
            20
        )
        self.acceleration_slider.setEnabled(
            False
        )

        self.acceleration_value_label = QLabel(
            "20%"
        )
        self.acceleration_value_label.setMinimumWidth(
            45
        )
        self.acceleration_value_label.setAlignment(
            Qt.AlignRight
            |
            Qt.AlignVCenter
        )

        self.acceleration_slider.valueChanged.connect(
            self.update_acceleration_label
        )

        acceleration_layout.addWidget(
            self.acceleration_slider,
            1,
        )
        acceleration_layout.addWidget(
            self.acceleration_value_label
        )

        advanced_layout.addRow(
            "Acceleration scaling",
            acceleration_row,
        )

        weight_info = QLabel(
            "Scoring weights are automatically normalised to 100%."
        )
        weight_info.setWordWrap(True)
        weight_info.setStyleSheet(
            "color: #888888;"
        )
        advanced_layout.addRow(
            weight_info
        )

        (
            self.clearance_weight_slider,
            self.clearance_weight_label,
            clearance_weight_row,
        ) = self.make_weight_slider(
            40
        )

        (
            self.smoothness_weight_slider,
            self.smoothness_weight_label,
            smoothness_weight_row,
        ) = self.make_weight_slider(
            30
        )

        (
            self.path_length_weight_slider,
            self.path_length_weight_label,
            path_length_weight_row,
        ) = self.make_weight_slider(
            20
        )

        (
            self.duration_weight_slider,
            self.duration_weight_label,
            duration_weight_row,
        ) = self.make_weight_slider(
            10
        )

        advanced_layout.addRow(
            "Clearance weight",
            clearance_weight_row,
        )
        advanced_layout.addRow(
            "Smoothness weight",
            smoothness_weight_row,
        )
        advanced_layout.addRow(
            "Path length weight",
            path_length_weight_row,
        )
        advanced_layout.addRow(
            "Duration weight",
            duration_weight_row,
        )

        self.advanced_planning_widget.setVisible(
            False
        )

        planning_form.addRow(
            self.advanced_planning_widget
        )

        self.update_weight_labels()

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
        self.progress.setValue(0)
        planning_form.addRow(
            "Progress",
            self.progress,
        )

        self.planning_candidates = {}
        self.planning_total_candidates = 0
        self.current_candidate_view = 0
        self.selected_candidate_number = None
        self.selected_candidate_cost = None

        candidate_box = QGroupBox(
            "Candidate Result"
        )
        candidate_layout = QVBoxLayout(
            candidate_box
        )

        candidate_navigation = QHBoxLayout()

        self.previous_candidate_button = QPushButton(
            "◀"
        )
        self.previous_candidate_button.setFixedWidth(
            45
        )
        self.previous_candidate_button.setEnabled(
            False
        )
        self.previous_candidate_button.clicked.connect(
            self.previous_candidate
        )

        self.candidate_number_label = QLabel(
            "No planning run yet"
        )
        self.candidate_number_label.setAlignment(
            Qt.AlignCenter
        )
        self.candidate_number_label.setStyleSheet(
            "font-weight: bold;"
        )

        self.next_candidate_button = QPushButton(
            "▶"
        )
        self.next_candidate_button.setFixedWidth(
            45
        )
        self.next_candidate_button.setEnabled(
            False
        )
        self.next_candidate_button.clicked.connect(
            self.next_candidate
        )

        candidate_navigation.addWidget(
            self.previous_candidate_button
        )
        candidate_navigation.addWidget(
            self.candidate_number_label,
            1,
        )
        candidate_navigation.addWidget(
            self.next_candidate_button
        )

        candidate_layout.addLayout(
            candidate_navigation
        )

        self.planning_candidate_label = QLabel(
            "Plan a trajectory to see candidate results."
        )
        self.planning_candidate_label.setWordWrap(
            True
        )
        self.planning_candidate_label.setMinimumHeight(
            115
        )
        self.planning_candidate_label.setStyleSheet(
            "padding: 10px; "
            "border: 2px solid #666666; "
            "border-radius: 6px; "
            "color: #666666;"
        )

        candidate_layout.addWidget(
            self.planning_candidate_label
        )

        planning_form.addRow(
            candidate_box
        )

        left.addWidget(
            planning_box
        )

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

    def make_weight_slider(
        self,
        default_value,
    ):
        row = QWidget()
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(
            0,
            0,
            0,
            0,
        )

        slider = QSlider(
            Qt.Horizontal
        )
        slider.setRange(
            0,
            100,
        )
        slider.setValue(
            default_value
        )

        value_label = QLabel()
        value_label.setMinimumWidth(
            55
        )
        value_label.setAlignment(
            Qt.AlignRight
            |
            Qt.AlignVCenter
        )

        slider.valueChanged.connect(
            self.update_weight_labels
        )

        row_layout.addWidget(
            slider,
            1,
        )
        row_layout.addWidget(
            value_label
        )

        return (
            slider,
            value_label,
            row,
        )

    def update_weight_labels(self):
        if not hasattr(
            self,
            "clearance_weight_slider",
        ):
            return

        values = [
            self.clearance_weight_slider.value(),
            self.smoothness_weight_slider.value(),
            self.path_length_weight_slider.value(),
            self.duration_weight_slider.value(),
        ]

        total = sum(values)

        if total <= 0:
            normalised = [
                0.0,
                0.0,
                0.0,
                0.0,
            ]
        else:
            normalised = [
                value / total
                for value in values
            ]

        labels = [
            self.clearance_weight_label,
            self.smoothness_weight_label,
            self.path_length_weight_label,
            self.duration_weight_label,
        ]

        for label, value in zip(
            labels,
            normalised,
        ):
            label.setText(
                f"{value * 100:.0f}%"
            )

    def toggle_advanced_planning(
        self,
        checked,
    ):
        if checked:
            self._collapsed_window_size = (
                self.size()
            )

        self.advanced_planning_widget.setVisible(
            checked
        )

        self.advanced_planning_button.setText(
            "Advanced Planning ▲"
            if checked
            else
            "Advanced Planning ▼"
        )

        if (
            not checked
            and
            hasattr(
                self,
                "_collapsed_window_size",
            )
        ):
            QTimer.singleShot(
                0,
                lambda: self.resize(
                    self._collapsed_window_size
                ),
            )

    def sync_acceleration_to_speed(
        self,
        value,
    ):
        if (
            hasattr(
                self,
                "separate_acceleration_checkbox",
            )
            and
            not self.separate_acceleration_checkbox.isChecked()
        ):
            self.acceleration_slider.setValue(
                value
            )

    def update_acceleration_control_state(
        self,
        checked,
    ):
        self.acceleration_slider.setEnabled(
            checked
        )

        if not checked:
            self.acceleration_slider.setValue(
                self.speed_spinbox.value()
            )

    def update_acceleration_label(
        self,
        value,
    ):
        self.acceleration_value_label.setText(
            f"{value}%"
        )

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
                "Build and run a validated sequence of saved trajectories."
            )
        )

        cycle_box = QGroupBox(
            "Experiment Cycle"
        )
        cycle_layout = QVBoxLayout(
            cycle_box
        )

        self.cycle_trajectory_boxes = []
        self.cycle_step_rows = []

        self.cycle_steps_layout = QVBoxLayout()
        cycle_layout.addLayout(
            self.cycle_steps_layout
        )

        for _ in range(4):
            self.add_cycle_step(
                invalidate=False
            )

        self.add_cycle_step_button = QPushButton(
            "+ ADD STEP"
        )
        self.add_cycle_step_button.clicked.connect(
            self.add_cycle_step
        )
        cycle_layout.addWidget(
            self.add_cycle_step_button
        )

        self.validate_cycle_button = QPushButton(
            "VALIDATE CYCLE"
        )
        self.validate_cycle_button.clicked.connect(
            self.validate_cycle
        )
        cycle_layout.addWidget(
            self.validate_cycle_button
        )

        self.pause_each_step_checkbox = QCheckBox(
            "Pause after each step"
        )
        self.pause_each_step_checkbox.setChecked(
            True
        )
        cycle_layout.addWidget(
            self.pause_each_step_checkbox
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

        controls = QHBoxLayout()

        self.run_cycle_button = QPushButton(
            "RUN CYCLE"
        )
        self.run_cycle_button.clicked.connect(
            self.run_cycle
        )
        self.run_cycle_button.setEnabled(
            False
        )

        self.continue_cycle_button = QPushButton(
            "CONTINUE"
        )
        self.continue_cycle_button.clicked.connect(
            self.continue_cycle
        )
        self.continue_cycle_button.setEnabled(
            False
        )

        self.stop_cycle_button = QPushButton(
            "STOP CYCLE"
        )
        self.stop_cycle_button.clicked.connect(
            self.stop_cycle
        )
        self.stop_cycle_button.setEnabled(
            False
        )

        controls.addWidget(
            self.run_cycle_button
        )
        controls.addWidget(
            self.continue_cycle_button
        )
        controls.addWidget(
            self.stop_cycle_button
        )

        cycle_layout.addLayout(
            controls
        )
        layout.addWidget(
            cycle_box
        )

        logging_box = QGroupBox(
            "Experiment Data Logging"
        )
        logging_layout = QVBoxLayout(
            logging_box
        )

        self.record_cycle_checkbox = QCheckBox(
            "Record data during cycle"
        )
        self.record_cycle_checkbox.setChecked(
            True
        )
        logging_layout.addWidget(
            self.record_cycle_checkbox
        )

        self.logging_status_label = QLabel(
            "Logging: OFF"
        )
        self.logging_status_label.setStyleSheet(
            "font-weight: bold;"
        )
        logging_layout.addWidget(
            self.logging_status_label
        )

        self.logging_file_label = QLabel(
            "No log file active."
        )
        self.logging_file_label.setWordWrap(
            True
        )
        logging_layout.addWidget(
            self.logging_file_label
        )

        logging_buttons = QHBoxLayout()

        self.start_logging_button = QPushButton(
            "START LOGGING"
        )
        self.start_logging_button.clicked.connect(
            self.start_logging
        )

        self.stop_logging_button = QPushButton(
            "STOP LOGGING"
        )
        self.stop_logging_button.clicked.connect(
            self.stop_logging
        )
        self.stop_logging_button.setEnabled(
            False
        )

        logging_buttons.addWidget(
            self.start_logging_button
        )
        logging_buttons.addWidget(
            self.stop_logging_button
        )

        logging_layout.addLayout(
            logging_buttons
        )
        layout.addWidget(
            logging_box
        )
        layout.addStretch()

    # ============================================================
    # EXPERIMENT CYCLE STEP EDITOR
    # ============================================================

    def available_trajectory_names(self):
        return [
            self.trajectory_list.item(
                index
            ).text()
            for index in range(
                self.trajectory_list.count()
            )
        ]

    def add_cycle_step(
        self,
        checked=False,
        invalidate=True,
    ):
        if self.cycle_running:
            return

        row_widget = QWidget()

        row_layout = QHBoxLayout(
            row_widget
        )
        row_layout.setContentsMargins(
            0,
            0,
            0,
            0,
        )

        step_label = QLabel()

        trajectory_box = QComboBox()
        trajectory_box.addItem(
            "-- select trajectory --"
        )
        trajectory_box.addItems(
            self.available_trajectory_names()
        )
        trajectory_box.currentTextChanged.connect(
            self.invalidate_cycle_validation
        )

        remove_button = QPushButton(
            "REMOVE"
        )
        remove_button.clicked.connect(
            lambda checked=False,
            widget=row_widget:
            self.remove_cycle_step(
                widget
            )
        )

        row_layout.addWidget(
            step_label
        )
        row_layout.addWidget(
            trajectory_box,
            1,
        )
        row_layout.addWidget(
            remove_button
        )

        row_data = {
            "widget": row_widget,
            "label": step_label,
            "box": trajectory_box,
            "remove": remove_button,
        }

        self.cycle_step_rows.append(
            row_data
        )
        self.cycle_trajectory_boxes.append(
            trajectory_box
        )
        self.cycle_steps_layout.addWidget(
            row_widget
        )

        self.update_cycle_step_rows()

        if (
            invalidate
            and
            hasattr(
                self,
                "run_cycle_button",
            )
        ):
            self.invalidate_cycle_validation()

    def remove_cycle_step(
        self,
        row_widget,
    ):
        if self.cycle_running:
            return

        if len(
            self.cycle_step_rows
        ) <= 2:
            self.set_status(
                "An experiment cycle requires "
                "at least two steps."
            )
            return

        for index, row in enumerate(
            self.cycle_step_rows
        ):
            if (
                row["widget"]
                is row_widget
            ):
                self.cycle_step_rows.pop(
                    index
                )
                self.cycle_trajectory_boxes.pop(
                    index
                )

                row_widget.setParent(
                    None
                )
                row_widget.deleteLater()

                break

        self.update_cycle_step_rows()
        self.invalidate_cycle_validation()

    def update_cycle_step_rows(self):
        can_remove = (
            len(
                self.cycle_step_rows
            )
            >
            2
        )

        for index, row in enumerate(
            self.cycle_step_rows
        ):
            row["label"].setText(
                f"Step {index + 1}"
            )

            row["remove"].setEnabled(
                can_remove
                and
                not self.cycle_running
            )

    def sensor_is_recent(
        self,
        timestamp,
        timeout=1.0,
    ):
        if timestamp is None:
            return False

        return (
            time.monotonic()
            - timestamp
            <= timeout
        )

    def update_sensor_display(self):
        laser_recent = self.sensor_is_recent(
            self.node.last_laser_update
        )
        force_recent = self.sensor_is_recent(
            self.node.last_force_update
        )
        pressure_recent = self.sensor_is_recent(
            self.node.last_pressure_update
        )

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
        if self.cycle_state in [
            "READY",
            "COMPLETE",
        ]:
            cycle_colour = GREEN

        elif self.cycle_state in [
            "PREPARING",
            "RUNNING",
            "PAUSED",
            "STOP_REQUESTED",
        ]:
            cycle_colour = ORANGE

        elif self.cycle_state in [
            "FAILED",
            "STOPPED",
        ]:
            cycle_colour = RED

        else:
            cycle_colour = NEUTRAL

        self.cycle_status_label.setStyleSheet(
            f"font-weight: bold; "
            f"color: {cycle_colour};"
        )

        logging_colour = (
            GREEN
            if self.experiment_logger.active
            else NEUTRAL
        )

        self.logging_status_label.setStyleSheet(
            f"font-weight: bold; "
            f"color: {logging_colour};"
        )

    def spin_ros(self):
        if rclpy.ok():
            rclpy.spin_once(
                self.node,
                timeout_sec=0.0,
            )

    def wait_for_backend(self):
        if not rclpy.ok():
            return

        ready = (
            self.node.list_poses_client.service_is_ready()
            and
            self.node.list_trajectories_client.service_is_ready()
        )

        self.diagnostics_tab.set_backend_connected(
            ready
        )

        if (
            ready
            and
            not self.backend_was_ready
        ):
            self.set_status(
                "Backend connected."
            )
            self.refresh_all()

        elif (
            not ready
            and
            self.backend_was_ready
        ):
            self.set_status(
                "Workbench backend disconnected."
            )

        elif not ready:
            self.set_status(
                "Waiting for workbench backend..."
            )

        self.backend_was_ready = ready

    def set_status(
        self,
        text,
    ):
        self.status_label.setText(
            text
        )

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

        if any(
            word in status
            for word in red_words
        ):
            colour = RED

        elif any(
            word in status
            for word in green_words
        ):
            colour = GREEN

        elif any(
            word in status
            for word in orange_words
        ):
            colour = ORANGE

        else:
            colour = NEUTRAL

        self.status_label.setStyleSheet(
            f"padding: 8px; "
            f"font-weight: bold; "
            f"color: {colour};"
        )

    def closeEvent(
        self,
        event,
    ):
        if self.experiment_logger.active:
            self.stop_logging()

        event.accept()
