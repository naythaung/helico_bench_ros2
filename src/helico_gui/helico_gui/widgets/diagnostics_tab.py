import time

from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QGroupBox,
    QLabel,
)


GREEN = "#2e7d32"
ORANGE = "#ed8b00"
RED = "#c62828"


def set_status(label, text, colour):

    label.setText(text)

    label.setStyleSheet(
        f"font-weight: bold; color: {colour};"
    )


class DiagnosticsTab(QWidget):

    def __init__(self, parent=None):

        super().__init__(parent)

        # ========================================================
        # CONNECTION STATE
        # ========================================================

        self.start_time = time.monotonic()

        self.sensor_was_streaming = False
        self.sensor_restart_since = None

        self.helico_was_streaming = False
        self.helico_restart_since = None

        self.restart_grace_period = 5.0

        # ========================================================
        # MAIN LAYOUT
        # ========================================================

        layout = QVBoxLayout(self)

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

        # ========================================================
        # WORKBENCH
        # ========================================================

        workbench_box = QGroupBox(
            "Workbench"
        )

        workbench_layout = QVBoxLayout(
            workbench_box
        )

        self.backend_status = QLabel()

        set_status(
            self.backend_status,
            "Workbench Backend: CHECKING",
            ORANGE,
        )

        workbench_layout.addWidget(
            self.backend_status
        )

        layout.addWidget(
            workbench_box
        )

        # ========================================================
        # BENCH SENSORS
        # ========================================================

        sensor_box = QGroupBox(
            "Bench Sensors"
        )

        sensor_layout = QVBoxLayout(
            sensor_box
        )

        self.sensor_controller_status = QLabel()
        self.laser_status = QLabel()
        self.force_status = QLabel()

        set_status(
            self.sensor_controller_status,
            "Bench Sensor Controller: WAITING",
            ORANGE,
        )

        set_status(
            self.laser_status,
            "    Laser: WAITING",
            ORANGE,
        )

        set_status(
            self.force_status,
            "    Force: WAITING",
            ORANGE,
        )

        sensor_layout.addWidget(
            self.sensor_controller_status
        )

        sensor_layout.addWidget(
            self.laser_status
        )

        sensor_layout.addWidget(
            self.force_status
        )

        layout.addWidget(
            sensor_box
        )

        # ========================================================
        # HELICO
        # ========================================================

        helico_box = QGroupBox(
            "Helico"
        )

        helico_layout = QVBoxLayout(
            helico_box
        )

        self.helico_controller_status = QLabel()
        self.pressure_status = QLabel()

        set_status(
            self.helico_controller_status,
            "Helico Controller: WAITING",
            ORANGE,
        )

        set_status(
            self.pressure_status,
            "    Pressure: WAITING",
            ORANGE,
        )

        helico_layout.addWidget(
            self.helico_controller_status
        )

        helico_layout.addWidget(
            self.pressure_status
        )

        layout.addWidget(
            helico_box
        )

        layout.addStretch()

    # ============================================================
    # WORKBENCH STATUS
    # ============================================================

    def set_backend_connected(
        self,
        connected,
    ):

        if connected:

            set_status(
                self.backend_status,
                "Workbench Backend: CONNECTED",
                GREEN,
            )

        else:

            set_status(
                self.backend_status,
                "Workbench Backend: WAITING",
                ORANGE,
            )

    # ============================================================
    # HARDWARE STATUS
    # ============================================================

    def update_hardware(
        self,
        laser_recent,
        force_recent,
        pressure_recent,
    ):

        now = time.monotonic()

        self.update_bench_sensors(
            now,
            laser_recent,
            force_recent,
        )

        self.update_helico(
            now,
            pressure_recent,
        )

    # ============================================================
    # BENCH SENSOR CONTROLLER
    # ============================================================

    def update_bench_sensors(
        self,
        now,
        laser_recent,
        force_recent,
    ):

        sensor_streaming = (
            laser_recent
            and force_recent
        )

        # --------------------------------------------------------
        # BOTH STREAMING
        # --------------------------------------------------------

        if sensor_streaming:

            self.sensor_was_streaming = True
            self.sensor_restart_since = None

            set_status(
                self.sensor_controller_status,
                "Bench Sensor Controller: CONNECTED",
                GREEN,
            )

            set_status(
                self.laser_status,
                "    Laser: STREAMING",
                GREEN,
            )

            set_status(
                self.force_status,
                "    Force: STREAMING",
                GREEN,
            )

            return

        # --------------------------------------------------------
        # PARTIAL DATA
        # --------------------------------------------------------

        if laser_recent or force_recent:

            self.sensor_was_streaming = True
            self.sensor_restart_since = None

            set_status(
                self.sensor_controller_status,
                "Bench Sensor Controller: PARTIAL DATA",
                ORANGE,
            )

            if laser_recent:

                set_status(
                    self.laser_status,
                    "    Laser: STREAMING",
                    GREEN,
                )

            else:

                set_status(
                    self.laser_status,
                    "    Laser: RESTARTING",
                    ORANGE,
                )

            if force_recent:

                set_status(
                    self.force_status,
                    "    Force: STREAMING",
                    GREEN,
                )

            else:

                set_status(
                    self.force_status,
                    "    Force: RESTARTING",
                    ORANGE,
                )

            return

        # --------------------------------------------------------
        # PREVIOUSLY STREAMING, NOW LOST
        # --------------------------------------------------------

        if self.sensor_was_streaming:

            if self.sensor_restart_since is None:

                self.sensor_restart_since = now

            restart_age = (
                now
                -
                self.sensor_restart_since
            )

            if (
                restart_age
                <
                self.restart_grace_period
            ):

                set_status(
                    self.sensor_controller_status,
                    "Bench Sensor Controller: RESTARTING",
                    ORANGE,
                )

                set_status(
                    self.laser_status,
                    "    Laser: RESTARTING",
                    ORANGE,
                )

                set_status(
                    self.force_status,
                    "    Force: RESTARTING",
                    ORANGE,
                )

            else:

                set_status(
                    self.sensor_controller_status,
                    "Bench Sensor Controller: DISCONNECTED",
                    RED,
                )

                set_status(
                    self.laser_status,
                    "    Laser: NO DATA",
                    RED,
                )

                set_status(
                    self.force_status,
                    "    Force: NO DATA",
                    RED,
                )

            return

        # --------------------------------------------------------
        # INITIAL STARTUP
        # --------------------------------------------------------

        startup_age = (
            now
            -
            self.start_time
        )

        if (
            startup_age
            <
            self.restart_grace_period
        ):

            set_status(
                self.sensor_controller_status,
                "Bench Sensor Controller: WAITING",
                ORANGE,
            )

            set_status(
                self.laser_status,
                "    Laser: WAITING",
                ORANGE,
            )

            set_status(
                self.force_status,
                "    Force: WAITING",
                ORANGE,
            )

        else:

            set_status(
                self.sensor_controller_status,
                "Bench Sensor Controller: DISCONNECTED",
                RED,
            )

            set_status(
                self.laser_status,
                "    Laser: NO DATA",
                RED,
            )

            set_status(
                self.force_status,
                "    Force: NO DATA",
                RED,
            )

    # ============================================================
    # HELICO CONTROLLER
    # ============================================================

    def update_helico(
        self,
        now,
        pressure_recent,
    ):

        # --------------------------------------------------------
        # STREAMING
        # --------------------------------------------------------

        if pressure_recent:

            self.helico_was_streaming = True
            self.helico_restart_since = None

            set_status(
                self.helico_controller_status,
                "Helico Controller: CONNECTED",
                GREEN,
            )

            set_status(
                self.pressure_status,
                "    Pressure: STREAMING",
                GREEN,
            )

            return

        # --------------------------------------------------------
        # PREVIOUSLY STREAMING, NOW LOST
        # --------------------------------------------------------

        if self.helico_was_streaming:

            if self.helico_restart_since is None:

                self.helico_restart_since = now

            restart_age = (
                now
                -
                self.helico_restart_since
            )

            if (
                restart_age
                <
                self.restart_grace_period
            ):

                set_status(
                    self.helico_controller_status,
                    "Helico Controller: RESTARTING",
                    ORANGE,
                )

                set_status(
                    self.pressure_status,
                    "    Pressure: RESTARTING",
                    ORANGE,
                )

            else:

                set_status(
                    self.helico_controller_status,
                    "Helico Controller: DISCONNECTED",
                    RED,
                )

                set_status(
                    self.pressure_status,
                    "    Pressure: NO DATA",
                    RED,
                )

            return

        # --------------------------------------------------------
        # INITIAL STARTUP
        # --------------------------------------------------------

        startup_age = (
            now
            -
            self.start_time
        )

        if (
            startup_age
            <
            self.restart_grace_period
        ):

            set_status(
                self.helico_controller_status,
                "Helico Controller: WAITING",
                ORANGE,
            )

            set_status(
                self.pressure_status,
                "    Pressure: WAITING",
                ORANGE,
            )

        else:

            set_status(
                self.helico_controller_status,
                "Helico Controller: DISCONNECTED",
                RED,
            )

            set_status(
                self.pressure_status,
                "    Pressure: NO DATA",
                RED,
            )