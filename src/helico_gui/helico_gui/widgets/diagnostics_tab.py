from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QGroupBox,
    QLabel,
)


class DiagnosticsTab(QWidget):

    def __init__(self, parent=None):

        super().__init__(parent)

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

        self.backend_status = QLabel(
            "Workbench Backend: CHECKING"
        )

        workbench_layout.addWidget(
            self.backend_status
        )

        layout.addWidget(
            workbench_box
        )

        # ========================================================
        # BENCH SENSOR CONTROLLER
        # ========================================================

        sensor_box = QGroupBox(
            "Bench Sensors"
        )

        sensor_layout = QVBoxLayout(
            sensor_box
        )

        self.sensor_controller_status = QLabel(
            "Bench Sensor Controller: WAITING"
        )

        self.laser_status = QLabel(
            "    Laser: WAITING"
        )

        self.force_status = QLabel(
            "    Force: WAITING"
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
        # HELICO CONTROLLER
        # ========================================================

        helico_box = QGroupBox(
            "Helico"
        )

        helico_layout = QVBoxLayout(
            helico_box
        )

        self.helico_controller_status = QLabel(
            "Helico Controller: WAITING"
        )

        self.pressure_status = QLabel(
            "    Pressure: WAITING"
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

    def set_backend_connected(
        self,
        connected,
    ):

        if connected:

            self.backend_status.setText(
                "Workbench Backend: CONNECTED"
            )

        else:

            self.backend_status.setText(
                "Workbench Backend: WAITING"
            )

    def update_hardware(
        self,
        laser_recent,
        force_recent,
        pressure_recent,
    ):

        # ========================================================
        # BENCH SENSOR CONTROLLER
        # ========================================================

        if laser_recent and force_recent:

            self.sensor_controller_status.setText(
                "Bench Sensor Controller: CONNECTED"
            )

        elif laser_recent or force_recent:

            self.sensor_controller_status.setText(
                "Bench Sensor Controller: PARTIAL DATA"
            )

        else:

            self.sensor_controller_status.setText(
                "Bench Sensor Controller: DISCONNECTED"
            )

        self.laser_status.setText(
            "    Laser: STREAMING"
            if laser_recent
            else "    Laser: NO DATA"
        )

        self.force_status.setText(
            "    Force: STREAMING"
            if force_recent
            else "    Force: NO DATA"
        )

        # ========================================================
        # HELICO CONTROLLER
        # ========================================================

        if pressure_recent:

            self.helico_controller_status.setText(
                "Helico Controller: CONNECTED"
            )

        else:

            self.helico_controller_status.setText(
                "Helico Controller: DISCONNECTED"
            )

        self.pressure_status.setText(
            "    Pressure: STREAMING"
            if pressure_recent
            else "    Pressure: NO DATA"
        )
