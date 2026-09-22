from PySide6.QtWidgets import (
    QWidget,
    QHBoxLayout,
    QGroupBox,
    QLabel,
)


class HardwareStatusWidget(QWidget):

    def __init__(self, parent=None):

        super().__init__(parent)

        layout = QHBoxLayout(self)

        layout.setContentsMargins(
            0,
            0,
            0,
            0,
        )

        # ========================================================
        # BENCH SENSORS
        # ========================================================

        sensor_box = QGroupBox(
            "Bench Sensors"
        )

        sensor_layout = QHBoxLayout(
            sensor_box
        )

        self.laser_label = QLabel(
            "Laser: --"
        )

        self.force_label = QLabel(
            "Force: --"
        )

        self.sensor_status_label = QLabel(
            "WAITING"
        )

        self.laser_label.setStyleSheet(
            "font-size: 16px; "
            "font-weight: bold;"
        )

        self.force_label.setStyleSheet(
            "font-size: 16px; "
            "font-weight: bold;"
        )

        self.sensor_status_label.setStyleSheet(
            "font-weight: bold;"
        )

        sensor_layout.addWidget(
            self.laser_label
        )

        sensor_layout.addWidget(
            self.force_label
        )

        sensor_layout.addStretch()

        sensor_layout.addWidget(
            self.sensor_status_label
        )

        # ========================================================
        # HELICO
        # ========================================================

        helico_box = QGroupBox(
            "Helico"
        )

        helico_layout = QHBoxLayout(
            helico_box
        )

        self.pressure_label = QLabel(
            "Pressure: --"
        )

        self.helico_status_label = QLabel(
            "WAITING"
        )

        self.pressure_label.setStyleSheet(
            "font-size: 16px; "
            "font-weight: bold;"
        )

        self.helico_status_label.setStyleSheet(
            "font-weight: bold;"
        )

        helico_layout.addWidget(
            self.pressure_label
        )

        helico_layout.addStretch()

        helico_layout.addWidget(
            self.helico_status_label
        )

        # ========================================================
        # LAYOUT
        # ========================================================

        layout.addWidget(
            sensor_box,
            2,
        )

        layout.addWidget(
            helico_box,
            1,
        )

    def update_values(
        self,
        laser,
        force,
        pressure,
        laser_recent,
        force_recent,
        pressure_recent,
    ):

        # ========================================================
        # BENCH SENSOR VALUES
        # ========================================================

        if laser_recent:

            self.laser_label.setText(
                f"Laser: {laser:.2f}"
            )

        else:

            self.laser_label.setText(
                "Laser: --"
            )

        if force_recent:

            self.force_label.setText(
                f"Force: {force:.2f}"
            )

        else:

            self.force_label.setText(
                "Force: --"
            )

        # ========================================================
        # HELICO PRESSURE
        # ========================================================

        if pressure_recent:

            self.pressure_label.setText(
                f"Pressure: {pressure:.2f}"
            )

        else:

            self.pressure_label.setText(
                "Pressure: --"
            )

        # ========================================================
        # CONTROLLER STATUS
        # ========================================================

        if laser_recent and force_recent:

            self.sensor_status_label.setText(
                "STREAMING"
            )

        elif laser_recent or force_recent:

            self.sensor_status_label.setText(
                "PARTIAL"
            )

        else:

            self.sensor_status_label.setText(
                "WAITING"
            )

        if pressure_recent:

            self.helico_status_label.setText(
                "STREAMING"
            )

        else:

            self.helico_status_label.setText(
                "WAITING"
            )
