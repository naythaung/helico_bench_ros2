import time

from PySide6.QtWidgets import (
    QWidget,
    QHBoxLayout,
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


class HardwareStatusWidget(QWidget):

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

        self.sensor_status_label = QLabel()

        self.laser_label.setStyleSheet(
            "font-size: 16px; "
            "font-weight: bold;"
        )

        self.force_label.setStyleSheet(
            "font-size: 16px; "
            "font-weight: bold;"
        )

        set_status(
            self.sensor_status_label,
            "WAITING",
            ORANGE,
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

        self.helico_status_label = QLabel()

        self.pressure_label.setStyleSheet(
            "font-size: 16px; "
            "font-weight: bold;"
        )

        set_status(
            self.helico_status_label,
            "WAITING",
            ORANGE,
        )

        helico_layout.addWidget(
            self.pressure_label
        )

        helico_layout.addStretch()

        helico_layout.addWidget(
            self.helico_status_label
        )

        # ========================================================
        # ADD GROUPS
        # ========================================================

        layout.addWidget(
            sensor_box,
            2,
        )

        layout.addWidget(
            helico_box,
            1,
        )

    # ============================================================
    # UPDATE VALUES
    # ============================================================

    def update_values(
        self,
        laser,
        force,
        pressure,
        laser_recent,
        force_recent,
        pressure_recent,
    ):

        now = time.monotonic()

        # ========================================================
        # SENSOR VALUES
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

        if pressure_recent:

            self.pressure_label.setText(
                f"Pressure: {pressure:.2f}"
            )

        else:

            self.pressure_label.setText(
                "Pressure: --"
            )

        # ========================================================
        # BENCH SENSOR STATUS
        # ========================================================

        sensor_streaming = (
            laser_recent
            and force_recent
        )

        if sensor_streaming:

            self.sensor_was_streaming = True
            self.sensor_restart_since = None

            set_status(
                self.sensor_status_label,
                "STREAMING",
                GREEN,
            )

        elif laser_recent or force_recent:

            self.sensor_was_streaming = True
            self.sensor_restart_since = None

            set_status(
                self.sensor_status_label,
                "PARTIAL",
                ORANGE,
            )

        elif self.sensor_was_streaming:

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
                    self.sensor_status_label,
                    "RESTARTING",
                    ORANGE,
                )

            else:

                set_status(
                    self.sensor_status_label,
                    "DISCONNECTED",
                    RED,
                )

        else:

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
                    self.sensor_status_label,
                    "WAITING",
                    ORANGE,
                )

            else:

                set_status(
                    self.sensor_status_label,
                    "DISCONNECTED",
                    RED,
                )

        # ========================================================
        # HELICO STATUS
        # ========================================================

        if pressure_recent:

            self.helico_was_streaming = True
            self.helico_restart_since = None

            set_status(
                self.helico_status_label,
                "STREAMING",
                GREEN,
            )

        elif self.helico_was_streaming:

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
                    self.helico_status_label,
                    "RESTARTING",
                    ORANGE,
                )

            else:

                set_status(
                    self.helico_status_label,
                    "DISCONNECTED",
                    RED,
                )

        else:

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
                    self.helico_status_label,
                    "WAITING",
                    ORANGE,
                )

            else:

                set_status(
                    self.helico_status_label,
                    "DISCONNECTED",
                    RED,
                )