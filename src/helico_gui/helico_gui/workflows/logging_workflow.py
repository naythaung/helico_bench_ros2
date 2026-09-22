class LoggingWorkflowMixin:
    """GUI-facing experiment logging behaviour."""

    @property
    def logging_active(self):
        return self.experiment_logger.active

    def start_logging(self):
        success, result = self.experiment_logger.start()

        if not success:
            self.set_status(f"Could not start logging: {result}")
            return False

        self.logging_timer.start()
        self.start_logging_button.setEnabled(False)
        self.stop_logging_button.setEnabled(True)
        self.logging_status_label.setText("Logging: ON")
        self.logging_file_label.setText(f"File: {result}")
        self.set_status("Experiment logging started.")
        return True

    def stop_logging(self):
        if not self.experiment_logger.active:
            return

        self.logging_timer.stop()
        path = self.experiment_logger.stop()

        self.start_logging_button.setEnabled(True)
        self.stop_logging_button.setEnabled(False)
        self.logging_status_label.setText("Logging: OFF")

        if path is not None:
            self.logging_file_label.setText(f"Saved: {path}")

        self.set_status("Experiment logging stopped.")

    def current_cycle_context(self):
        if (
            self.validated_cycle_names
            and self.cycle_current_index < len(self.validated_cycle_names)
        ):
            return (
                self.cycle_current_index + 1,
                self.validated_cycle_names[self.cycle_current_index],
            )

        if self.validated_cycle_names and self.cycle_state == "COMPLETE":
            return (
                len(self.validated_cycle_names),
                self.validated_cycle_names[-1],
            )

        return 0, ""

    def write_log_sample(self):
        if not self.experiment_logger.active:
            return

        laser_recent = self.sensor_is_recent(self.node.last_laser_update)
        force_recent = self.sensor_is_recent(self.node.last_force_update)
        pressure_recent = self.sensor_is_recent(self.node.last_pressure_update)

        laser = self.node.latest_laser if laser_recent else ""
        force = self.node.latest_force if force_recent else ""
        pressure = self.node.latest_pressure if pressure_recent else ""

        step_index, trajectory_name = self.current_cycle_context()

        success, error = self.experiment_logger.write_sample(
            laser=laser,
            force=force,
            pressure=pressure,
            cycle_state=self.cycle_state,
            step_index=step_index,
            trajectory_name=trajectory_name,
        )

        if not success:
            self.set_status(f"Logging error: {error}")
            self.stop_logging()
