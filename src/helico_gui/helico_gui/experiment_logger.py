import csv

from pathlib import Path
from datetime import datetime


class ExperimentLogger:

    def __init__(self):

        self.active = False
        self.file = None
        self.writer = None
        self.path = None

    def start(self):

        if self.active:
            return False, "Logging is already active."

        log_directory = (
            Path.cwd()
            /
            "experiment_logs"
        )

        log_directory.mkdir(
            parents=True,
            exist_ok=True,
        )

        timestamp = datetime.now().strftime(
            "%Y-%m-%d_%H-%M-%S"
        )

        self.path = (
            log_directory
            /
            f"helico_experiment_{timestamp}.csv"
        )

        try:

            self.file = open(
                self.path,
                "w",
                newline="",
            )

            self.writer = csv.writer(
                self.file
            )

            self.writer.writerow(
                [
                    "timestamp",
                    "laser",
                    "force",
                    "pressure",
                    "cycle_state",
                    "step_index",
                    "trajectory_name",
                ]
            )

            self.file.flush()

            self.active = True

            return True, str(self.path)

        except Exception as error:

            self.active = False
            self.file = None
            self.writer = None
            self.path = None

            return False, str(error)

    def stop(self):

        if not self.active:
            return self.path

        self.active = False

        try:

            if self.file is not None:

                self.file.flush()
                self.file.close()

        except Exception:

            pass

        self.file = None
        self.writer = None

        return self.path

    def write_sample(
        self,
        laser,
        force,
        pressure,
        cycle_state,
        step_index,
        trajectory_name,
    ):

        if not self.active:
            return False, ""

        if self.writer is None:
            return False, "Log writer unavailable."

        timestamp = (
            datetime.now()
            .astimezone()
            .isoformat(
                timespec="milliseconds"
            )
        )

        try:

            self.writer.writerow(
                [
                    timestamp,
                    laser,
                    force,
                    pressure,
                    cycle_state,
                    step_index,
                    trajectory_name,
                ]
            )

            self.file.flush()

            return True, ""

        except Exception as error:

            return False, str(error)