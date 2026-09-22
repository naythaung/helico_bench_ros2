class CycleController:

    IDLE = "IDLE"
    READY = "READY"
    PREPARING = "PREPARING"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    STOP_REQUESTED = "STOP_REQUESTED"
    COMPLETE = "COMPLETE"
    STOPPED = "STOPPED"
    FAILED = "FAILED"

    def __init__(self):

        self.validated_names = []

        self.pending_names = []
        self.pending_details = {}

        self.current_index = 0

        self.running = False
        self.paused = False
        self.stop_requested = False

        self.state = self.IDLE

    def reset_validation(self):

        if self.running:
            return False

        self.validated_names = []

        self.pending_names = []
        self.pending_details = {}

        self.current_index = 0

        self.state = self.IDLE

        return True

    def begin_validation(
        self,
        names,
    ):

        self.validated_names = []

        self.pending_names = list(
            names
        )

        self.pending_details = {}

    def add_inspection(
        self,
        index,
        name,
        start_pose,
        target_pose,
    ):

        self.pending_details[
            index
        ] = {
            "name": name,
            "start_pose": start_pose,
            "target_pose": target_pose,
        }

    def validation_complete(self):

        return (
            len(self.pending_details)
            ==
            len(self.pending_names)
        )

    def ordered_details(self):

        return [
            self.pending_details[index]
            for index
            in range(
                len(self.pending_names)
            )
        ]

    def set_validated(
        self,
        names,
    ):

        self.validated_names = list(
            names
        )

        self.state = self.READY

    def begin_run(self):

        self.running = True
        self.paused = False
        self.stop_requested = False

        self.current_index = 0

        self.state = self.PREPARING

    def begin_step(self):

        self.paused = False
        self.state = self.RUNNING

    def complete_step(self):

        self.current_index += 1

    def has_finished(self):

        return (
            self.current_index
            >=
            len(self.validated_names)
        )

    def current_name(self):

        if (
            self.current_index
            <
            len(self.validated_names)
        ):

            return self.validated_names[
                self.current_index
            ]

        return ""

    def total_steps(self):

        return len(
            self.validated_names
        )

    def request_stop(self):

        self.stop_requested = True
        self.state = self.STOP_REQUESTED

    def set_paused(self):

        self.paused = True
        self.state = self.PAUSED

    def resume(self):

        self.paused = False
        self.state = self.RUNNING

    def complete(self):

        self.running = False
        self.paused = False
        self.stop_requested = False

        self.state = self.COMPLETE

    def stop(self):

        self.running = False
        self.paused = False
        self.stop_requested = False

        self.state = self.STOPPED

    def fail(self):

        self.running = False
        self.paused = False
        self.stop_requested = False

        self.state = self.FAILED