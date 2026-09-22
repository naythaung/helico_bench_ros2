from PySide6.QtWidgets import QMessageBox

from meca500_interfaces.srv import InspectTrajectory
from meca500_interfaces.action import GoToTrajectoryStart, ExecuteTrajectory


class CycleWorkflowMixin:
    """Experiment-cycle validation and execution.

    The mixin owns workflow behaviour while CycleController stores the state.
    The property aliases below keep the workflow code readable and make the
    state model explicit in one place.
    """

    @property
    def validated_cycle_names(self):
        return self.cycle.validated_names

    @validated_cycle_names.setter
    def validated_cycle_names(self, value):
        self.cycle.validated_names = value

    @property
    def pending_cycle_names(self):
        return self.cycle.pending_names

    @pending_cycle_names.setter
    def pending_cycle_names(self, value):
        self.cycle.pending_names = value

    @property
    def pending_cycle_details(self):
        return self.cycle.pending_details

    @pending_cycle_details.setter
    def pending_cycle_details(self, value):
        self.cycle.pending_details = value

    @property
    def cycle_current_index(self):
        return self.cycle.current_index

    @cycle_current_index.setter
    def cycle_current_index(self, value):
        self.cycle.current_index = value

    @property
    def cycle_running(self):
        return self.cycle.running

    @cycle_running.setter
    def cycle_running(self, value):
        self.cycle.running = value

    @property
    def cycle_paused(self):
        return self.cycle.paused

    @cycle_paused.setter
    def cycle_paused(self, value):
        self.cycle.paused = value

    @property
    def stop_cycle_requested(self):
        return self.cycle.stop_requested

    @stop_cycle_requested.setter
    def stop_cycle_requested(self, value):
        self.cycle.stop_requested = value

    @property
    def cycle_state(self):
        return self.cycle.state

    @cycle_state.setter
    def cycle_state(self, value):
        self.cycle.state = value


    def invalidate_cycle_validation(self, *_):
        if self.cycle_running:
            return
        self.validated_cycle_names = []
        self.run_cycle_button.setEnabled(False)
        self.continue_cycle_button.setEnabled(False)
        self.stop_cycle_button.setEnabled(False)
        self.cycle_progress.setValue(0)
        self.cycle_state = 'IDLE'
        self.cycle_status_label.setText('Cycle not validated.')

    def validate_cycle(self):
        if self.cycle_running:
            return
        trajectory_names = [box.currentText().strip() for box in self.cycle_trajectory_boxes]
        trajectory_names = [name for name in trajectory_names if name and name != '-- select trajectory --']
        if len(trajectory_names) < 2:
            self.validated_cycle_names = []
            self.cycle_status_label.setText('INVALID: select at least two trajectories.')
            self.run_cycle_button.setEnabled(False)
            return
        if not self.node.inspect_trajectory_client.service_is_ready():
            self.validated_cycle_names = []
            self.cycle_status_label.setText('Cannot validate: trajectory inspection service is unavailable.')
            self.run_cycle_button.setEnabled(False)
            return
        self.cycle_status_label.setText('Validating cycle...')
        self.run_cycle_button.setEnabled(False)
        self.validated_cycle_names = []
        self.pending_cycle_names = trajectory_names
        self.pending_cycle_details = {}
        for index, name in enumerate(trajectory_names):
            request = InspectTrajectory.Request()
            request.name = name
            future = self.node.inspect_trajectory_client.call_async(request)
            future.add_done_callback(lambda future, index=index, name=name: self.cycle_inspection_received(future, index, name))

    def cycle_inspection_received(self, future, index, name):
        try:
            response = future.result()
            if not response.success:
                self.validated_cycle_names = []
                self.cycle_status_label.setText(f"INVALID: could not inspect '{name}'.")
                self.run_cycle_button.setEnabled(False)
                return
            self.pending_cycle_details[index] = {'name': name, 'start_pose': response.start_pose, 'target_pose': response.target_pose}
            if len(self.pending_cycle_details) == len(self.pending_cycle_names):
                self.finish_cycle_validation()
        except Exception as error:
            self.validated_cycle_names = []
            self.cycle_status_label.setText(f'Validation failed: {error}')
            self.run_cycle_button.setEnabled(False)

    def finish_cycle_validation(self):
        ordered = [self.pending_cycle_details[i] for i in range(len(self.pending_cycle_names))]
        for i in range(len(ordered) - 1):
            current = ordered[i]
            following = ordered[i + 1]
            if current['target_pose'] != following['start_pose']:
                self.validated_cycle_names = []
                self.cycle_status_label.setText(f"INVALID CYCLE\n\nStep {i + 1}: {current['name']}\nends at: {current['target_pose']}\n\nStep {i + 2}: {following['name']}\nstarts at: {following['start_pose']}")
                self.run_cycle_button.setEnabled(False)
                return
        description = []
        for i, trajectory in enumerate(ordered):
            description.append(f"{i + 1}. {trajectory['name']} ({trajectory['start_pose']} → {trajectory['target_pose']})")
        self.validated_cycle_names = [trajectory['name'] for trajectory in ordered]
        self.cycle_state = 'READY'
        self.cycle_status_label.setText('VALID CYCLE\n\n' + '\n'.join(description))
        self.run_cycle_button.setEnabled(True)
        self.cycle_progress.setValue(0)

    def set_cycle_selection_enabled(self, enabled):
        self.validate_cycle_button.setEnabled(enabled)
        for box in self.cycle_trajectory_boxes:
            box.setEnabled(enabled)

    def run_cycle(self):
        if not self.validated_cycle_names:
            self.set_status("Validate the cycle before running.")
            return

        if not self.node.go_to_start_client.server_is_ready():
            self.set_status("Go-to-start server is not running.")
            return

        if not self.node.execute_client.server_is_ready():
            self.set_status("Execution server is not running.")
            return

        answer = QMessageBox.question(
            self,
            "Run Experiment Cycle",
            "Run the validated experiment cycle?",
            QMessageBox.Yes | QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return

        if not self.logging_active:
            self.cycle_started_logging = self.start_logging()
        else:
            self.cycle_started_logging = False

        self.cycle_running = True
        self.cycle_paused = False
        self.stop_cycle_requested = False
        self.cycle_current_index = 0
        self.cycle_state = "PREPARING"

        self.run_cycle_button.setEnabled(False)
        self.continue_cycle_button.setEnabled(False)
        self.stop_cycle_button.setEnabled(True)
        self.set_cycle_selection_enabled(False)
        self.cycle_progress.setValue(0)

        first_trajectory = self.validated_cycle_names[0]
        self.cycle_status_label.setText(
            f"PREPARING CYCLE\n\nGoing to start of:\n{first_trajectory}"
        )
        self.set_status(f"Going to start of '{first_trajectory}'...")

        goal = GoToTrajectoryStart.Goal()
        goal.trajectory_name = first_trajectory

        future = self.node.go_to_start_client.send_goal_async(
            goal,
            feedback_callback=self.cycle_motion_feedback,
        )
        future.add_done_callback(self.cycle_go_to_start_goal_response)

    def cycle_motion_feedback(self, feedback_msg):
        self.set_status(feedback_msg.feedback.status)

    def cycle_go_to_start_goal_response(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.cycle_failed('Go-to-start request was rejected.')
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.cycle_go_to_start_result)
        except Exception as error:
            self.cycle_failed(f'Go-to-start failed: {error}')

    def cycle_go_to_start_result(self, future):
        try:
            result = future.result().result
            if not result.success:
                self.cycle_failed('Could not reach cycle start: ' + result.message)
                return
            if self.stop_cycle_requested:
                self.cycle_stopped()
                return
            self.execute_cycle_step()
        except Exception as error:
            self.cycle_failed(f'Go-to-start failed: {error}')

    def execute_cycle_step(self):
        if not self.cycle_running:
            return
        if self.stop_cycle_requested:
            self.cycle_stopped()
            return
        if self.cycle_current_index >= len(self.validated_cycle_names):
            self.cycle_complete()
            return
        self.cycle_paused = False
        self.cycle_state = 'RUNNING'
        self.continue_cycle_button.setEnabled(False)
        name = self.validated_cycle_names[self.cycle_current_index]
        step_number = self.cycle_current_index + 1
        total_steps = len(self.validated_cycle_names)
        self.cycle_status_label.setText(f'RUNNING CYCLE\n\nStep {step_number} of {total_steps}\n{name}')
        self.set_status(f"Executing cycle step {step_number}/{total_steps}: '{name}'")
        progress = int(self.cycle_current_index / total_steps * 100)
        self.cycle_progress.setValue(progress)
        goal = ExecuteTrajectory.Goal()
        goal.trajectory_name = name
        future = self.node.execute_client.send_goal_async(goal, feedback_callback=self.cycle_motion_feedback)
        future.add_done_callback(self.cycle_execute_goal_response)

    def cycle_execute_goal_response(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.cycle_failed('Trajectory execution was rejected.')
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.cycle_execute_result)
        except Exception as error:
            self.cycle_failed(f'Trajectory execution failed: {error}')

    def cycle_execute_result(self, future):
        try:
            result = future.result().result
            if not result.success:
                self.cycle_failed('Cycle stopped: ' + result.message)
                return
            completed_name = self.validated_cycle_names[self.cycle_current_index]
            self.cycle_current_index += 1
            completed = self.cycle_current_index
            total = len(self.validated_cycle_names)
            progress = int(completed / total * 100)
            self.cycle_progress.setValue(progress)
            if completed >= total:
                self.cycle_complete()
                return
            if self.stop_cycle_requested:
                self.cycle_stopped()
                return
            if self.pause_each_step_checkbox.isChecked():
                next_name = self.validated_cycle_names[self.cycle_current_index]
                self.cycle_paused = True
                self.cycle_state = 'PAUSED'
                self.continue_cycle_button.setEnabled(True)
                self.cycle_status_label.setText(f'PAUSED\n\nStep {completed} of {total} complete\n{completed_name}\n\nNext:\n{next_name}\n\nPress CONTINUE when ready.')
                self.set_status(f'Cycle paused after step {completed}.')
                return
            self.execute_cycle_step()
        except Exception as error:
            self.cycle_failed(f'Cycle execution failed: {error}')

    def continue_cycle(self):
        if not self.cycle_running:
            return
        if not self.cycle_paused:
            return
        if self.stop_cycle_requested:
            self.cycle_stopped()
            return
        self.cycle_paused = False
        self.cycle_state = 'RUNNING'
        self.continue_cycle_button.setEnabled(False)
        self.execute_cycle_step()

    def stop_cycle(self):
        if not self.cycle_running:
            return
        self.stop_cycle_requested = True
        self.cycle_state = 'STOP_REQUESTED'
        self.stop_cycle_button.setEnabled(False)
        if self.cycle_paused:
            self.cycle_stopped()
            return
        self.cycle_status_label.setText('STOP REQUESTED\n\nThe current motion will finish first.\nThe cycle will stop at the next step boundary.')
        self.set_status('Stop requested. Waiting for the current trajectory to finish.')

    def cycle_complete(self):
        self.cycle_running = False
        self.cycle_paused = False
        self.stop_cycle_requested = False
        self.cycle_state = "COMPLETE"

        self.cycle_progress.setValue(100)
        self.cycle_status_label.setText("CYCLE COMPLETE")
        self.set_status("Experiment cycle completed successfully.")

        self.run_cycle_button.setEnabled(True)
        self.continue_cycle_button.setEnabled(False)
        self.stop_cycle_button.setEnabled(False)
        self.set_cycle_selection_enabled(True)

        self._stop_cycle_owned_logging()

    def cycle_stopped(self):
        self.cycle_running = False
        self.cycle_paused = False
        self.stop_cycle_requested = False
        self.cycle_state = "STOPPED"

        self.cycle_status_label.setText(
            "CYCLE STOPPED\n\nThe robot is at a completed step boundary."
        )
        self.set_status("Experiment cycle stopped.")

        self.run_cycle_button.setEnabled(True)
        self.continue_cycle_button.setEnabled(False)
        self.stop_cycle_button.setEnabled(False)
        self.set_cycle_selection_enabled(True)

        self._stop_cycle_owned_logging()

    def cycle_failed(self, message):
        self.cycle_running = False
        self.cycle_paused = False
        self.stop_cycle_requested = False
        self.cycle_state = "FAILED"

        self.cycle_status_label.setText("CYCLE STOPPED\n\n" + message)
        self.set_status(message)

        self.run_cycle_button.setEnabled(True)
        self.continue_cycle_button.setEnabled(False)
        self.stop_cycle_button.setEnabled(False)
        self.set_cycle_selection_enabled(True)

        self._stop_cycle_owned_logging()

    def _stop_cycle_owned_logging(self):
        if self.cycle_started_logging and self.logging_active:
            self.stop_logging()
        self.cycle_started_logging = False
