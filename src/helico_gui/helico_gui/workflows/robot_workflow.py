from PySide6.QtWidgets import QMessageBox, QInputDialog

from meca500_interfaces.srv import (
    ListPoses,
    CapturePose,
    DeletePose,
    ListTrajectories,
    InspectTrajectory,
    DeleteTrajectory,
)
from meca500_interfaces.action import (
    PlanTrajectory,
    GoToTrajectoryStart,
    ExecuteTrajectory,
)


class RobotWorkflowMixin:
    """Pose, trajectory planning, inspection and single-motion workflows."""


    def update_default_trajectory_name(self):
        start = self.start_pose_box.currentText().strip()
        target = self.target_pose_box.currentText().strip()
        if start and target:
            self.trajectory_name_entry.setText(f'{start}_TO_{target}')

    def refresh_all(self):
        self.refresh_poses()
        self.refresh_trajectories()

    def refresh_poses(self):
        if not self.node.list_poses_client.service_is_ready():
            return
        future = self.node.list_poses_client.call_async(ListPoses.Request())
        future.add_done_callback(self.poses_received)

    def poses_received(self, future):
        try:
            response = future.result()
            names = list(response.names)
            types = list(response.types)
            joint_poses = [name for name, pose_type in zip(names, types) if pose_type == 'joint']
            self.pose_list.clear()
            self.pose_list.addItems(names)
            self.start_pose_box.clear()
            self.start_pose_box.addItems(joint_poses)
            self.target_pose_box.clear()
            self.target_pose_box.addItems(names)
            if 'meca_zero' in joint_poses:
                self.start_pose_box.setCurrentText('meca_zero')
            if 'meca_demo' in names:
                self.target_pose_box.setCurrentText('meca_demo')
        except Exception as error:
            self.set_status(f'Pose refresh failed: {error}')

    def refresh_trajectories(self):
        if not self.node.list_trajectories_client.service_is_ready():
            return
        future = self.node.list_trajectories_client.call_async(ListTrajectories.Request())
        future.add_done_callback(self.trajectories_received)

    def trajectories_received(self, future):
        try:
            response = future.result()
            names = list(response.names)
            self.trajectory_list.clear()
            self.trajectory_list.addItems(names)
            for box in self.cycle_trajectory_boxes:
                current = box.currentText()
                box.blockSignals(True)
                box.clear()
                box.addItem('-- select trajectory --')
                box.addItems(names)
                if current in names:
                    box.setCurrentText(current)
                box.blockSignals(False)
            self.set_status('Ready')
        except Exception as error:
            self.set_status(f'Trajectory refresh failed: {error}')

    def plan_trajectory(self):
        start_pose = self.start_pose_box.currentText()
        target_pose = self.target_pose_box.currentText()
        trajectory_name = self.trajectory_name_entry.text().strip()
        if not start_pose:
            self.set_status('Select a start pose.')
            return
        if not target_pose:
            self.set_status('Select a target pose.')
            return
        if not trajectory_name:
            self.set_status('Enter a trajectory name.')
            return
        if not self.node.plan_client.server_is_ready():
            self.set_status('Planning server is not running.')
            return
        goal = PlanTrajectory.Goal()
        goal.start_pose = start_pose
        goal.target_pose = target_pose
        goal.trajectory_name = trajectory_name
        goal.num_candidates = self.candidate_spinbox.value()
        speed = self.speed_spinbox.value() / 100.0
        goal.velocity_scaling = speed
        goal.acceleration_scaling = speed
        goal.minimum_required_clearance = self.clearance_spinbox.value() / 1000.0
        goal.weight_clearance = 0.4
        goal.weight_smoothness = 0.3
        goal.weight_path_length = 0.2
        goal.weight_duration = 0.1
        self.progress.setValue(0)
        self.set_status('Sending planning request...')
        future = self.node.plan_client.send_goal_async(goal, feedback_callback=self.plan_feedback)
        future.add_done_callback(self.plan_goal_response)

    def plan_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.set_status(feedback.status)
        status = feedback.status.lower()
        if 'initialising' in status:
            progress = 5
        elif feedback.total_candidates > 0 and ('planning candidate' in status or 'evaluating candidate' in status):
            total_steps = feedback.total_candidates * 2
            candidate_index = max(feedback.current_candidate - 1, 0)
            if 'planning candidate' in status:
                completed_steps = candidate_index * 2
            else:
                completed_steps = candidate_index * 2 + 1
            fraction = completed_steps / total_steps
            progress = int(5 + fraction * 80)
        elif 'scoring' in status:
            progress = 90
        elif 'saving' in status:
            progress = 97
        else:
            progress = min(self.progress.value(), 99)
        self.progress.setValue(min(progress, 99))

    def plan_goal_response(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.set_status('Planning goal rejected.')
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.plan_result)
        except Exception as error:
            self.set_status(f'Planning request failed: {error}')

    def plan_result(self, future):
        try:
            result = future.result().result
            if result.success:
                self.progress.setValue(100)
                self.set_status(f"Planned '{result.trajectory_name}' | candidate {result.selected_candidate} | clearance {result.minimum_clearance * 1000:.1f} mm")
                self.refresh_trajectories()
            else:
                self.set_status('Planning failed: ' + result.message)
        except Exception as error:
            self.set_status(f'Planning result failed: {error}')

    def selected_trajectory(self):
        item = self.trajectory_list.currentItem()
        if item is None:
            self.set_status('Select a saved trajectory first.')
            return None
        return item.text()

    def inspect_trajectory(self, name):
        if not name:
            return
        if not self.node.inspect_trajectory_client.service_is_ready():
            return
        request = InspectTrajectory.Request()
        request.name = name
        future = self.node.inspect_trajectory_client.call_async(request)
        future.add_done_callback(self.trajectory_details_received)

    def trajectory_details_received(self, future):
        try:
            response = future.result()
            if not response.success:
                self.details_label.setText(response.message)
                return
            clearance_pass = response.minimum_clearance >= response.minimum_required_clearance
            clearance_status = 'PASS' if clearance_pass else 'FAIL'
            self.details_label.setText(f'{response.start_pose}  → {response.target_pose}\n\nPLANNING SETTINGS\nSpeed scaling: {response.velocity_scaling * 100:.0f}%\nAcceleration scaling: {response.acceleration_scaling * 100:.0f}%\nRequired clearance: {response.minimum_required_clearance * 1000:.1f} mm\nCandidates: {response.num_candidates}\nPlanner: {response.planner_id}\n\nSCORING WEIGHTS\nClearance: {response.weight_clearance:.2f}\nSmoothness: {response.weight_smoothness:.2f}\nPath length: {response.weight_path_length:.2f}\nDuration: {response.weight_duration:.2f}\n\nRESULT\nMinimum clearance: {response.minimum_clearance * 1000:.2f} mm [{clearance_status}]\nClosest objects: {response.closest_object_a} ↔ {response.closest_object_b}\nSmoothness: {response.smoothness:.6f}\nPath length: {response.path_length:.4f} rad\nDuration: {response.duration:.2f} s')
        except Exception as error:
            self.details_label.setText(f'Inspection failed: {error}')

    def go_to_start(self):
        name = self.selected_trajectory()
        if name is None:
            return
        if not self.node.go_to_start_client.server_is_ready():
            self.set_status('Go-to-start server is not running.')
            return
        goal = GoToTrajectoryStart.Goal()
        goal.trajectory_name = name
        self.set_status(f"Going to start of '{name}'...")
        future = self.node.go_to_start_client.send_goal_async(goal, feedback_callback=self.motion_feedback)
        future.add_done_callback(self.motion_goal_response)

    def execute_trajectory(self):
        name = self.selected_trajectory()
        if name is None:
            return
        answer = QMessageBox.question(self, 'Execute Trajectory', f"Execute trajectory '{name}'?", QMessageBox.Yes | QMessageBox.No)
        if answer != QMessageBox.Yes:
            return
        if not self.node.execute_client.server_is_ready():
            self.set_status('Execution server is not running.')
            return
        goal = ExecuteTrajectory.Goal()
        goal.trajectory_name = name
        self.set_status(f"Executing '{name}'...")
        future = self.node.execute_client.send_goal_async(goal, feedback_callback=self.motion_feedback)
        future.add_done_callback(self.motion_goal_response)

    def motion_feedback(self, feedback_msg):
        self.set_status(feedback_msg.feedback.status)

    def motion_goal_response(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.set_status('Motion goal rejected.')
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.motion_result)
        except Exception as error:
            self.set_status(f'Motion request failed: {error}')

    def motion_result(self, future):
        try:
            result = future.result().result
            self.set_status(result.message)
        except Exception as error:
            self.set_status(f'Motion result failed: {error}')

    def capture_pose(self):
        name, ok = QInputDialog.getText(self, 'Save Current Robot Pose', 'Pose name:')
        name = name.strip()
        if not ok or not name:
            return
        if not self.node.capture_pose_client.service_is_ready():
            self.set_status('Capture pose service is not ready.')
            return
        request = CapturePose.Request()
        request.name = name
        future = self.node.capture_pose_client.call_async(request)
        future.add_done_callback(self.capture_pose_result)

    def capture_pose_result(self, future):
        try:
            response = future.result()
            self.set_status(response.message)
            if response.success:
                self.refresh_poses()
        except Exception as error:
            self.set_status(f'Save pose failed: {error}')

    def delete_pose(self):
        item = self.pose_list.currentItem()
        if item is None:
            self.set_status('Select a pose to delete.')
            return
        name = item.text()
        answer = QMessageBox.question(self, 'Delete Pose', f"Delete pose '{name}'?", QMessageBox.Yes | QMessageBox.No)
        if answer != QMessageBox.Yes:
            return
        request = DeletePose.Request()
        request.name = name
        future = self.node.delete_pose_client.call_async(request)
        future.add_done_callback(self.delete_pose_result)

    def delete_pose_result(self, future):
        try:
            response = future.result()
            self.set_status(response.message)
            if response.success:
                self.refresh_poses()
        except Exception as error:
            self.set_status(f'Delete pose failed: {error}')

    def delete_trajectory(self):
        name = self.selected_trajectory()
        if name is None:
            return
        answer = QMessageBox.question(self, 'Delete Trajectory', f"Delete trajectory '{name}'?", QMessageBox.Yes | QMessageBox.No)
        if answer != QMessageBox.Yes:
            return
        request = DeleteTrajectory.Request()
        request.name = name
        future = self.node.delete_trajectory_client.call_async(request)
        future.add_done_callback(self.delete_trajectory_result)

    def delete_trajectory_result(self, future):
        try:
            response = future.result()
            self.set_status(response.message)
            if response.success:
                self.details_label.setText('Select a saved trajectory.')
                self.refresh_trajectories()
        except Exception as error:
            self.set_status(f'Delete trajectory failed: {error}')
