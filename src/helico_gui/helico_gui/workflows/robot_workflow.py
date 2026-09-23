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


CANDIDATE_GREEN = "#2e7d32"
CANDIDATE_ORANGE = "#ed8b00"
CANDIDATE_RED = "#c62828"
CANDIDATE_NEUTRAL = "#666666"


class RobotWorkflowMixin:
    """Pose, trajectory planning, inspection and single-motion workflows."""

    def update_default_trajectory_name(self):
        start = (
            self.start_pose_box
            .currentText()
            .strip()
        )
        target = (
            self.target_pose_box
            .currentText()
            .strip()
        )

        if start and target:
            self.trajectory_name_entry.setText(
                f"{start}_TO_{target}"
            )

    def refresh_all(self):
        self.refresh_poses()
        self.refresh_trajectories()

    def refresh_poses(self):
        if not self.node.list_poses_client.service_is_ready():
            return

        future = (
            self.node.list_poses_client
            .call_async(
                ListPoses.Request()
            )
        )

        future.add_done_callback(
            self.poses_received
        )

    def poses_received(
        self,
        future,
    ):
        try:
            response = future.result()

            names = list(
                response.names
            )
            types = list(
                response.types
            )

            joint_poses = [
                name
                for name, pose_type
                in zip(
                    names,
                    types,
                )
                if pose_type == "joint"
            ]

            self.pose_list.clear()
            self.pose_list.addItems(
                names
            )

            self.start_pose_box.clear()
            self.start_pose_box.addItems(
                joint_poses
            )

            self.target_pose_box.clear()
            self.target_pose_box.addItems(
                names
            )

            if "meca_zero" in joint_poses:
                self.start_pose_box.setCurrentText(
                    "meca_zero"
                )

            if "meca_demo" in names:
                self.target_pose_box.setCurrentText(
                    "meca_demo"
                )

        except Exception as error:
            self.set_status(
                f"Pose refresh failed: {error}"
            )

    def refresh_trajectories(self):
        if not self.node.list_trajectories_client.service_is_ready():
            return

        future = (
            self.node.list_trajectories_client
            .call_async(
                ListTrajectories.Request()
            )
        )

        future.add_done_callback(
            self.trajectories_received
        )

    def trajectories_received(
        self,
        future,
    ):
        try:
            response = future.result()
            names = list(
                response.names
            )

            self.trajectory_list.clear()
            self.trajectory_list.addItems(
                names
            )

            for box in self.cycle_trajectory_boxes:
                current = box.currentText()

                box.blockSignals(
                    True
                )
                box.clear()
                box.addItem(
                    "-- select trajectory --"
                )
                box.addItems(
                    names
                )

                if current in names:
                    box.setCurrentText(
                        current
                    )

                box.blockSignals(
                    False
                )

            self.set_status(
                "Ready"
            )

        except Exception as error:
            self.set_status(
                f"Trajectory refresh failed: {error}"
            )

    # ============================================================
    # TRAJECTORY PLANNING
    # ============================================================

    def normalised_planning_weights(
        self,
    ):
        values = [
            self.clearance_weight_slider.value(),
            self.smoothness_weight_slider.value(),
            self.path_length_weight_slider.value(),
            self.duration_weight_slider.value(),
        ]

        total = sum(
            values
        )

        if total <= 0:
            return None

        return [
            value / total
            for value in values
        ]

    def plan_trajectory(self):
        start_pose = (
            self.start_pose_box
            .currentText()
        )

        target_pose = (
            self.target_pose_box
            .currentText()
        )

        trajectory_name = (
            self.trajectory_name_entry
            .text()
            .strip()
        )

        if not start_pose:
            self.set_status(
                "Select a start pose."
            )
            return

        if not target_pose:
            self.set_status(
                "Select a target pose."
            )
            return

        if not trajectory_name:
            self.set_status(
                "Enter a trajectory name."
            )
            return

        if not self.node.plan_client.server_is_ready():
            self.set_status(
                "Planning server is not running."
            )
            return

        weights = (
            self.normalised_planning_weights()
        )

        if weights is None:
            self.set_status(
                "At least one scoring weight must be greater than zero."
            )
            return

        goal = PlanTrajectory.Goal()

        goal.start_pose = start_pose
        goal.target_pose = target_pose
        goal.trajectory_name = trajectory_name

        goal.num_candidates = (
            self.candidate_spinbox.value()
        )

        speed = (
            self.speed_spinbox.value()
            / 100.0
        )

        goal.velocity_scaling = speed

        if (
            self.separate_acceleration_checkbox
            .isChecked()
        ):
            goal.acceleration_scaling = (
                self.acceleration_slider.value()
                / 100.0
            )
        else:
            goal.acceleration_scaling = (
                speed
            )

        goal.minimum_required_clearance = (
            self.clearance_spinbox.value()
            / 1000.0
        )

        (
            goal.weight_clearance,
            goal.weight_smoothness,
            goal.weight_path_length,
            goal.weight_duration,
        ) = weights

        self.progress.setValue(
            0
        )

        self.planning_candidates = {}
        self.planning_total_candidates = (
            goal.num_candidates
        )
        self.current_candidate_view = 0
        self.selected_candidate_number = None
        self.selected_candidate_cost = None

        self.previous_candidate_button.setEnabled(
            goal.num_candidates > 1
        )
        self.next_candidate_button.setEnabled(
            goal.num_candidates > 1
        )

        self.candidate_number_label.setText(
            f"Preparing "
            f"{goal.num_candidates} candidates"
        )

        self.planning_candidate_label.setText(
            "Planning starting..."
        )
        self.planning_candidate_label.setStyleSheet(
            "padding: 10px; "
            f"border: 2px solid {CANDIDATE_ORANGE}; "
            "border-radius: 6px; "
            f"color: {CANDIDATE_ORANGE}; "
            "font-weight: bold;"
        )

        self.set_status(
            "Sending planning request..."
        )

        future = (
            self.node.plan_client
            .send_goal_async(
                goal,
                feedback_callback=self.plan_feedback,
            )
        )

        future.add_done_callback(
            self.plan_goal_response
        )

    # ============================================================
    # CANDIDATE BROWSER
    # ============================================================

    def previous_candidate(self):
        total = (
            self.planning_total_candidates
        )

        if total <= 0:
            return

        if self.current_candidate_view <= 1:
            candidate_number = total
        else:
            candidate_number = (
                self.current_candidate_view
                - 1
            )

        self.show_candidate(
            candidate_number
        )

    def next_candidate(self):
        total = (
            self.planning_total_candidates
        )

        if total <= 0:
            return

        if (
            self.current_candidate_view <= 0
            or
            self.current_candidate_view >= total
        ):
            candidate_number = 1
        else:
            candidate_number = (
                self.current_candidate_view
                + 1
            )

        self.show_candidate(
            candidate_number
        )

    def show_candidate(
        self,
        candidate_number,
    ):
        total = (
            self.planning_total_candidates
        )

        if (
            total <= 0
            or
            candidate_number <= 0
        ):
            return

        candidate_number = max(
            1,
            min(
                candidate_number,
                total,
            ),
        )

        self.current_candidate_view = (
            candidate_number
        )

        selected = (
            self.selected_candidate_number
            == candidate_number
        )

        selected_text = (
            "  •  SELECTED"
            if selected
            else ""
        )

        self.candidate_number_label.setText(
            f"Candidate "
            f"{candidate_number} / {total}"
            f"{selected_text}"
        )

        candidate = (
            self.planning_candidates.get(
                candidate_number
            )
        )

        if candidate is None:
            self.planning_candidate_label.setText(
                "Not processed yet."
            )
            self.planning_candidate_label.setStyleSheet(
                "padding: 10px; "
                f"border: 2px solid {CANDIDATE_NEUTRAL}; "
                "border-radius: 6px; "
                f"color: {CANDIDATE_NEUTRAL};"
            )
            return

        if not candidate["complete"]:
            self.planning_candidate_label.setText(
                f"{candidate['status']}\n\n"
                "Processing..."
            )
            self.planning_candidate_label.setStyleSheet(
                "padding: 10px; "
                f"border: 2px solid {CANDIDATE_ORANGE}; "
                "border-radius: 6px; "
                f"color: {CANDIDATE_ORANGE}; "
                "font-weight: bold;"
            )
            return

        if candidate["passed"]:
            result_text = (
                "PASS"
            )
            colour = (
                CANDIDATE_GREEN
            )
        else:
            result_text = (
                "FAIL"
            )
            colour = (
                CANDIDATE_RED
            )

        lines = [
            result_text,
        ]

        message = (
            candidate["message"]
        )

        if message:
            lines.append(
                message
            )

        if candidate["has_metrics"]:
            lines.extend(
                [
                    "",
                    (
                        "Clearance: "
                        f"{candidate['clearance'] * 1000:.1f} mm"
                    ),
                    (
                        "Smoothness: "
                        f"{candidate['smoothness']:.6f}"
                    ),
                    (
                        "Path length: "
                        f"{candidate['path_length']:.3f} rad"
                    ),
                    (
                        "Duration: "
                        f"{candidate['duration']:.2f} s"
                    ),
                ]
            )

        if (
            selected
            and
            self.selected_candidate_cost
            is not None
        ):
            lines.extend(
                [
                    "",
                    (
                        "Relative cost: "
                        f"{self.selected_candidate_cost:.4f}"
                    ),
                ]
            )

        self.planning_candidate_label.setText(
            "\n".join(
                lines
            )
        )

        self.planning_candidate_label.setStyleSheet(
            "padding: 10px; "
            f"border: 2px solid {colour}; "
            "border-radius: 6px; "
            f"color: {colour}; "
            "font-weight: bold;"
        )

    def plan_feedback(
        self,
        feedback_msg,
    ):
        feedback = (
            feedback_msg.feedback
        )

        self.set_status(
            feedback.status
        )

        status = (
            feedback.status.lower()
        )

        if feedback.total_candidates > 0:
            self.planning_total_candidates = (
                feedback.total_candidates
            )

        candidate_number = (
            feedback.current_candidate
        )

        candidate_feedback = (
            "planning candidate" in status
            or
            "evaluating candidate" in status
            or
            "candidate passed" in status
            or
            "candidate rejected" in status
            or
            (
                "candidate " in status
                and
                "failed" in status
            )
        )

        if (
            candidate_number > 0
            and
            candidate_feedback
        ):
            has_metrics = (
                feedback.candidate_complete
                and
                feedback.candidate_message
                != "Planning failed"
            )

            self.planning_candidates[
                candidate_number
            ] = {
                "complete":
                    feedback.candidate_complete,

                "passed":
                    feedback.candidate_passed,

                "status":
                    feedback.status,

                "clearance":
                    feedback.minimum_clearance,

                "smoothness":
                    feedback.smoothness,

                "path_length":
                    feedback.path_length,

                "duration":
                    feedback.duration,

                "message":
                    feedback.candidate_message,

                "has_metrics":
                    has_metrics,
            }

            self.show_candidate(
                candidate_number
            )

        if "initialising" in status:
            progress = 5

        elif (
            feedback.total_candidates > 0
            and
            candidate_number > 0
            and
            candidate_feedback
        ):
            total_steps = (
                feedback.total_candidates
                * 2
            )

            candidate_index = max(
                candidate_number - 1,
                0,
            )

            if "planning candidate" in status:
                completed_steps = (
                    candidate_index
                    * 2
                )

            elif "evaluating candidate" in status:
                completed_steps = (
                    candidate_index
                    * 2
                    + 1
                )

            else:
                completed_steps = (
                    candidate_index
                    * 2
                    + 2
                )

            fraction = min(
                completed_steps
                / total_steps,
                1.0,
            )

            progress = int(
                5
                + fraction * 80
            )

        elif "scoring" in status:
            progress = 90

            self.candidate_number_label.setText(
                "Comparing candidates"
            )

            self.planning_candidate_label.setText(
                "Scoring successful candidates..."
            )
            self.planning_candidate_label.setStyleSheet(
                "padding: 10px; "
                f"border: 2px solid {CANDIDATE_ORANGE}; "
                "border-radius: 6px; "
                f"color: {CANDIDATE_ORANGE}; "
                "font-weight: bold;"
            )

        elif "saving" in status:
            progress = 97

            self.candidate_number_label.setText(
                "Selecting best candidate"
            )

            self.planning_candidate_label.setText(
                "Saving selected trajectory..."
            )
            self.planning_candidate_label.setStyleSheet(
                "padding: 10px; "
                f"border: 2px solid {CANDIDATE_ORANGE}; "
                "border-radius: 6px; "
                f"color: {CANDIDATE_ORANGE}; "
                "font-weight: bold;"
            )

        else:
            progress = min(
                self.progress.value(),
                99,
            )

        self.progress.setValue(
            min(
                progress,
                99,
            )
        )

    def plan_goal_response(
        self,
        future,
    ):
        try:
            goal_handle = (
                future.result()
            )

            if not goal_handle.accepted:
                self.set_status(
                    "Planning goal rejected."
                )
                return

            result_future = (
                goal_handle
                .get_result_async()
            )

            result_future.add_done_callback(
                self.plan_result
            )

        except Exception as error:
            self.set_status(
                f"Planning request failed: {error}"
            )

    def plan_result(
        self,
        future,
    ):
        try:
            result = (
                future
                .result()
                .result
            )

            if result.success:
                self.progress.setValue(
                    100
                )

                self.selected_candidate_number = (
                    result.selected_candidate
                )

                self.selected_candidate_cost = (
                    result.weighted_cost
                )

                selected = (
                    self.planning_candidates.get(
                        result.selected_candidate
                    )
                )

                if selected is None:
                    self.planning_candidates[
                        result.selected_candidate
                    ] = {
                        "complete":
                            True,

                        "passed":
                            True,

                        "status":
                            "Candidate passed",

                        "clearance":
                            result.minimum_clearance,

                        "smoothness":
                            result.smoothness,

                        "path_length":
                            result.path_length,

                        "duration":
                            result.duration,

                        "message":
                            "Passed clearance gate",

                        "has_metrics":
                            True,
                    }

                self.show_candidate(
                    result.selected_candidate
                )

                self.set_status(
                    f"Planned "
                    f"'{result.trajectory_name}' "
                    f"| selected candidate "
                    f"{result.selected_candidate} "
                    f"| cost "
                    f"{result.weighted_cost:.4f}"
                )

                self.refresh_trajectories()

            else:
                self.candidate_number_label.setText(
                    "Planning failed"
                )

                self.planning_candidate_label.setText(
                    "PLANNING FAILED\n\n"
                    + result.message
                )

                self.planning_candidate_label.setStyleSheet(
                    "padding: 10px; "
                    f"border: 2px solid {CANDIDATE_RED}; "
                    "border-radius: 6px; "
                    f"color: {CANDIDATE_RED}; "
                    "font-weight: bold;"
                )

                self.set_status(
                    "Planning failed: "
                    + result.message
                )

        except Exception as error:
            self.candidate_number_label.setText(
                "Planning result error"
            )

            self.planning_candidate_label.setText(
                f"Planning result error:\n"
                f"{error}"
            )

            self.planning_candidate_label.setStyleSheet(
                "padding: 10px; "
                f"border: 2px solid {CANDIDATE_RED}; "
                "border-radius: 6px; "
                f"color: {CANDIDATE_RED}; "
                "font-weight: bold;"
            )

            self.set_status(
                f"Planning result failed: "
                f"{error}"
            )

    # ============================================================
    # SAVED TRAJECTORY LIBRARY
    # ============================================================

    def selected_trajectory(
        self,
    ):
        item = (
            self.trajectory_list
            .currentItem()
        )

        if item is None:
            self.set_status(
                "Select a saved trajectory first."
            )
            return None

        return item.text()

    def inspect_trajectory(
        self,
        name,
    ):
        if not name:
            return

        if not self.node.inspect_trajectory_client.service_is_ready():
            return

        request = (
            InspectTrajectory.Request()
        )
        request.name = name

        future = (
            self.node
            .inspect_trajectory_client
            .call_async(
                request
            )
        )

        future.add_done_callback(
            self.trajectory_details_received
        )

    def trajectory_details_received(
        self,
        future,
    ):
        try:
            response = (
                future.result()
            )

            if not response.success:
                self.details_label.setText(
                    response.message
                )
                return

            clearance_pass = (
                response.minimum_clearance
                >=
                response.minimum_required_clearance
            )

            clearance_status = (
                "PASS"
                if clearance_pass
                else "FAIL"
            )

            self.details_label.setText(
                f"{response.start_pose}  → "
                f"{response.target_pose}\n\n"

                "PLANNING SETTINGS\n"

                "Speed scaling: "
                f"{response.velocity_scaling * 100:.0f}%\n"

                "Acceleration scaling: "
                f"{response.acceleration_scaling * 100:.0f}%\n"

                "Required clearance: "
                f"{response.minimum_required_clearance * 1000:.1f} mm\n"

                "Candidates: "
                f"{response.num_candidates}\n"

                "Planner: "
                f"{response.planner_id}\n\n"

                "SCORING WEIGHTS\n"

                "Clearance: "
                f"{response.weight_clearance:.2f}\n"

                "Smoothness: "
                f"{response.weight_smoothness:.2f}\n"

                "Path length: "
                f"{response.weight_path_length:.2f}\n"

                "Duration: "
                f"{response.weight_duration:.2f}\n\n"

                "RESULT\n"

                "Minimum clearance: "
                f"{response.minimum_clearance * 1000:.2f} mm "
                f"[{clearance_status}]\n"

                "Closest objects: "
                f"{response.closest_object_a} ↔ "
                f"{response.closest_object_b}\n"

                "Smoothness: "
                f"{response.smoothness:.6f}\n"

                "Path length: "
                f"{response.path_length:.4f} rad\n"

                "Duration: "
                f"{response.duration:.2f} s"
            )

        except Exception as error:
            self.details_label.setText(
                f"Inspection failed: {error}"
            )

    # ============================================================
    # SINGLE TRAJECTORY EXECUTION
    # ============================================================

    def go_to_start(
        self,
    ):
        name = (
            self.selected_trajectory()
        )

        if name is None:
            return

        if not self.node.go_to_start_client.server_is_ready():
            self.set_status(
                "Go-to-start server is not running."
            )
            return

        goal = (
            GoToTrajectoryStart.Goal()
        )
        goal.trajectory_name = name

        self.set_status(
            f"Going to start of '{name}'..."
        )

        future = (
            self.node
            .go_to_start_client
            .send_goal_async(
                goal,
                feedback_callback=self.motion_feedback,
            )
        )

        future.add_done_callback(
            self.motion_goal_response
        )

    def execute_trajectory(
        self,
    ):
        name = (
            self.selected_trajectory()
        )

        if name is None:
            return

        answer = QMessageBox.question(
            self,
            "Execute Trajectory",
            f"Execute trajectory '{name}'?",
            QMessageBox.Yes
            |
            QMessageBox.No,
        )

        if answer != QMessageBox.Yes:
            return

        if not self.node.execute_client.server_is_ready():
            self.set_status(
                "Execution server is not running."
            )
            return

        goal = (
            ExecuteTrajectory.Goal()
        )
        goal.trajectory_name = name

        self.set_status(
            f"Executing '{name}'..."
        )

        future = (
            self.node
            .execute_client
            .send_goal_async(
                goal,
                feedback_callback=self.motion_feedback,
            )
        )

        future.add_done_callback(
            self.motion_goal_response
        )

    def motion_feedback(
        self,
        feedback_msg,
    ):
        self.set_status(
            feedback_msg.feedback.status
        )

    def motion_goal_response(
        self,
        future,
    ):
        try:
            goal_handle = (
                future.result()
            )

            if not goal_handle.accepted:
                self.set_status(
                    "Motion goal rejected."
                )
                return

            result_future = (
                goal_handle
                .get_result_async()
            )

            result_future.add_done_callback(
                self.motion_result
            )

        except Exception as error:
            self.set_status(
                f"Motion request failed: {error}"
            )

    def motion_result(
        self,
        future,
    ):
        try:
            result = (
                future
                .result()
                .result
            )

            self.set_status(
                result.message
            )

        except Exception as error:
            self.set_status(
                f"Motion result failed: {error}"
            )

    # ============================================================
    # POSE LIBRARY
    # ============================================================

    def capture_pose(
        self,
    ):
        name, ok = QInputDialog.getText(
            self,
            "Save Current Robot Pose",
            "Pose name:",
        )

        name = (
            name.strip()
        )

        if not ok or not name:
            return

        if not self.node.capture_pose_client.service_is_ready():
            self.set_status(
                "Capture pose service is not ready."
            )
            return

        request = (
            CapturePose.Request()
        )
        request.name = name

        future = (
            self.node
            .capture_pose_client
            .call_async(
                request
            )
        )

        future.add_done_callback(
            self.capture_pose_result
        )

    def capture_pose_result(
        self,
        future,
    ):
        try:
            response = (
                future.result()
            )

            self.set_status(
                response.message
            )

            if response.success:
                self.refresh_poses()

        except Exception as error:
            self.set_status(
                f"Save pose failed: {error}"
            )

    def delete_pose(
        self,
    ):
        item = (
            self.pose_list.currentItem()
        )

        if item is None:
            self.set_status(
                "Select a pose to delete."
            )
            return

        name = (
            item.text()
        )

        answer = QMessageBox.question(
            self,
            "Delete Pose",
            f"Delete pose '{name}'?",
            QMessageBox.Yes
            |
            QMessageBox.No,
        )

        if answer != QMessageBox.Yes:
            return

        request = (
            DeletePose.Request()
        )
        request.name = name

        future = (
            self.node
            .delete_pose_client
            .call_async(
                request
            )
        )

        future.add_done_callback(
            self.delete_pose_result
        )

    def delete_pose_result(
        self,
        future,
    ):
        try:
            response = (
                future.result()
            )

            self.set_status(
                response.message
            )

            if response.success:
                self.refresh_poses()

        except Exception as error:
            self.set_status(
                f"Delete pose failed: {error}"
            )

    def delete_trajectory(
        self,
    ):
        name = (
            self.selected_trajectory()
        )

        if name is None:
            return

        answer = QMessageBox.question(
            self,
            "Delete Trajectory",
            f"Delete trajectory '{name}'?",
            QMessageBox.Yes
            |
            QMessageBox.No,
        )

        if answer != QMessageBox.Yes:
            return

        request = (
            DeleteTrajectory.Request()
        )
        request.name = name

        future = (
            self.node
            .delete_trajectory_client
            .call_async(
                request
            )
        )

        future.add_done_callback(
            self.delete_trajectory_result
        )

    def delete_trajectory_result(
        self,
        future,
    ):
        try:
            response = (
                future.result()
            )

            self.set_status(
                response.message
            )

            if response.success:
                self.details_label.setText(
                    "Select a saved trajectory."
                )
                self.refresh_trajectories()

        except Exception as error:
            self.set_status(
                f"Delete trajectory failed: {error}"
            )
