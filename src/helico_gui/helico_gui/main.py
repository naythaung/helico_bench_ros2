import tkinter as tk
from tkinter import ttk, messagebox, simpledialog

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

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


class HelicoGui(Node):

    def __init__(self, root):
        super().__init__("helico_gui")

        self.root = root

        self.root.title("Meca500 Workbench")

        self.root.geometry("900x1100")
        
        self.root.minsize(
            850,
            800,
        )       

        # ---------------------------------------------------------
        # ROS SERVICE CLIENTS
        # ---------------------------------------------------------

        self.list_poses_client = self.create_client(
            ListPoses,
            "/meca/list_poses",
        )

        self.list_trajectories_client = self.create_client(
            ListTrajectories,
            "/meca/list_trajectories",
        )

        self.inspect_trajectory_client = self.create_client(
            InspectTrajectory,
            "/meca/inspect_trajectory",
        )
        
        self.capture_pose_client = self.create_client(
            CapturePose,
            "/meca/capture_pose",
        )

        self.delete_pose_client = self.create_client(
            DeletePose,
            "/meca/delete_pose",
        )

        self.delete_trajectory_client = self.create_client(
            DeleteTrajectory,
            "/meca/delete_trajectory",
)

        # ---------------------------------------------------------
        # ROS ACTION CLIENTS
        # ---------------------------------------------------------

        self.plan_client = ActionClient(
            self,
            PlanTrajectory,
            "/meca/plan_trajectory",
        )

        self.go_to_start_client = ActionClient(
            self,
            GoToTrajectoryStart,
            "/meca/go_to_trajectory_start",
        )

        self.execute_client = ActionClient(
            self,
            ExecuteTrajectory,
            "/meca/execute_trajectory",
        )

        # ---------------------------------------------------------
        # BUILD GUI
        # ---------------------------------------------------------

        self.build_gui()


        # ---------------------------------------------------------
        # START ROS / GUI LOOP
        # ---------------------------------------------------------

        self.root.after(
            200,
            self.wait_for_backend,
        )

        self.root.after(
            50,
            self.spin_ros,
        )

    # =============================================================
    # GUI
    # =============================================================

    def build_gui(self):

        main = ttk.Frame(
            self.root,
            padding=20,
        )

        main.pack(
            fill="both",
            expand=True,
        )

        # ---------------------------------------------------------
        # TITLE
        # ---------------------------------------------------------

        title = ttk.Label(
            main,
            text="Meca500 Workbench",
            font=(
                "Arial",
                20,
                "bold",
            ),
        )

        title.pack(
            pady=(0, 20),
        )

        # ---------------------------------------------------------
        # PLANNING PANEL
        # ---------------------------------------------------------

        planning_frame = ttk.LabelFrame(
            main,
            text="Trajectory Planning",
            padding=15,
        )

        planning_frame.pack(
            fill="x",
            pady=(0, 15),
        )

        ttk.Label(
            planning_frame,
            text="Start pose",
        ).grid(
            row=0,
            column=0,
            sticky="w",
        )

        self.start_pose_box = ttk.Combobox(
            planning_frame,
            state="readonly",
            width=25,
        )

        self.start_pose_box.grid(
            row=0,
            column=1,
            padx=10,
            pady=5,
        )

        ttk.Label(
            planning_frame,
            text="Target pose",
        ).grid(
            row=1,
            column=0,
            sticky="w",
        )

        self.target_pose_box = ttk.Combobox(
            planning_frame,
            state="readonly",
            width=25,
        )

        self.target_pose_box.grid(
            row=1,
            column=1,
            padx=10,
            pady=5,
        )

        ttk.Label(
            planning_frame,
            text="Trajectory name",
        ).grid(
            row=2,
            column=0,
            sticky="w",
        )

        self.trajectory_name_entry = ttk.Entry(
            planning_frame,
            width=28,
        )

        self.trajectory_name_entry.grid(
            row=2,
            column=1,
            padx=10,
            pady=5,
        )

        self.trajectory_name_entry.insert(
            0,
            "gui_trajectory",
        )

        ttk.Label(
            planning_frame,
            text="Candidates",
        ).grid(
            row=3,
            column=0,
            sticky="w",
        )

        self.candidate_spinbox = ttk.Spinbox(
            planning_frame,
            from_=1,
            to=20,
            width=8,
        )

        self.candidate_spinbox.set(3)

        self.candidate_spinbox.grid(
            row=3,
            column=1,
            sticky="w",
            padx=10,
            pady=5,
        )

        self.plan_button = ttk.Button(
            planning_frame,
            text="PLAN TRAJECTORY",
            command=self.plan_trajectory,
        )

        self.plan_button.grid(
            row=4,
            column=0,
            columnspan=2,
            pady=15,
        )

        pose_frame = ttk.LabelFrame(
            main,
            text="Pose Library",
            padding=10,
        )

        pose_frame.pack(
            fill="x",
            pady=(0, 15),
        )

        self.pose_list = tk.Listbox(
            pose_frame,
            height=5,
        )

        self.pose_list.pack(
            fill="x",
            expand=True,
        )

        pose_button_frame = ttk.Frame(
            pose_frame,
        )

        pose_button_frame.pack(
            pady=(10, 0),
        )

        ttk.Button(
            pose_button_frame,
            text="Capture Current Robot Pose",
            command=self.capture_pose,
        ).pack(
            side="left",
            padx=5,
        )

        ttk.Button(
            pose_button_frame,
            text="Delete Pose",
            command=self.delete_pose,
        ).pack(
            side="left",
            padx=5,
        )

        # ---------------------------------------------------------
        # STATUS
        # ---------------------------------------------------------

        status_frame = ttk.LabelFrame(
            main,
            text="Status",
            padding=10,
        )

        status_frame.pack(
            fill="x",
            pady=(0, 15),
        )

        self.status_var = tk.StringVar(value="Starting...")

        self.status_label = ttk.Label(
            status_frame,
            textvariable=self.status_var,
        )

        self.status_label.pack(
            anchor="w",
        )

        self.progress = ttk.Progressbar(
            status_frame,
            mode="determinate",
            maximum=100,
        )

        self.progress.pack(
            fill="x",
            pady=(8, 0),
        )

        # ---------------------------------------------------------
        # TRAJECTORY LIBRARY
        # ---------------------------------------------------------

        trajectory_frame = ttk.LabelFrame(
            main,
            text="Saved Trajectories",
            padding=10,
        )

        trajectory_frame.pack(
            fill="both",
            expand=True,
        )

        self.trajectory_list = tk.Listbox(
            trajectory_frame,
            height=10,
        )

        self.trajectory_list.pack(
            fill="both",
            expand=True,
        )

        self.trajectory_list.bind(
            "<<ListboxSelect>>",
            self.on_trajectory_selected,
        )

        button_frame = ttk.Frame(
            trajectory_frame,
        )

        button_frame.pack(
            pady=10,
        )

        self.refresh_button = ttk.Button(
            button_frame,
            text="Refresh",
            command=self.refresh_all,
        )

        self.refresh_button.pack(
            side="left",
            padx=5,
        )

        self.go_to_start_button = ttk.Button(
            button_frame,
            text="Go To Start",
            command=self.go_to_start,
        )

        self.go_to_start_button.pack(
            side="left",
            padx=5,
        )

        self.execute_button = ttk.Button(
            button_frame,
            text="Execute",
            command=self.execute_trajectory,
        )

        self.execute_button.pack(
            side="left",
            padx=5,
        )
        
        self.delete_trajectory_button = ttk.Button(
            button_frame,
            text="Delete Trajectory",
            command=self.delete_trajectory,
        )

        self.delete_trajectory_button.pack(
            side="left",
            padx=5,
        )
        
        details_frame = ttk.LabelFrame(
            main,
            text="Trajectory Details",
            padding=10,
        )

        details_frame.pack(
            fill="x",
            pady=(15, 0),
        )

        self.trajectory_details_var = tk.StringVar(
            value="Select a saved trajectory."
        )

        self.trajectory_details_label = ttk.Label(
            details_frame,
            textvariable=self.trajectory_details_var,
            justify="left",
        )

        self.trajectory_details_label.pack(
            anchor="w",
        )

    # =============================================================
    # ROS EVENT LOOP
    # =============================================================

    def spin_ros(self):

        if rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.0,
            )

            self.root.after(
                50,
                self.spin_ros,
            )

    # =============================================================
    # REFRESH
    # =============================================================

    def wait_for_backend(self):

        poses_ready = self.list_poses_client.service_is_ready()

        trajectories_ready = self.list_trajectories_client.service_is_ready()

        if poses_ready and trajectories_ready:

            self.status_var.set("Backend connected.")

            self.refresh_all()

        else:

            self.status_var.set("Waiting for workbench backend...")

            self.root.after(
                500,
                self.wait_for_backend,
            )

    def refresh_all(self):

        self.refresh_poses()
        self.refresh_trajectories()

    def refresh_poses(self):

        if not self.list_poses_client.service_is_ready():

            self.status_var.set("Waiting for workbench server...")

            return

        request = ListPoses.Request()

        future = self.list_poses_client.call_async(request)

        future.add_done_callback(self.poses_received)

    def poses_received(self, future):

        try:

            response = future.result()

            poses = list(response.names)
            
            self.pose_list.delete(
                0,
                tk.END,
            )

            for name in poses:
                self.pose_list.insert(
                    tk.END,
                    name,
                )

            pose_types = list(response.types)

            joint_poses = [
                name
                for name, pose_type in zip(poses, pose_types)
                if pose_type == "joint"
            ]

            self.start_pose_box["values"] = joint_poses

            self.target_pose_box["values"] = poses

            if poses:

                if "meca_zero" in poses:
                    self.start_pose_box.set("meca_zero")
                elif joint_poses and not self.start_pose_box.get():
                    self.start_pose_box.set(joint_poses[0])

                if "meca_demo" in poses:
                    self.target_pose_box.set("meca_demo")
                elif not self.target_pose_box.get():
                    self.target_pose_box.set(poses[0])

        except Exception as error:

            self.status_var.set(f"Pose refresh failed: {error}")

    def refresh_trajectories(self):

        if not (self.list_trajectories_client.service_is_ready()):
            return

        request = ListTrajectories.Request()

        future = self.list_trajectories_client.call_async(request)

        future.add_done_callback(self.trajectories_received)

    def trajectories_received(
        self,
        future,
    ):

        try:

            response = future.result()

            self.trajectory_list.delete(
                0,
                tk.END,
            )

            for name in response.names:

                self.trajectory_list.insert(
                    tk.END,
                    name,
                )

            self.status_var.set("Ready")

        except Exception as error:

            self.status_var.set(f"Trajectory refresh failed: {error}")

    # =============================================================
    # PLAN
    # =============================================================

    def plan_trajectory(self):

        start_pose = self.start_pose_box.get()

        target_pose = self.target_pose_box.get()

        trajectory_name = self.trajectory_name_entry.get().strip()

        if not start_pose:

            self.status_var.set("Select a start pose.")

            return

        if not target_pose:

            self.status_var.set("Select a target pose.")

            return

        if not trajectory_name:

            self.status_var.set("Enter a trajectory name.")

            return

        if not self.plan_client.server_is_ready():

            self.status_var.set("Planning server is not running.")

            return

        goal = PlanTrajectory.Goal()

        goal.start_pose = start_pose
        goal.target_pose = target_pose
        goal.trajectory_name = trajectory_name

        goal.num_candidates = int(self.candidate_spinbox.get())

        goal.minimum_required_clearance = 0.0

        goal.weight_clearance = 0.4
        goal.weight_smoothness = 0.3
        goal.weight_path_length = 0.2
        goal.weight_duration = 0.1

        self.status_var.set("Sending planning request...")

        self.progress["value"] = 0

        future = self.plan_client.send_goal_async(
            goal,
            feedback_callback=(self.plan_feedback),
        )

        future.add_done_callback(self.plan_goal_response)

    def plan_feedback(
        self,
        feedback_msg,
    ):

        feedback = feedback_msg.feedback

        self.status_var.set(feedback.status)

        if feedback.total_candidates > 0:

            progress = feedback.current_candidate / feedback.total_candidates * 100.0

            self.progress["value"] = progress

    def plan_goal_response(
        self,
        future,
    ):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.status_var.set("Planning goal rejected.")

            return

        self.status_var.set("Planning...")

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(self.plan_result)

    def plan_result(
        self,
        future,
    ):

        result = future.result().result

        if result.success:

            self.status_var.set(
                "Planned "
                f"'{result.trajectory_name}' "
                f"| candidate "
                f"{result.selected_candidate} "
                f"| clearance "
                f"{result.minimum_clearance * 1000:.1f} mm"
            )

            self.progress["value"] = 100

            self.refresh_trajectories()

        else:

            self.status_var.set("Planning failed: " + result.message)

    def capture_pose(self):

        name = simpledialog.askstring(
            "Capture Pose",
            "Name for the current robot pose:",
            parent=self.root,
        )

        if not name:
            return

        if not self.capture_pose_client.service_is_ready():

            self.status_var.set(
                "Capture pose service is not ready."
            )

            return

        request = CapturePose.Request()
        request.name = name.strip()

        future = self.capture_pose_client.call_async(
            request
        )

        future.add_done_callback(
            self.capture_pose_result
        )


    def capture_pose_result(
        self,
        future,
    ):

        try:

            response = future.result()

            self.status_var.set(
                response.message
            )

            if response.success:
                self.refresh_poses()

        except Exception as error:

            self.status_var.set(
                f"Capture pose failed: {error}"
            )


    def delete_pose(self):

        selection = self.pose_list.curselection()

        if not selection:

            self.status_var.set(
                "Select a pose to delete."
            )

            return

        name = self.pose_list.get(
            selection[0]
        )

        confirmed = messagebox.askyesno(
            "Delete Pose",
            f"Delete pose '{name}'?",
            parent=self.root,
        )

        if not confirmed:
            return

        if not self.delete_pose_client.service_is_ready():

            self.status_var.set(
                "Delete pose service is not ready."
            )

            return

        request = DeletePose.Request()
        request.name = name

        future = self.delete_pose_client.call_async(
            request
        )

        future.add_done_callback(
            self.delete_pose_result
        )


    def delete_pose_result(
        self,
        future,
    ):

        try:

            response = future.result()

            self.status_var.set(
                response.message
            )

            if response.success:
                self.refresh_poses()

        except Exception as error:

            self.status_var.set(
                f"Delete pose failed: {error}"
            )

    # =============================================================
    # SELECTED TRAJECTORY
    # =============================================================

    def selected_trajectory(self):

        selection = self.trajectory_list.curselection()

        if not selection:

            self.status_var.set("Select a saved trajectory first.")

            return None

        return self.trajectory_list.get(selection[0])


    def on_trajectory_selected(
            self,
            event=None,
        ):

            selection = (
                self.trajectory_list
                .curselection()
            )

            if not selection:
                return

            name = (
                self.trajectory_list
                .get(selection[0])
            )

            self.inspect_trajectory(
                name
            )


    def inspect_trajectory(
        self,
        name,
    ):

        if not (
            self.inspect_trajectory_client
            .service_is_ready()
        ):

            self.trajectory_details_var.set(
                "Trajectory inspection service is not ready."
            )

            return

        request = (
            InspectTrajectory.Request()
        )

        request.name = name

        future = (
            self.inspect_trajectory_client
            .call_async(request)
        )

        future.add_done_callback(
            self.trajectory_details_received
        )


    def trajectory_details_received(
        self,
        future,
    ):

        try:

            response = future.result()

            if not response.success:

                self.trajectory_details_var.set(
                    response.message
                )

                return

            details = (
                f"Start pose: {response.start_pose}\n"
                f"Target pose: {response.target_pose}\n"
                f"Candidates: {response.num_candidates}\n"
                f"Minimum clearance: "
                f"{response.minimum_clearance * 1000:.1f} mm\n"
                f"Closest objects: "
                f"{response.closest_object_a} ↔ "
                f"{response.closest_object_b}\n"
                f"Smoothness: "
                f"{response.smoothness:.6f}\n"
                f"Path length: "
                f"{response.path_length:.4f} rad\n"
                f"Duration: "
                f"{response.duration:.3f} s"
            )

            self.trajectory_details_var.set(
                details
            )

        except Exception as error:

            self.trajectory_details_var.set(
                f"Inspection failed: {error}"
            )
    
    def delete_trajectory(self):

        name = self.selected_trajectory()

        if name is None:
            return

        confirmed = messagebox.askyesno(
            "Delete Trajectory",
            f"Delete trajectory '{name}'?",
            parent=self.root,
        )

        if not confirmed:
            return

        if not self.delete_trajectory_client.service_is_ready():

            self.status_var.set(
                "Delete trajectory service is not ready."
            )

            return

        request = DeleteTrajectory.Request()
        request.name = name

        future = self.delete_trajectory_client.call_async(
            request
        )

        future.add_done_callback(
            self.delete_trajectory_result
        )


    def delete_trajectory_result(
        self,
        future,
    ):

        try:

            response = future.result()

            self.status_var.set(
                response.message
            )

            if response.success:

                self.trajectory_details_var.set(
                    "Select a saved trajectory."
                )

                self.refresh_trajectories()

        except Exception as error:

            self.status_var.set(
                f"Delete trajectory failed: {error}"
            )
        
    # =============================================================
    # GO TO START
    # =============================================================

    def go_to_start(self):

        name = self.selected_trajectory()

        if name is None:
            return

        if not (self.go_to_start_client.server_is_ready()):

            self.status_var.set("Go-to-start server is not running.")

            return

        goal = GoToTrajectoryStart.Goal()

        goal.trajectory_name = name

        self.status_var.set(f"Going to start of '{name}'...")

        future = self.go_to_start_client.send_goal_async(
            goal,
            feedback_callback=(self.motion_feedback),
        )

        future.add_done_callback(self.motion_goal_response)

    # =============================================================
    # EXECUTE
    # =============================================================

    def execute_trajectory(self):

        name = self.selected_trajectory()

        if name is None:
            return
        
        confirmed = messagebox.askyesno(
                    "Execute Trajectory",
                    f"Execute trajectory '{name}'?",
                    parent=self.root,
                )
        
        if not confirmed:
            return

        if not (self.execute_client.server_is_ready()):

            self.status_var.set("Execution server is not running.")

            return

        goal = ExecuteTrajectory.Goal()

        goal.trajectory_name = name

        self.status_var.set(f"Executing '{name}'...")

        future = self.execute_client.send_goal_async(
            goal,
            feedback_callback=(self.motion_feedback),
        )

        future.add_done_callback(self.motion_goal_response)

    def motion_feedback(
        self,
        feedback_msg,
    ):

        self.status_var.set(feedback_msg.feedback.status)

    def motion_goal_response(
        self,
        future,
    ):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.status_var.set("Motion goal rejected.")

            return

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(self.motion_result)

    def motion_result(
        self,
        future,
    ):

        result = future.result().result

        self.status_var.set(result.message)


def main():

    rclpy.init()

    root = tk.Tk()

    node = HelicoGui(root)

    try:

        root.mainloop()

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
