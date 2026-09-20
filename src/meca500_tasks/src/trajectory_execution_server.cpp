#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include "meca500_interfaces/action/go_to_trajectory_start.hpp"
#include "meca500_interfaces/action/execute_trajectory.hpp"

#include "meca500_tasks/trajectory_library.hpp"


class TrajectoryExecutionServer : public rclcpp::Node
{
public:
    using GoToStart =
        meca500_interfaces::action::GoToTrajectoryStart;

    using ExecuteTrajectory =
        meca500_interfaces::action::ExecuteTrajectory;

    using GoToStartHandle =
        rclcpp_action::ServerGoalHandle<GoToStart>;

    using ExecuteHandle =
        rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;


    TrajectoryExecutionServer()
        : Node("meca500_trajectory_execution_server")
    {
        // ---------------------------------------------------------
        // GO TO TRAJECTORY START
        // ---------------------------------------------------------

        go_to_start_server_ =
            rclcpp_action::create_server<GoToStart>(
                this,
                "/meca/go_to_trajectory_start",

                std::bind(
                    &TrajectoryExecutionServer::handleGoToStartGoal,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2),

                std::bind(
                    &TrajectoryExecutionServer::handleGoToStartCancel,
                    this,
                    std::placeholders::_1),

                std::bind(
                    &TrajectoryExecutionServer::handleGoToStartAccepted,
                    this,
                    std::placeholders::_1));


        // ---------------------------------------------------------
        // EXECUTE SAVED TRAJECTORY
        // ---------------------------------------------------------

        execute_server_ =
            rclcpp_action::create_server<ExecuteTrajectory>(
                this,
                "/meca/execute_trajectory",

                std::bind(
                    &TrajectoryExecutionServer::handleExecuteGoal,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2),

                std::bind(
                    &TrajectoryExecutionServer::handleExecuteCancel,
                    this,
                    std::placeholders::_1),

                std::bind(
                    &TrajectoryExecutionServer::handleExecuteAccepted,
                    this,
                    std::placeholders::_1));


        RCLCPP_INFO(
            this->get_logger(),
            "Trajectory execution action server ready.");
    }


private:
    // =============================================================
    // GO TO START
    // =============================================================

    rclcpp_action::GoalResponse handleGoToStartGoal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const GoToStart::Goal> goal)
    {
        if (goal->trajectory_name.empty())
        {
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Go-to-start request for '%s'.",
            goal->trajectory_name.c_str());

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }


    rclcpp_action::CancelResponse handleGoToStartCancel(
        const std::shared_ptr<GoToStartHandle>)
    {
        // Proper MoveIt cancellation can be added later.
        return rclcpp_action::CancelResponse::REJECT;
    }


    void handleGoToStartAccepted(
        const std::shared_ptr<GoToStartHandle> goal_handle)
    {
        std::thread(
            [this, goal_handle]()
            {
                executeGoToStart(goal_handle);
            })
            .detach();
    }


    void executeGoToStart(
        const std::shared_ptr<GoToStartHandle> goal_handle)
    {
        const auto goal =
            goal_handle->get_goal();

        auto result =
            std::make_shared<GoToStart::Result>();

        auto feedback =
            std::make_shared<GoToStart::Feedback>();

        try
        {
            // -----------------------------------------------------
            // LOAD TRAJECTORY
            // -----------------------------------------------------

            feedback->status =
                "Loading saved trajectory";

            goal_handle->publish_feedback(
                feedback);


            trajectory_msgs::msg::JointTrajectory
                trajectory;


            if (!meca500_tasks::loadTrajectory(
                    goal->trajectory_name,
                    trajectory))
            {
                throw std::runtime_error(
                    "Could not load trajectory '" +
                    goal->trajectory_name +
                    "'.");
            }


            if (trajectory.points.empty())
            {
                throw std::runtime_error(
                    "Saved trajectory has no points.");
            }


            const auto &start_positions =
                trajectory.points.front().positions;


            // -----------------------------------------------------
            // MOVEIT
            // -----------------------------------------------------

            feedback->status =
                "Planning route to trajectory start";

            goal_handle->publish_feedback(
                feedback);


            using moveit::planning_interface::
                MoveGroupInterface;


            MoveGroupInterface move_group_interface(
                shared_from_this(),
                "meca_arm");


            move_group_interface
                .setPlanningPipelineId(
                    "ompl");

            move_group_interface
                .setPlannerId(
                    "RRTConnectkConfigDefault");

            move_group_interface
                .setPlanningTime(
                    5.0);

            move_group_interface
                .setMaxVelocityScalingFactor(
                    0.2);

            move_group_interface
                .setMaxAccelerationScalingFactor(
                    0.2);

            move_group_interface
                .setStartStateToCurrentState();


            const bool target_set =
                move_group_interface
                    .setJointValueTarget(
                        start_positions);


            if (!target_set)
            {
                throw std::runtime_error(
                    "Could not set saved trajectory start as target.");
            }


            MoveGroupInterface::Plan plan;


            const bool planning_success =
                static_cast<bool>(
                    move_group_interface.plan(
                        plan));


            if (!planning_success)
            {
                throw std::runtime_error(
                    "Could not plan to saved trajectory start.");
            }


            // -----------------------------------------------------
            // EXECUTE ROUTE TO START
            // -----------------------------------------------------

            feedback->status =
                "Moving to trajectory start";

            goal_handle->publish_feedback(
                feedback);


            const auto move_result =
                move_group_interface.execute(
                    plan);


            if (move_result !=
                moveit::core::MoveItErrorCode::SUCCESS)
            {
                throw std::runtime_error(
                    "Execution to trajectory start failed.");
            }


            result->success =
                true;

            result->message =
                "Reached start of trajectory '" +
                goal->trajectory_name +
                "'.";


            goal_handle->succeed(
                result);
        }
        catch (const std::exception &error)
        {
            result->success =
                false;

            result->message =
                error.what();


            RCLCPP_ERROR(
                this->get_logger(),
                "Go-to-start failed: %s",
                error.what());


            goal_handle->abort(
                result);
        }
    }


    // =============================================================
    // EXECUTE SAVED TRAJECTORY
    // =============================================================

    rclcpp_action::GoalResponse handleExecuteGoal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ExecuteTrajectory::Goal> goal)
    {
        if (goal->trajectory_name.empty())
        {
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Execute request for '%s'.",
            goal->trajectory_name.c_str());

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }


    rclcpp_action::CancelResponse handleExecuteCancel(
        const std::shared_ptr<ExecuteHandle>)
    {
        // Proper controller cancellation can be added later.
        return rclcpp_action::CancelResponse::REJECT;
    }


    void handleExecuteAccepted(
        const std::shared_ptr<ExecuteHandle> goal_handle)
    {
        std::thread(
            [this, goal_handle]()
            {
                executeSavedTrajectory(
                    goal_handle);
            })
            .detach();
    }


    void executeSavedTrajectory(
        const std::shared_ptr<ExecuteHandle> goal_handle)
    {
        const auto goal =
            goal_handle->get_goal();

        auto result =
            std::make_shared<
                ExecuteTrajectory::Result>();

        auto feedback =
            std::make_shared<
                ExecuteTrajectory::Feedback>();


        try
        {
            // -----------------------------------------------------
            // LOAD SAVED TRAJECTORY
            // -----------------------------------------------------

            feedback->status =
                "Loading saved trajectory";

            goal_handle->publish_feedback(
                feedback);


            trajectory_msgs::msg::JointTrajectory
                trajectory;


            if (!meca500_tasks::loadTrajectory(
                    goal->trajectory_name,
                    trajectory))
            {
                throw std::runtime_error(
                    "Could not load trajectory '" +
                    goal->trajectory_name +
                    "'.");
            }


            if (trajectory.points.empty())
            {
                throw std::runtime_error(
                    "Saved trajectory has no points.");
            }


            // -----------------------------------------------------
            // GET CURRENT ROBOT STATE
            // -----------------------------------------------------

            feedback->status =
                "Checking trajectory start state";

            goal_handle->publish_feedback(
                feedback);


            using moveit::planning_interface::
                MoveGroupInterface;


            MoveGroupInterface move_group_interface(
                shared_from_this(),
                "meca_arm");


            const auto current_state =
                move_group_interface
                    .getCurrentState(
                        10.0);


            if (!current_state)
            {
                throw std::runtime_error(
                    "No current robot state received.");
            }


            const auto *joint_model_group =
                current_state
                    ->getJointModelGroup(
                        "meca_arm");


            if (!joint_model_group)
            {
                throw std::runtime_error(
                    "Could not find joint group 'meca_arm'.");
            }


            std::vector<double>
                current_joints;


            current_state
                ->copyJointGroupPositions(
                    joint_model_group,
                    current_joints);


            const auto &expected_start =
                trajectory
                    .points
                    .front()
                    .positions;


            if (current_joints.size() !=
                expected_start.size())
            {
                throw std::runtime_error(
                    "Joint count does not match saved trajectory.");
            }


            // -----------------------------------------------------
            // START STATE VALIDATION
            // -----------------------------------------------------

            constexpr double
                start_tolerance = 0.05;


            for (std::size_t i = 0;
                 i < current_joints.size();
                 ++i)
            {
                const double error =
                    std::abs(
                        current_joints[i] -
                        expected_start[i]);


                if (error > start_tolerance)
                {
                    throw std::runtime_error(
                        "Robot is not at the saved trajectory start state. "
                        "Use Go To Start first.");
                }
            }


            // -----------------------------------------------------
            // EXECUTE
            // -----------------------------------------------------

            feedback->status =
                "Executing saved trajectory";

            goal_handle->publish_feedback(
                feedback);


            MoveGroupInterface::Plan
                replay_plan;


            replay_plan
                .trajectory
                .joint_trajectory =
                    trajectory;


            const auto execute_result =
                move_group_interface.execute(
                    replay_plan);


            if (execute_result !=
                moveit::core::MoveItErrorCode::SUCCESS)
            {
                throw std::runtime_error(
                    "Saved trajectory execution failed.");
            }


            result->success =
                true;

            result->message =
                "Trajectory '" +
                goal->trajectory_name +
                "' executed successfully.";


            goal_handle->succeed(
                result);
        }
        catch (const std::exception &error)
        {
            result->success =
                false;

            result->message =
                error.what();


            RCLCPP_ERROR(
                this->get_logger(),
                "Trajectory execution failed: %s",
                error.what());


            goal_handle->abort(
                result);
        }
    }


    // =============================================================
    // ACTION SERVERS
    // =============================================================

    rclcpp_action::Server<
        GoToStart>::SharedPtr
        go_to_start_server_;

    rclcpp_action::Server<
        ExecuteTrajectory>::SharedPtr
        execute_server_;
};


int main(int argc, char **argv)
{
    rclcpp::init(
        argc,
        argv);

    auto node =
        std::make_shared<
            TrajectoryExecutionServer>();

    rclcpp::spin(
        node);

    rclcpp::shutdown();

    return 0;
}
