#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "meca500_interfaces/action/plan_trajectory.hpp"
#include "meca500_tasks/trajectory_planner.hpp"


class PlanTrajectoryServer : public rclcpp::Node
{
public:
    using PlanTrajectory =
        meca500_interfaces::action::PlanTrajectory;

    using GoalHandlePlanTrajectory =
        rclcpp_action::ServerGoalHandle<PlanTrajectory>;


    PlanTrajectoryServer()
        : Node("meca500_plan_trajectory_server")
    {
        action_server_ =
            rclcpp_action::create_server<PlanTrajectory>(
                this,
                "/meca/plan_trajectory",

                std::bind(
                    &PlanTrajectoryServer::handleGoal,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2),

                std::bind(
                    &PlanTrajectoryServer::handleCancel,
                    this,
                    std::placeholders::_1),

                std::bind(
                    &PlanTrajectoryServer::handleAccepted,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "Plan trajectory action server ready.");
    }


private:
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const PlanTrajectory::Goal> goal)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Planning request: %s -> %s",
            goal->start_pose.c_str(),
            goal->target_pose.c_str());

        if (goal->start_pose.empty() ||
            goal->target_pose.empty() ||
            goal->trajectory_name.empty())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Rejected planning goal: missing required name.");

            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }


    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<
            GoalHandlePlanTrajectory>)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Received request to cancel planning.");

        return rclcpp_action::CancelResponse::ACCEPT;
    }


    void handleAccepted(
        const std::shared_ptr<
            GoalHandlePlanTrajectory> goal_handle)
    {
        std::thread(
            [this, goal_handle]()
            {
                execute(goal_handle);
            })
            .detach();
    }


    void execute(
        const std::shared_ptr<
            GoalHandlePlanTrajectory> goal_handle)
    {
        const auto goal =
            goal_handle->get_goal();

        auto result =
            std::make_shared<
                PlanTrajectory::Result>();


        meca500_tasks::PlanningRequest request;

        request.start_pose =
            goal->start_pose;

        request.target_pose =
            goal->target_pose;

        request.trajectory_name =
            goal->trajectory_name;

        request.num_candidates =
            goal->num_candidates;

        request.minimum_required_clearance =
            goal->minimum_required_clearance;

        request.weight_clearance =
            goal->weight_clearance;

        request.weight_smoothness =
            goal->weight_smoothness;

        request.weight_path_length =
            goal->weight_path_length;

        request.weight_duration =
            goal->weight_duration;


        auto feedback_callback =
            [goal_handle](
                int current_candidate,
                int total_candidates,
                const std::string &status)
            {
                auto feedback =
                    std::make_shared<
                        PlanTrajectory::Feedback>();

                feedback->current_candidate =
                    current_candidate;

                feedback->total_candidates =
                    total_candidates;

                feedback->status =
                    status;

                goal_handle->publish_feedback(
                    feedback);
            };


        const auto planning_result =
            meca500_tasks::planTrajectory(
                shared_from_this(),
                request,
                feedback_callback);


        result->success =
            planning_result.success;

        result->message =
            planning_result.message;

        result->trajectory_name =
            planning_result.trajectory_name;

        result->selected_candidate =
            planning_result.selected_candidate;

        result->weighted_cost =
            planning_result.weighted_cost;

        result->minimum_clearance =
            planning_result.metrics.minimum_clearance;

        result->smoothness =
            planning_result.metrics.smoothness;

        result->path_length =
            planning_result.metrics.path_length;

        result->duration =
            planning_result.metrics.duration;


        if (planning_result.success)
        {
            goal_handle->succeed(
                result);
        }
        else
        {
            goal_handle->abort(
                result);
        }
    }


    rclcpp_action::Server<
        PlanTrajectory>::SharedPtr
        action_server_;
};


int main(int argc, char **argv)
{
    rclcpp::init(
        argc,
        argv);

    auto node =
        std::make_shared<
            PlanTrajectoryServer>();

    rclcpp::spin(
        node);

    rclcpp::shutdown();

    return 0;
}
