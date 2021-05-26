#include "../inc/visualization.h"
#include "../inc/optimal_planner.h"

namespace teb_local_planner
{

    TebVisualization::TebVisualization() : initialized_(false)
    {
    }

    TebVisualization::TebVisualization(const TebConfig& cfg) : initialized_(false)
    {
        initialize(cfg);
    }

    void TebVisualization::initialize(const TebConfig& cfg)
    {
        cfg_ = &cfg;
        initialized_ = true;
    }

    void TebVisualization::publishGlobalPlan(const std::vector<PoseStamped>& global_plan) const
    {
    }

    void TebVisualization::publishLocalPlan(const std::vector<PoseStamped>& local_plan) const
    {
    }

    void TebVisualization::publishLocalPlanAndPoses(const TimedElasticBand& teb) const
    {
    }

    void TebVisualization::publishInfeasibleRobotPose(const PoseSE2& current_pose, const BaseRobotFootprintModel& robot_model)
    {
        //publishRobotFootprintModel(current_pose, robot_model, "InfeasibleRobotPoses", toColorMsg(0.5, 0.8, 0.0, 0.0));
    }


    void TebVisualization::publishObstacles(const ObstContainer& obstacles) const
    {
    }


    void TebVisualization::publishViaPoints(const std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& via_points, const std::string& ns) const
    {
    }

    void TebVisualization::publishTebContainer(const TebOptPlannerContainer& teb_planner, const std::string& ns)
    {
    }

    void TebVisualization::publishFeedbackMessage(const std::vector< boost::shared_ptr<TebOptimalPlanner> >& teb_planners,
                                                  unsigned int selected_trajectory_idx, const ObstContainer& obstacles)
    {
    }

    void TebVisualization::publishFeedbackMessage(const TebOptimalPlanner& teb_planner, const ObstContainer& obstacles)
    {
    }

    bool TebVisualization::printErrorWhenNotInitialized() const
    {
    }

} // namespace teb_local_planner