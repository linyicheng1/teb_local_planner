#include "../inc/teb_config.h"

namespace teb_local_planner
{

    void TebConfig::checkParameters() const
    {
        // positive backward velocity?
        if (robot.max_vel_x_backwards <= 0)
            printf("TebLocalPlannerROS() Param Warning: Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");

        // bounds smaller than penalty epsilon
        if (robot.max_vel_x <= optim.penalty_epsilon)
            printf("TebLocalPlannerROS() Param Warning: max_vel_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");

        if (robot.max_vel_x_backwards <= optim.penalty_epsilon)
            printf("TebLocalPlannerROS() Param Warning: max_vel_x_backwards <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");

        if (robot.max_vel_theta <= optim.penalty_epsilon)
            printf("TebLocalPlannerROS() Param Warning: max_vel_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");

        if (robot.acc_lim_x <= optim.penalty_epsilon)
            printf("TebLocalPlannerROS() Param Warning: acc_lim_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");

        if (robot.acc_lim_theta <= optim.penalty_epsilon)
            printf("TebLocalPlannerROS() Param Warning: acc_lim_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");

        // dt_ref and dt_hyst
        if (trajectory.dt_ref <= trajectory.dt_hysteresis)
            printf("TebLocalPlannerROS() Param Warning: dt_ref <= dt_hysteresis. The hysteresis is not allowed to be greater or equal!. Undefined behavior... Change at least one of them!");

        // min number of samples
        if (trajectory.min_samples <3)
            printf("TebLocalPlannerROS() Param Warning: parameter min_samples is smaller than 3! Sorry, I haven't enough degrees of freedom to plan a trajectory for you. Please increase ...");

        // costmap obstacle behind robot
        if (obstacles.costmap_obstacles_behind_robot_dist < 0)
            printf("TebLocalPlannerROS() Param Warning: parameter 'costmap_obstacles_behind_robot_dist' should be positive or zero.");

        // hcp: obstacle heading threshold
        if (hcp.obstacle_keypoint_offset>=1 || hcp.obstacle_keypoint_offset<=0)
            printf("TebLocalPlannerROS() Param Warning: parameter obstacle_heading_threshold must be in the interval ]0,1[. 0=0deg opening angle, 1=90deg opening angle.");

        // carlike
        if (robot.cmd_angle_instead_rotvel && robot.wheelbase==0)
            printf("TebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but wheelbase is set to zero: undesired behavior.");

        if (robot.cmd_angle_instead_rotvel && robot.min_turning_radius==0)
            printf("TebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but min_turning_radius is set to zero: undesired behavior. You are mixing a carlike and a diffdrive robot");

        // positive weight_adapt_factor
        if (optim.weight_adapt_factor < 1.0)
            printf("TebLocalPlannerROS() Param Warning: parameter weight_adapt_factor shoud be >= 1.0");

        if (recovery.oscillation_filter_duration < 0)
            printf("TebLocalPlannerROS() Param Warning: parameter oscillation_filter_duration must be >= 0");

        // weights
        if (optim.weight_optimaltime <= 0)
            printf("TebLocalPlannerROS() Param Warning: parameter weight_optimaltime shoud be > 0 (even if weight_shortest_path is in use)");

    }

} // namespace teb_local_planner
