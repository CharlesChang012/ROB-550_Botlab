#include <planning/motion_planner.hpp>
#include <planning/astar.hpp>
#include <utils/grid_utils.hpp>
#include <utils/timestamp.h>
#include <mbot_lcm_msgs/path2D_t.hpp>
#include <cmath>


MotionPlanner::MotionPlanner(const MotionPlannerParams& params)
: params_(params)
{
    setParams(params);
}


MotionPlanner::MotionPlanner(const MotionPlannerParams& params, const SearchParams& searchParams)
: params_(params)
, searchParams_(searchParams)
{
}


mbot_lcm_msgs::path2D_t MotionPlanner::planPath(const mbot_lcm_msgs::pose2D_t& start,
                                                     const mbot_lcm_msgs::pose2D_t& goal,
                                                     const SearchParams& searchParams) const
{
    // If the goal isn't valid, then no path can actually exist
    if(!isValidGoal(goal))
    {
        mbot_lcm_msgs::path2D_t failedPath;
        failedPath.utime = utime_now();
        failedPath.path_length = 1;
        failedPath.path.push_back(start);

        std::cout << "INFO: path rejected due to invalid goal\n";

        return failedPath;
    }

    // Otherwise, use A* to find the path
    return search_for_path(start, goal, distances_, searchParams);
}


mbot_lcm_msgs::path2D_t MotionPlanner::planPath(const mbot_lcm_msgs::pose2D_t& start,
                                                     const mbot_lcm_msgs::pose2D_t& goal) const
{
    return planPath(start, goal, searchParams_);
}


bool MotionPlanner::isValidGoal(const mbot_lcm_msgs::pose2D_t& goal) const
{
    float dx = goal.x - prev_goal.x, dy = goal.y - prev_goal.y;
    float distanceFromPrev = std::sqrt(dx * dx + dy * dy);

    //if there's more than 1 frontier, don't go to a target that is within a robot diameter of the current pose
    // if(num_frontiers != 1 && distanceFromPrev < 2 * searchParams_.minDistanceToObstacle) return false;

    auto goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances_);

    // A valid goal is in the grid
    if(distances_.isCellInGrid(goalCell.x, goalCell.y))
    {
        // And is far enough from obstacles that the robot can physically occupy the space
        // Add an extra cell to account for discretization error and make motion a little safer by not trying to
        // completely snuggle up against the walls in the motion plan
        return distances_(goalCell.x, goalCell.y) > params_.robotRadius;
    }

    // A goal must be in the map for the robot to reach it
    return false;
}

bool MotionPlanner::isValidGoal(const Point<int>& goalCell) const
{
    // A valid goal is in the grid
    if(distances_.isCellInGrid(goalCell.x, goalCell.y))
    {
        // And is far enough from obstacles that the robot can physically occupy the space
        // Add an extra cell to account for discretization error and make motion a little safer by not trying to
        // completely snuggle up against the walls in the motion plan
        return distances_(goalCell.x, goalCell.y) > params_.robotRadius;
    }
    // A goal must be in the map for the robot to reach it
    return false;
}


bool MotionPlanner::isPathSafe(const mbot_lcm_msgs::path2D_t& path) const
{
    ///////////// TODO: Implement your test for a safe path here //////////////////

    for(const auto& pose : path.path)
    {
        auto cell = global_position_to_grid_cell(Point<double>(pose.x, pose.y), distances_);

        // Check if the cell is within the grid and far enough from obstacles
        if(!distances_.isCellInGrid(cell.x, cell.y) || distances_(cell.x, cell.y) <= params_.robotRadius)
        {
            return false; // Path is not safe
        }
    }

    return true; // Path is safe
}


void MotionPlanner::setMap(const OccupancyGrid& map)
{
    distances_.setDistances(map);
}


void MotionPlanner::setParams(const MotionPlannerParams& params)
{
    searchParams_.minDistanceToObstacle = params_.robotRadius;
    searchParams_.maxDistanceWithCost = 10.0 * searchParams_.minDistanceToObstacle;
    searchParams_.distanceCostExponent = 1.0;
}

void MotionPlanner::updateConeDistance(mbot_lcm_msgs::mbot_cone_t& cone)
{
    Point<double> ConePosition(cone.pose.x, cone.pose.y);
    Point<int> coneCell = global_position_to_grid_cell(ConePosition, distances_);
    distances_.setConeDistance(coneCell.x, coneCell.y);
}

void MotionPlanner::updateConeDistanceBack(mbot_lcm_msgs::mbot_cone_t& cone)
{
    Point<double> ConePosition(cone.pose.x, cone.pose.y);
    Point<int> coneCell = global_position_to_grid_cell(ConePosition, distances_);
    distances_.setConeDistanceBack(coneCell.x, coneCell.y);
}