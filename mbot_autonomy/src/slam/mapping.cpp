#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono>
using namespace std::chrono;

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
    : kMaxLaserDistance_(maxLaserDistance), kHitOdds_(hitOdds), kMissOdds_(missOdds), initialized_(false)
{
}

void Mapping::updateMap(const mbot_lcm_msgs::lidar_t &scan,
                        const mbot_lcm_msgs::pose2D_t &pose,
                        OccupancyGrid &map)
{
    if (!initialized_)
        previousPose_ = pose;
    initialized_ = true;

    MovingLaserScan movingScan(scan, previousPose_, pose);

    /// TODO: Update the map's log odds using the movingScan
    //
    // Hint: Consider both the cells the laser hit and the cells it passed through.

    

    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t &ray, OccupancyGrid &map)
{
    /// TODO: Implement how to score the cell that the laser endpoint hits

    
}

void Mapping::scoreRay(const adjusted_ray_t &ray, OccupancyGrid &map)
{
    /// TODO: Implement how to score the cells that the laser ray passes through

    
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t &ray, const OccupancyGrid &map)
{
    /// TODO: Implement the Bresenham's line algorithm to find cells touched by the ray.
    
    return {}; // Placeholder
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t &ray, const OccupancyGrid &map)
{
    /// TODO: Implement an alternative approach to find cells touched by the ray.
    
    return {}; // Placeholder
}

void Mapping::increaseCellOdds(int x, int y, OccupancyGrid &map)
{
    /// TODO: Increase the odds of the cell at (x,y)
}

void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid &map)
{
    /// TODO: Decrease the odds of the cell at (x,y)
}
