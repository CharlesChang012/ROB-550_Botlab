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

    for(auto &ray : movingScan){
        scoreEndpoint(ray, map);
    } 
    
    for(auto &ray : movingScan){
        scoreRay(ray, map);
    } 

    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t &ray, OccupancyGrid &map)
{
    /// TODO: Implement how to score the cell that the laser endpoint hits
    if(ray.range < kMaxLaserDistance_){
        Point<float> rayStart = global_position_to_grid_position(ray.origin, map);

        // Find end cell of the ray
        Point<int> rayEnd;
        rayEnd.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
        rayEnd.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

        // If end cell is in map grid then update the cell odds (is obstacle)
        if(map.isCellInGrid(rayEnd.x, rayEnd.y)){
            increaseCellOdds(rayEnd.x, rayEnd.y, map);
        }
    }  
}

void Mapping::scoreRay(const adjusted_ray_t &ray, OccupancyGrid &map)
{
    /// TODO: Implement how to score the cells that the laser ray passes through
    if(ray.range < kMaxLaserDistance_){
        std::vector<Point<int>> freeCells = bresenham(ray, map);

        // Iterate through all cells except the last one (which is the endpoint)
        for(size_t i = 0; i < freeCells.size() - 1; i++)
        {
            // Check if the cell is within the map grid

            if(map.isCellInGrid(freeCells[i].x, freeCells[i].y))
            {
                decreaseCellOdds(freeCells[i].x, freeCells[i].y, map);
            }
        }
    }

}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t &ray, const OccupancyGrid &map)
{
    /// TODO: Implement the Bresenham's line algorithm to find cells touched by the ray.
    // Extract start and end points
    Point<float> rayStart = global_position_to_grid_position(ray.origin, map);
    int x0 = static_cast<int>(rayStart.x);
    int y0 = static_cast<int>(rayStart.y);
    int x1 = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
    int y1 = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

    // Calculate absolute differences and signs
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;

    // Initialize error term
    int err = dx - dy;

    // Vector to store free cells
    std::vector<Point<int>> freeCells;

    // Current point
    Point<int> startPoint;
    startPoint.x = x0;
    startPoint.y = y0;
    int x = x0;
    int y = y0;

    // Add initial point
    freeCells.push_back(startPoint);

    // Main Bresenham's line algorithm loop
    while (x != x1 || y != y1) {
        // Calculate error term doubling
        int e2 = 2 * err;

        // Move in x direction
        if (e2 >= -dy) {
            err -= dy;
            x += sx;
        }

        // Move in y direction
        if (e2 <= dx) {
            err += dx;
            y += sy;
        }

        // Add current cell
        Point<int> freePoint;
        freePoint.x = x;
        freePoint.y = y;
        freeCells.push_back(freePoint);
    }

    
    return freeCells;
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t &ray, const OccupancyGrid &map)
{
    /// TODO: Implement an alternative approach to find cells touched by the ray.
    float stepSize = 0.5 * map.metersPerCell();       // 0.05 meters for each cell default
    float maxRange = ray.range;
    int numSteps = static_cast<int>(maxRange / stepSize);

    std::vector<Point<int>> touchedCells;
    Point<int> prevCell;
    Point<double> rayStart = global_position_to_grid_position(ray.origin, map);

    for (int i = 0; i < numSteps; ++i) {
        
        Point<int> rayCell;     // grid cell coordinates
        // rayCell = global_position_to_grid_cell(ray.origin, map); // Convert the ray origin to grid coordinates
        rayCell.x = static_cast<int>(i * stepSize * std::cos(ray.theta) * map.cellsPerMeter() + rayStart.x);
        rayCell.y = static_cast<int>(i * stepSize * std::sin(ray.theta) * map.cellsPerMeter() + rayStart.y);

        if (map.isCellInGrid(rayCell.x, rayCell.y)) {
            if (rayCell != prevCell) {      // Avoid duplicates
                touchedCells.push_back(rayCell);
            }
            prevCell = rayCell;
        }
    }
    return touchedCells; // Placeholder
}

void Mapping::increaseCellOdds(int x, int y, OccupancyGrid &map)
{
    /// TODO: Increase the odds of the cell at (x,y)
    if(!initialized_){
        // do nothing
    }
    else if(127 - map(x, y) > kHitOdds_){
        map(x, y) += kHitOdds_;
    }
    else{
        map(x, y) = 127;
    }
}

void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid &map)
{
    /// TODO: Decrease the odds of the cell at (x,y)
    if(!initialized_){
        // do nothing
    }
    else if(map(x, y) + 128 > kMissOdds_){
        map(x, y) -= kMissOdds_;
    }
    else{
        map(x, y) = -128;
    }
}
