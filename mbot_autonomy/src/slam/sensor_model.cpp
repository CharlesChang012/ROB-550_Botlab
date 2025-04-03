#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <utils/geometric/point.hpp>
#include <queue>

SensorModel::SensorModel(void)
:   sigma_hit_(0.075),
	occupancy_threshold_(10),
	ray_stride_(7),
	max_ray_range_(1000),
    search_range(2),
    offset_quality_weight(3)
{
    initialize_bfs_offsets();
}

void SensorModel::initialize_bfs_offsets()
{
    /// TODO: Initialize the BFS offsets based on the search range 
    bfs_offsets_.clear();

    for(int x = -search_range; x <= search_range; x++){
        for(int y = -search_range; y <= search_range; y++){
            bfs_offsets_.emplace_back(x, y);
        }
    }
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    /// TODO: Compute the likelihood of the given particle using the provided laser scan and map. 
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);
    double scanScore = 0.0;
    
    for(auto &ray : movingScan){

        Point<float> rayEnd = getRayEndPointOnMap(ray, map);

        if(map.logOdds(rayEnd.x, rayEnd.y) > 0.0){
            scanScore += 1.0;
        }
    }
    
    /*
    for(auto &ray : movingScan){
        scanScore += scoreRay(ray, map);
    }
    */
    
    return scanScore; // Placeholder
}

double SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Compute a score for a given ray based on its end point and the map. 
    // Consider the offset from the nearest occupied cell.  
    Point<float> rayEnd = getRayEndPointOnMap(ray, map);
    Point<int> rayEndCell;
    rayEndCell.x = static_cast<int>(rayEnd.x);
    rayEndCell.y = static_cast<int>(rayEnd.y);

    Point<int> nearestCell = gridBFS(rayEndCell, map);
    double dist = distanceBtwTwoCells(rayEndCell, nearestCell);

    double score = 0.0;
    if(dist == 0.0){
        score = 2;
    }
    else if(dist > 1.0){
        score = 1/dist;
    }
    else{
        score = 1;
    }
    
    return score; // Placeholder
}

double SensorModel::NormalPdf(const double& x)
{
    return (1.0/(sqrt(2.0 * M_PI)*sigma_hit_))*exp((-0.5*x*x)/(sigma_hit_*sigma_hit_));
}

Point<int> SensorModel::gridBFS(const Point<int> end_point, const OccupancyGrid& map)
{
    /// TODO: Use Breadth First Search to find the nearest occupied cell to the given end point. 
    // BFS initialization
    std::queue<Point<int>> bfsQueue;
    std::vector<std::vector<bool>> visited(map.widthInCells(), std::vector<bool>(map.heightInCells(), false));
    
    // Use the offsets to guide the search around the ray's endpoint
    for (const auto& offset : bfs_offsets_) {
        Point<int> neighbor = {end_point.x + offset.x, end_point.y + offset.y};

        // Check boundaries and if the neighbor is not visited
        if (map.isCellInGrid(neighbor.x, neighbor.y) && 
            !visited[neighbor.x][neighbor.y]) {
            
            visited[neighbor.x][neighbor.y] = true;
            bfsQueue.push(neighbor);
        }
    }
    
    // Perform BFS within the offset-guided search space
    while (!bfsQueue.empty()) {
        Point<int> current = bfsQueue.front();
        bfsQueue.pop();
        
        // Check if the current cell is occupied
        if (map.logOdds(current.x, current.y) > 0.0) {
            return current;  // Return the first occupied cell found
        }
        
        // Explore neighbors
        for (const auto& offset : bfs_offsets_) {
            Point<int> neighbor = {current.x + offset.x, current.y + offset.y};
            
            // Check boundaries and if the neighbor is already visited
            if (map.isCellInGrid(neighbor.x, neighbor.y) && 
                !visited[neighbor.x][neighbor.y]) {
                
                visited[neighbor.x][neighbor.y] = true;
                bfsQueue.push(neighbor);
            }
        }
    }
        
    // If no occupied cell is found within the search range, return a default point   
    return Point<int>(0,0); // Placeholder
}

Point<float> SensorModel::getRayEndPointOnMap(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    Point<double> endPoint(ray.origin.x + ray.range * std::cos(ray.theta), 
                           ray.origin.y + ray.range * std::sin(ray.theta));

    Point<float> rayEnd = global_position_to_grid_position(endPoint, map);

    return rayEnd;
}

/*---------------------- User Defined Functions -------------------*/
double SensorModel::distanceBtwTwoCells(Point<int> cell1, Point<int> cell2){

    int dist_x = cell1.x - cell2.x;
    int dist_y = cell1.y - cell2.y;
    double dist = std::sqrt(dist_x * dist_x + dist_y * dist_y);

    return dist;
}
