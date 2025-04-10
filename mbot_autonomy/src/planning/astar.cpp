#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>

using namespace std::chrono;

mbot_lcm_msgs::path2D_t search_for_path(mbot_lcm_msgs::pose2D_t start,
                                             mbot_lcm_msgs::pose2D_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    bool found_path = false;
    
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    Node* startNode = new Node(startCell.x, startCell.y);
    Node* goalNode = new Node(goalCell.x, goalCell.y);
    
    /* ------------- A star search implementation ------------*/
    printf("\n");
    printf("Start node: %d, %d\n", startCell.x, startCell.y);
    printf("Goal node: %d, %d\n", goalCell.x, goalCell.y);

    if(!found_path && distances.isCellInGrid(goalCell.x, goalCell.y)){
        
        std::vector<Node*> closedList;  // Closed List to store explored nodes
        
        // Initialize start node
        startNode->g_cost = 0.0;
        startNode->h_cost = h_cost(startNode, goalNode, distances);
        startNode->parent = NULL;

        PriorityQueue pq;
        pq.push(startNode);

        while(!pq.empty()){
            Node* curNode = pq.pop();
            closedList.push_back(curNode);

            // If goalNode reached
            if(curNode->cell == goalNode->cell){
                found_path = true;
                goalNode->parent = curNode->parent; // Set the parent of the goal node to the current node
                break;
            }

            std::vector<Node*> neighbors = expand_node(curNode, distances, params);

            for(auto &neighbor : neighbors){

                // Check if explored, pass this node
                if(is_in_list(neighbor, closedList)){
                    continue;
                }
    
                neighbor->g_cost = g_cost(curNode, neighbor, distances, params);
                neighbor->h_cost = h_cost(neighbor, goalNode, distances);
                
                // Check if is better path
                if (!pq.is_member(neighbor)) {    // If not in checking list, push
                    neighbor->parent = curNode;
                    pq.push(neighbor);
                } else {
                    Node* existingNode = pq.get_member(neighbor);
                    if (neighbor->g_cost < existingNode->g_cost) {  // If new path has lower cost, push
                        existingNode->g_cost = neighbor->g_cost;
                        existingNode->parent = curNode;
                    }
                }

            }
        }
    }
    /* ------------------------------------------------------ */
    mbot_lcm_msgs::path2D_t path;
    path.utime = start.utime;
    if (found_path)
    {
        auto nodePath = extract_node_path(goalNode, startNode);
        path.path = extract_pose_path(nodePath, distances);
        // Remove last pose, and add the goal pose
        path.path.pop_back();
        path.path.push_back(goal);
    }

    else printf("[A*] Didn't find a path\n");
    path.path_length = path.path.size();
    return path;
}



double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    double h_cost = 0.0;
    ////////////////// TODO: Implement your heuristic //////////////////////////
    double dx = fabs(from->cell.x - goal->cell.x);
    double dy = fabs(from->cell.y - goal->cell.y);

    h_cost = (dx + dy) + (std::sqrt(2) - 2) * std::min(dx, dy);

    return h_cost;
}

double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    double g_cost = 0.0;
    ////////////////// TODO: Implement your goal cost, use obstacle distances //////////////////////////
    g_cost = from->g_cost;

    double dx = fabs(goal->cell.x - from->cell.x);
    double dy = fabs(goal->cell.y - from->cell.y);

    g_cost += (dx + dy) + (std::sqrt(2) - 2) * std::min(dx, dy);
    
    // Add cost related to distance from obstacles
    float distanceToObstacle = distances(goal->cell.x, goal->cell.y);

    if (distanceToObstacle < params.maxDistanceWithCost) {
       g_cost += pow(params.maxDistanceWithCost / distanceToObstacle, params.distanceCostExponent);
    }

    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    std::vector<Node*> neighbors;
    ////////////////// TODO: Implement your expand node algorithm //////////////////////////
    std::vector<Point<int>> moves = {Point<int>(1, 0), Point<int>(0, 1), Point<int>(-1, 0), Point<int>(0, -1), Point<int>(-1, -1), Point<int>(-1, 1), Point<int>(1, -1), Point<int>(1, 1)};

    for (auto &move : moves) {
        int newX = node->cell.x + move.x;
        int newY = node->cell.y + move.y;

        // Skip out-of-bounds nodes or node cell in obstacles
        if (!distances.isCellInGrid(newX, newY) || 
            distances(newX, newY) < params.minDistanceToObstacle) {
            continue;
        }

        Node* neighborNode = new Node(newX, newY);
        neighbors.push_back(neighborNode);
    }
    return neighbors;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    std::vector<Node*> path;
    ////////////////// TODO: Implement your extract node function //////////////////////////
    // Traverse nodes and add parent nodes to the vector

    Node* currentNode = goal_node;

    while (currentNode != NULL) {
        path.push_back(currentNode);
        currentNode = currentNode->parent;
    }

    // Reverse path
    std::reverse(path.begin(), path.end());

    return path;
}
// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose2D_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    std::vector<mbot_lcm_msgs::pose2D_t> path;
    ////////////////// TODO: Implement your extract_pose_path function //////////////////////////
    // This should turn the node path into a vector of poses (with heading) in the global frame
    // You should prune the path to get a waypoint path suitable for sending to motion controller

    std::vector<Node*> new_node_path = prune_node_path(nodes);

    for (int i = 0; i < new_node_path.size(); i++) {

        Point<float> point = grid_position_to_global_position(new_node_path[i]->cell, distances);

        mbot_lcm_msgs::pose2D_t p;
        p.x = point.x;
        p.y = point.y;
        //p.pose.utime = 0;

        // Calculate p.theta
        if(i == new_node_path.size()-1){
            p.theta = path.back().theta;
        }
        else{
            Node* currentNode = new_node_path[i];
            Node* nextNode = new_node_path[i + 1];
    
            // Calculate heading (angle between current node and next node)
            double dx = nextNode->cell.x - currentNode->cell.x;
            double dy = nextNode->cell.y - currentNode->cell.y;
            p.theta = wrap_to_pi(std::atan2(dy, dx));  // Heading in radians
        }

        path.push_back(p);
    }
    
    return path;
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (*node == *n) return n;
    }
    return NULL;

}

std::vector<Node*> prune_node_path(std::vector<Node*> nodePath)
{
    std::vector<Node*> new_node_path;
    ////////////////// TODO: Optionally implement a prune_node_path function //////////////////////////
    // This should remove points in the path along the same line
    for (size_t i = 0; i < nodePath.size(); i++) {
        if (i == 0 || i == nodePath.size() - 1) {
            new_node_path.push_back(nodePath[i]);
        }
        else {
            // Check if the node is along a straight line with the previous and next node
            Node* prev = nodePath[i - 1];
            Node* curr = nodePath[i];
            Node* next = nodePath[i + 1];

            double dx1 = curr->cell.x - prev->cell.x;
            double dy1 = curr->cell.y - prev->cell.y;
            double dx2 = next->cell.x - curr->cell.x;
            double dy2 = next->cell.y - curr->cell.y;

            // Check if the direction vectors are aligned (cross product == 0)
            if (dx1 * dy2 - dy1 * dx2 != 0) {
                new_node_path.push_back(curr);
            }
        }
    }

    return new_node_path;

}