#include <planning/exploration.hpp>
#include <planning/frontiers.hpp>
#include <utils/lcm_config.h>
#include <utils/grid_utils.hpp>
#include <utils/timestamp.h>
#include <mbot/mbot_channels.h>
#include <slam/slam_channels.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <unistd.h>
#include <cassert>


const float kReachedPositionThreshold = 0.05f;  // must get within this distance of a position for it to be explored

// Define an equality operator for poses to allow direct comparison of two paths
bool are_equal(const mbot_lcm_msgs::pose2D_t& lhs, const mbot_lcm_msgs::pose2D_t& rhs)
{
    return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.theta == rhs.theta);
}
using pose_t = mbot_lcm_msgs::pose2D_t;
using pose_vec_t = std::vector<pose_t>;
bool are_equal(const pose_vec_t& lhs, const pose_vec_t& rhs)
{
    if(lhs.size() != rhs.size()) return false;
    for (int i = 0; i < rhs.size(); i++) {
        if (!are_equal(lhs[i], rhs[i])) return false;
    }
    return true;
}


Exploration::Exploration(lcm::LCM* lcmInstance)
: state_(mbot_lcm_msgs::exploration_status_t::STATE_INITIALIZING)
, haveNewPose_(false)
, haveNewMap_(false)
, haveHomePose_(false)
, lcmInstance_(lcmInstance)
, pathReceived_(false)
{
    assert(lcmInstance_);   // confirm a nullptr wasn't passed in

    lcmInstance_->subscribe(SLAM_MAP_CHANNEL, &Exploration::handleMap, this);
    lcmInstance_->subscribe(SLAM_POSE_CHANNEL, &Exploration::handlePose, this);
    lcmInstance_->subscribe(MESSAGE_CONFIRMATION_CHANNEL, &Exploration::handleConfirmation, this);

    /* ----------- Cone Detection explorer ------------*/
    lcmInstance_->subscribe("MBOT_CONE_ARRAY", &Exploration::handleCone, this);

    // Send an initial message indicating that the exploration module is initializing. Once the first map and pose are
    // received, then it will change to the exploring map state.
    mbot_lcm_msgs::exploration_status_t status;
    status.utime = utime_now();
    status.state = mbot_lcm_msgs::exploration_status_t::STATE_INITIALIZING;
    status.status = mbot_lcm_msgs::exploration_status_t::STATUS_IN_PROGRESS;

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);

    MotionPlannerParams params;
    params.robotRadius = 0.2;
    planner_.setParams(params);

    // To prevent the exploration finishing on the start
    initialized_ = false;
}


bool Exploration::exploreEnvironment()
{
    while((state_ != mbot_lcm_msgs::exploration_status_t::STATE_COMPLETED_EXPLORATION)
        && (state_ != mbot_lcm_msgs::exploration_status_t::STATE_FAILED_EXPLORATION))
    {
        // If data is ready, then run an update of the exploration routine
        if(isReadyToUpdate())
        {
            runExploration();
        }
        // Otherwise wait a bit for data to arrive
        else
        {
            usleep(10000);
        }
    }

    // If the state is completed, then we didn't fail
    return state_ == mbot_lcm_msgs::exploration_status_t::STATE_COMPLETED_EXPLORATION;
}

void Exploration::handleMap(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::occupancy_grid_t* map)
{

    std::lock_guard<std::mutex> autoLock(dataLock_);
    incomingMap_.fromLCM(*map);
    haveNewMap_ = true;
}


void Exploration::handlePose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::pose2D_t* pose)
{

    std::lock_guard<std::mutex> autoLock(dataLock_);
    incomingPose_ = *pose;
    haveNewPose_ = true;
}

void Exploration::handleConfirmation(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::mbot_message_received_t* confirm)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    if(confirm->channel == CONTROLLER_PATH_CHANNEL && confirm->creation_time == most_recent_path_time) pathReceived_ = true;
}

void Exploration::handleCone(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::mbot_cone_array_t* cone_array)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);

    if (cone_array->array_size == 0) {
        //printf("no cone detected\n");
    }
    else{
        for (auto& detection : cone_array->detections) {
            bool found = false;
            for (auto& currentCone : currentConeArray_) {
                if (detection.name == currentCone.name) {
                    found = true;
                    break;
                }
            }
    
            if (!found) {
                mbot_lcm_msgs::mbot_cone_t coneWorldFrame = camera2WorldFrame(detection);
                currentConeArray_.push_back(coneWorldFrame);
                planner_.updateConeDistance(coneWorldFrame);
                printf("Cone detected: %s\n", detection.name.c_str());
                printf("Cone pose at world frame: %f, %f\n", coneWorldFrame.pose.x, coneWorldFrame.pose.y);
            }
        }
    }


    // if(!currentConeArray_.empty()){
    //     printf("Current position: (%f, %f)\n", currentPose_.x, currentPose_.y);
    //     printf("Listing all cone detected\n");
    //     for(const auto& cone : currentConeArray_)
    //     {
    //         printf("Cone position: (%f, %f)\n", cone.pose.x, cone.pose.y);
    //     }
    // }

}

ConeColor Exploration::string2ConeColor(const std::string& name)
{
    if (name == "red_cone")
        return ConeColor::RED;
    else if (name == "pink_cone")
        return ConeColor::PINK;
    else if (name == "yellow_cone")
        return ConeColor::YELLOW;
    else if (name == "green_cone")
        return ConeColor::GREEN;
    else if (name == "blue_cone")
        return ConeColor::BLUE;
}

bool Exploration::isReadyToUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    return haveNewMap_ && haveNewPose_;
}


void Exploration::runExploration(void)
{
    assert(isReadyToUpdate());
    copyDataForUpdate();
    executeStateMachine();
}


void Exploration::copyDataForUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);

    // Only copy the map if a new one has arrived because it is a costly operation
    if(haveNewMap_)
    {
        currentMap_ = incomingMap_;
        haveNewMap_ = false;
        // Update planner
        planner_.setMap(currentMap_);
    }

    // Always copy the pose because it is a cheap copy
    currentPose_ = incomingPose_;
    haveNewPose_ = false;

    // The first pose received is considered to be the home pose
    if(!haveHomePose_)
    {
        homePose_ = incomingPose_;
        haveHomePose_ = true;
    }
}


void Exploration::executeStateMachine(void)
{
    bool stateChanged = false;
    int8_t nextState = state_;

    // Save the path from the previous iteration to determine if a new path was created and needs to be published
    mbot_lcm_msgs::path2D_t previousPath = currentPath_;

    // Run the state machine until the state remains the same after an iteration of the loop
    do
    {
        switch(state_)
        {
            case mbot_lcm_msgs::exploration_status_t::STATE_INITIALIZING:
                nextState = executeInitializing();
                break;
            case mbot_lcm_msgs::exploration_status_t::STATE_EXPLORING_MAP:
                nextState = executeExploringMap(stateChanged);
                break;

            case mbot_lcm_msgs::exploration_status_t::STATE_RETURNING_HOME:
                nextState = executeReturningHome(stateChanged);
                break;

            case mbot_lcm_msgs::exploration_status_t::STATE_COMPLETED_EXPLORATION:
                nextState = executeCompleted(stateChanged);
                break;

            case mbot_lcm_msgs::exploration_status_t::STATE_FAILED_EXPLORATION:
                nextState = executeFailed(stateChanged);
                break;

            case mbot_lcm_msgs::exploration_status_t::STATE_HEADING_TO_CONES:
                nextState = executeHeadingToCones(stateChanged);
                break;
        }

        stateChanged = nextState != state_;
        state_ = nextState;

    } while(stateChanged);

    //if path confirmation was not received, resend path
    if(!pathReceived_)
    {
        if (currentPath_.path_length > 0)
            lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);
    }

    //if path changed, send current path
    if(!are_equal(previousPath.path, currentPath_.path))
    {

        if (currentPath_.path_length > 0)
            lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);

        pathReceived_ = false;
        most_recent_path_time = currentPath_.utime;
    }

}


int8_t Exploration::executeInitializing(void)
{
    // Create the status message
    // Immediately transition to exploring once the first bit of data has arrived
    mbot_lcm_msgs::exploration_status_t status;
    status.utime = utime_now();
    status.state = mbot_lcm_msgs::exploration_status_t::STATE_INITIALIZING;
    status.status = mbot_lcm_msgs::exploration_status_t::STATUS_COMPLETE;

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);

    return mbot_lcm_msgs::exploration_status_t::STATE_EXPLORING_MAP;
}


int8_t Exploration::executeExploringMap(bool initialize)
{
    //////////////////////// TODO: Implement your method for exploring the map ///////////////////////////
    /*
    * NOTES:
    *   - At the end of each iteration, then (1) or (2) must hold, otherwise exploration is considered to have failed:
    *       (1) frontiers_.empty() == true      : all frontiers have been explored as determined by find_map_frontiers()
    *       (2) currentPath_.path_length > 1 : currently following a path to the next frontier
    *
    *   - Use the provided function find_map_frontiers() to find all frontiers in the map.
    *   - You will need to implement logic to select which frontier to explore.
    *   - You will need to implement logic to decide when to select a new frontier to explore. Take into consideration:
    *       -- The map is evolving as you drive, so what previously looked like a safe path might not be once you have
    *           explored more of the map.
    *       -- You will likely be able to see the frontier before actually reaching the end of the path leading to it.
    */

    if (initialize)
    {
        // Reset exploration state if initializing
        frontiers_.clear();
        currentPath_.path.clear();
        currentPath_.path_length = 0;
        printf("Exploration frontier re-initialized\n");
    }

    // If the current path is empty or the robot has reached the end of the path, update frontiers
    if (currentPath_.path.empty() || distance_between_points(Point<float>(currentPose_.x, currentPose_.y),
                                                             Point<float>(currentPath_.path.back().x, currentPath_.path.back().y)) < kReachedPositionThreshold)
    {
        printf("Updating frontiers\n");
        frontiers_ = find_map_frontiers(currentMap_, currentPose_);

        frontier_processing_t cur_frontier_processing_t = plan_path_to_frontier(frontiers_, currentPose_, currentMap_, planner_);
        currentPath_ = cur_frontier_processing_t.path_selected;

        usleep(2000000);
    }

    // Create the status message
    mbot_lcm_msgs::exploration_status_t status;

    if (!frontiers_.empty() && currentPath_.path_length > 1)
    { // In progress: frontiers left, and there's a valid path
        status.utime = utime_now();
        status.state = mbot_lcm_msgs::exploration_status_t::STATE_EXPLORING_MAP;
        status.status = mbot_lcm_msgs::exploration_status_t::STATUS_IN_PROGRESS;
    }
    else if ((!frontiers_.empty() && currentPath_.path_length <= 1) || frontiers_.empty())
    { // Failed: frontiers left, but no valid path
        status.utime = utime_now();
        status.state = mbot_lcm_msgs::exploration_status_t::STATE_EXPLORING_MAP;
        status.status = mbot_lcm_msgs::exploration_status_t::STATUS_COMPLETE;
    }
    /*else
    { // Complete: no frontiers left
        status.utime = utime_now();
        status.state = mbot_lcm_msgs::exploration_status_t::STATE_EXPLORING_MAP;
        status.status = mbot_lcm_msgs::exploration_status_t::STATUS_COMPLETE;
    }*/

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);

    // Determine the next state
    switch (status.status)
    {
    // Don't change states if we're still a work-in-progress
    case mbot_lcm_msgs::exploration_status_t::STATUS_IN_PROGRESS:
        return mbot_lcm_msgs::exploration_status_t::STATE_EXPLORING_MAP;

    // If exploration is completed, then head home (Original Code)
    // case mbot_lcm_msgs::exploration_status_t::STATUS_COMPLETE:
    //     return mbot_lcm_msgs::exploration_status_t::STATE_RETURNING_HOME;

    // If exploration is completed, then go to cones by color (For competition)
    case mbot_lcm_msgs::exploration_status_t::STATUS_COMPLETE:
        return mbot_lcm_msgs::exploration_status_t::STATE_HEADING_TO_CONES;

    // If something has gone wrong and we can't reach all frontiers, then fail the exploration.
    case mbot_lcm_msgs::exploration_status_t::STATUS_FAILED:
        return mbot_lcm_msgs::exploration_status_t::STATE_FAILED_EXPLORATION;

    default:
        std::cerr << "ERROR: Exploration::executeExploringMap: Set an invalid exploration status. Exploration failed!";
        return mbot_lcm_msgs::exploration_status_t::STATE_FAILED_EXPLORATION;
    }
}


int8_t Exploration::executeReturningHome(bool initialize)
{
    printf("Returning home\n");

    //////////////////////// TODO: Implement your method for returning to the home pose ///////////////////////////
    /*
    * NOTES:
    *   - At the end of each iteration, then (1) or (2) must hold, otherwise exploration is considered to have failed:
    *       (1) dist(currentPose_, targetPose_) < kReachedPositionThreshold  :  reached the home pose
    *       (2) currentPath_.path_length > 1  :  currently following a path to the home pose
    */

    if (initialize)
    {
        // Reset the current path if initializing
        currentPath_.path.clear();
        currentPath_.path_length = 0;
        printf("Returning home re-initialized\n");
    }

    currentPath_ = planner_.planPath(currentPose_, homePose_);
    //printf("Planned path to home\n");

    // Create the status message
    mbot_lcm_msgs::exploration_status_t status;
    status.utime = utime_now();
    status.state = mbot_lcm_msgs::exploration_status_t::STATE_RETURNING_HOME;

    double distToHome = distance_between_points(Point<float>(homePose_.x, homePose_.y),
                                                Point<float>(currentPose_.x, currentPose_.y));

    //printf("distToHome : %f\n", distToHome);

    // If we're within the threshold of home, then we're done.
    if(distToHome <= kReachedPositionThreshold)
    {
        status.status = mbot_lcm_msgs::exploration_status_t::STATUS_COMPLETE;
    }
    // Otherwise, if there's a path, then keep following it
    else if(currentPath_.path.size() > 1)
    {
        status.status = mbot_lcm_msgs::exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Else, there's no valid path to follow and we aren't home, so we have failed.
    else
    {
        // printf("Failed 1\n");
        status.status = mbot_lcm_msgs::exploration_status_t::STATUS_FAILED;
    }

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);

    // Determine the next state
    if(status.status == mbot_lcm_msgs::exploration_status_t::STATUS_IN_PROGRESS)
    {
        return mbot_lcm_msgs::exploration_status_t::STATE_RETURNING_HOME;
    }
    else if(status.status == mbot_lcm_msgs::exploration_status_t::STATUS_FAILED)
    {
        // printf("Failed\n");
        return mbot_lcm_msgs::exploration_status_t::STATE_FAILED_EXPLORATION;
    }
    else return mbot_lcm_msgs::exploration_status_t::STATE_COMPLETED_EXPLORATION;
}

int8_t Exploration::executeCompleted(bool initialize)
{
    // Stay in the completed state forever because exploration only explores a single map.
    mbot_lcm_msgs::exploration_status_t msg;
    msg.utime = utime_now();
    msg.state = mbot_lcm_msgs::exploration_status_t::STATE_COMPLETED_EXPLORATION;
    msg.status = mbot_lcm_msgs::exploration_status_t::STATUS_COMPLETE;

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);

    return mbot_lcm_msgs::exploration_status_t::STATE_COMPLETED_EXPLORATION;
}


int8_t Exploration::executeFailed(bool initialize)
{
    // Send the execute failed forever. There is no way to recover.
    mbot_lcm_msgs::exploration_status_t msg;
    msg.utime = utime_now();
    msg.state = mbot_lcm_msgs::exploration_status_t::STATE_FAILED_EXPLORATION;
    msg.status = mbot_lcm_msgs::exploration_status_t::STATUS_FAILED;

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);

    return mbot_lcm_msgs::exploration_status_t::STATE_FAILED_EXPLORATION;
}

/* --------------- User Defined for Competition ---------------*/
mbot_lcm_msgs::mbot_cone_t Exploration::camera2WorldFrame(mbot_lcm_msgs::mbot_cone_t cone)
{
    mbot_lcm_msgs::mbot_cone_t coneWorldFrame = cone;

    // Transform cone position from camera frame to world frame
    float coneXCamera = cone.pose.z / 1000.0; // Convert from mm to meters
    float coneYCamera = -cone.pose.x / 1000.0; // Convert from mm to meters

    // Apply rotation and translation to transform to world frame
    float cosTheta = cos(currentPose_.theta);
    float sinTheta = sin(currentPose_.theta);

    coneWorldFrame.pose.x = currentPose_.x + (coneXCamera * cosTheta - coneYCamera * sinTheta);
    coneWorldFrame.pose.y = currentPose_.y + (coneXCamera * sinTheta + coneYCamera * cosTheta);

    return coneWorldFrame;
}

int8_t Exploration::executeHeadingToCones(bool initialize)
{
    

    //////////////////////// TODO: Implement your method for returning to the cone pose ///////////////////////////

    if (initialize)
    {
        // Reset the current path if initializing
        currentPath_.path.clear();
        currentPath_.path_length = 0;

        // Sort cones by color
        HeadingConeArray_ = currentConeArray_;
        std::sort(HeadingConeArray_.begin(), HeadingConeArray_.end(), [&](const mbot_lcm_msgs::mbot_cone_t& cone1, const mbot_lcm_msgs::mbot_cone_t& cone2) {
            ConeColor color1 = string2ConeColor(cone1.name);
            ConeColor color2 = string2ConeColor(cone2.name);
            return color1 < color2;
        });
        printf("Heading to Cones Initialized\n");

        for(auto &cone : HeadingConeArray_){
            printf("Cone positions: (%f, %f)\n", cone.pose.x, cone.pose.y);
        }
    }

    if((distance_between_points(Point<float>(currentPose_.x, currentPose_.y),
        Point<float>(currentPath_.path.back().x, currentPath_.path.back().y)) < kReachedPositionThreshold)){

        printf("Cone reached!\n");
        currentPath_.path.clear();
        currentPath_.path_length = 0;
    }

    if((currentPath_.path.empty() || (distance_between_points(Point<float>(currentPose_.x, currentPose_.y),
        Point<float>(currentPath_.path.back().x, currentPath_.path.back().y)) < kReachedPositionThreshold)) 
        && !HeadingConeArray_.empty()){
        // Heading to first cone
        mbot_lcm_msgs::mbot_cone_t currentCone_ = HeadingConeArray_.front();
        HeadingConeArray_.erase(HeadingConeArray_.begin());

        mbot_lcm_msgs::pose2D_t conePose;
        conePose.x = currentCone_.pose.x;
        conePose.y = currentCone_.pose.y;
        printf("Current Position: (%f, %f)\n", currentPose_.x, currentPose_.y);
        printf("Cone Position: (%f, %f)\n", conePose.x, conePose.y);
        planner_.updateConeDistanceBack(currentCone_);
        currentPath_ = planner_.planPath(currentPose_, conePose);
    }


    // Create the status message
    mbot_lcm_msgs::exploration_status_t status;

    if (currentPath_.path_length > 1)
    { // In progress: cones left, and there's a valid path
        status.utime = utime_now();
        status.state = mbot_lcm_msgs::exploration_status_t::STATE_HEADING_TO_CONES;
        status.status = mbot_lcm_msgs::exploration_status_t::STATUS_IN_PROGRESS;
    }
    else if (currentPath_.path_length <= 1 && !HeadingConeArray_.empty())
    { // Failed: cones left, but no valid path
        status.utime = utime_now();
        status.state = mbot_lcm_msgs::exploration_status_t::STATE_HEADING_TO_CONES;
        status.status = mbot_lcm_msgs::exploration_status_t::STATUS_FAILED;
    }
    else if (HeadingConeArray_.empty())
    { // Completed: no cones left
        status.utime = utime_now();
        status.state = mbot_lcm_msgs::exploration_status_t::STATE_HEADING_TO_CONES;
        status.status = mbot_lcm_msgs::exploration_status_t::STATUS_COMPLETE;
    }

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);

    // Determine the next state
    switch (status.status)
    {
        // Don't change states if we're still a work-in-progress
        case mbot_lcm_msgs::exploration_status_t::STATUS_IN_PROGRESS:
            return mbot_lcm_msgs::exploration_status_t::STATE_HEADING_TO_CONES;

        // If exploration is completed, then head home (Original Code)
        case mbot_lcm_msgs::exploration_status_t::STATUS_COMPLETE:
            return mbot_lcm_msgs::exploration_status_t::STATE_RETURNING_HOME;

        // If something has gone wrong and we can't reach all frontiers, then fail the exploration.
        case mbot_lcm_msgs::exploration_status_t::STATUS_FAILED:
            return mbot_lcm_msgs::exploration_status_t::STATE_FAILED_HEADING_TO_CONES;

        default:
            std::cerr << "ERROR: Exploration::executeHeadingToCones: Set an invalid exploration status. Heading to cone failed!";
            return mbot_lcm_msgs::exploration_status_t::STATE_FAILED_HEADING_TO_CONES;
    }
}