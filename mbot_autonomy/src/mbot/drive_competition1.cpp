#include <utils/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <mbot_lcm_msgs/path2D_t.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/twist2D_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>

using namespace mbot_lcm_msgs;

int main(int argc, char** argv)
{
    int numTimes = 2;

    std::cout << "Commanding robot to drive in competition 1 path. " << numTimes << " times.\n";

    // Choose a path (easy_path or hard_path)
    std::vector<std::vector<float>> easyPath = {
        {0.00, 0.00},
        {0.61, 0.00},
        {0.61, 0.61},
        {1.22, 0.61},
        {1.22, 0.00},
        {1.83, 0.00},
        {1.83, 1.22},
        {0.00, 1.22},
        {0.00, 0.00}
    };

    std::vector<std::vector<float>> hardPath = {
        {0.00, 0.00}, {0.30, 0.00}, {0.30, 0.00}, {0.37, 0.01}, {0.44, 0.03},
        {0.49, 0.07}, {0.54, 0.11}, {0.58, 0.17}, {0.60, 0.24}, {0.61, 0.30},
        {0.61, 0.30}, {0.62, 0.37}, {0.64, 0.44}, {0.68, 0.49}, {0.72, 0.54},
        {0.78, 0.58}, {0.85, 0.60}, {0.91, 0.61}, {0.91, 0.61}, {0.98, 0.60},
        {1.05, 0.58}, {1.10, 0.54}, {1.15, 0.49}, {1.19, 0.44}, {1.21, 0.37},
        {1.22, 0.30}, {1.22, 0.30}, {1.23, 0.24}, {1.25, 0.17}, {1.28, 0.11},
        {1.33, 0.07}, {1.39, 0.03}, {1.46, 0.01}, {1.52, 0.00}, {1.83, 0.00},
        {1.83, 0.91}, {1.83, 0.91}, {1.82, 0.98}, {1.80, 1.05}, {1.76, 1.10},
        {1.71, 1.15}, {1.66, 1.19}, {1.59, 1.21}, {1.52, 1.22}, {0.30, 1.22},
        {0.30, 1.22}, {0.24, 1.21}, {0.17, 1.19}, {0.11, 1.15}, {0.07, 1.10},
        {0.03, 1.05}, {0.01, 0.98}, {0.00, 0.91}, {0.00, 0.00}
    };

    std::vector<std::vector<float>> selectedPath;

    if(argv[1] == std::string("easy"))
    {
        std::cout << "Using easy path.\n";
        selectedPath = easyPath; // Use hardPath by default
    }
    else if(argv[1] == std::string("hard"))
    {
        std::cout << "Using hard path.\n";
        selectedPath = hardPath; // Use hardPath by default
    }
    else
    {
        std::cerr << "Invalid path choice. Defaulting to hard path.\n";

    }

    path2D_t path;
    //path.path.resize(numTimes * 1);

    pose2D_t nextPose;

    // Populate the path with waypoints
    for(int i = 0; i < numTimes; i++){
        for (const auto& waypoint : selectedPath) {
            nextPose.x = waypoint[0];
            nextPose.y = waypoint[1];
            nextPose.theta = 0.0f; // Default theta
            path.path.push_back(nextPose);
        }
    }

    path.path_length = path.path.size();

    twist2D_t maxVel;
    maxVel.vx = 0.8;
    maxVel.vy = 0.0;
    maxVel.wz = M_PI;

    lcm::LCM lcmInstance(MULTICAST_URL);
	std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    lcmInstance.publish(MBOT_MAX_VEL_CHANNEL, &maxVel);
    sleep(1);

    return 0;
}
