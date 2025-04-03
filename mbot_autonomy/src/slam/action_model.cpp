#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.005f)
, k2_(0.025f)
, min_dist_(0.0025)
, min_theta_(0.02)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());

}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose2D_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose2D_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if(!initialized_){
        previousPose_ = odometry;
        initialized_ = true;
    }

    dx_ = odometry.x - previousPose_.x;
    dy_ = odometry.y - previousPose_.y;
    dtheta_ = odometry.theta - previousPose_.theta;

    float direction = 1.0;

    rot1_ = angle_diff(std::atan2(dy_, dx_), previousPose_.theta);
    // If robot moves backwards
    if(std::abs(rot1_) > M_PI/2.0){
        rot1_ = angle_diff(M_PI, rot1_);
        direction = -1.0;
    }

    rot2_ = angle_diff(dtheta_, rot1_);
    trans_ = direction * std::sqrt(dx_ * dx_ + dy_ * dy_);

    // Moved
    //moved_ = (dx_ != 0.0 || dy_ != 0.0 || dtheta_ != 0.0);
    moved_ = (std::abs(trans_) > min_dist_) || (dtheta_ >= min_theta_);

    if(moved_){
        rot1Std_ = std::sqrt(k1_ * std::abs(rot1_));
        rot2Std_ = std::sqrt(k1_ * std::abs(rot2_));
        transStd_ = std::sqrt(k2_ * std::abs(trans_));
        //xStd_ = std::sqrt(k2_ * std::abs(dx_));
        //yStd_ = std::sqrt(k2_ * std::abs(dy_));
        //thetaStd_ = std::sqrt(k1_ * std::abs(dtheta_));
    }

    resetPrevious(odometry);
    utime_ = odometry.utime;

    return moved_;    // Placeholder
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample = sample;

    float sampleRot1 = std::normal_distribution<>(rot1_, rot1Std_)(numberGenerator_);
    float sampleTrans = std::normal_distribution<>(trans_, transStd_)(numberGenerator_);
    float sampleRot2 = std::normal_distribution<>(rot2_, rot2Std_)(numberGenerator_);

    newSample.pose.x += sampleTrans * cos(sample.pose.theta + sampleRot1);
    newSample.pose.y += sampleTrans * sin(sample.pose.theta + sampleRot1);
    newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampleRot1 + sampleRot2);

    // newSample.pose.x += std::normal_distribution<>(dx_, xStd_)(numberGenerator_);
    // newSample.pose.y += std::normal_distribution<>(dy_, yStd_)(numberGenerator_);
    // newSample.pose.theta += std::normal_distribution<>(dtheta_, thetaStd_)(numberGenerator_);
    
    newSample.pose.utime = utime_;
    newSample.parent_pose = sample.pose;

    return newSample;
}
