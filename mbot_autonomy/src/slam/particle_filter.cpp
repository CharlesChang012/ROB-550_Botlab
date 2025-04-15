#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>
#include <utils/geometric/angle_functions.hpp>
#include <ctime>
#include <chrono>

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  samplingAugmentation_(0.5, 0.9, numParticles),
  distribution_quality(1),
  quality_reinvigoration_percentage(0.1)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose2D_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleWeight = 1.0/kNumParticles_;
    posteriorPose_ = pose;

    for(auto &p : posterior_){
        p.pose.x = posteriorPose_.x;
        p.pose.y = posteriorPose_.y;
        p.pose.theta = wrap_to_pi(posteriorPose_.theta);
        p.weight = sampleWeight;
        p.pose.utime =  pose.utime;
        p.parent_pose = p.pose;
    }

}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    printf("Randomly Initialize particles\n");
    
    std::random_device rd;
    std::mt19937 generator(rd());

    // Random number generation for the range of x, y, and theta values
    std::uniform_real_distribution<> x_dist(-map.widthInMeters()/2, map.widthInMeters()/2);  // x within map bounds 
    std::uniform_real_distribution<> y_dist(-map.heightInMeters()/2, map.heightInMeters()/2);  // y within map bounds 
    std::uniform_real_distribution<> theta_dist(-M_PI, M_PI);  // theta in the range [-pi, pi]

    double sampleWeight = 1.0/kNumParticles_;

    int64_t cur_time = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();

    for (auto &p : posterior_) {
        while(true){
            p.pose.x = x_dist(generator);  // Random x position
            p.pose.y = y_dist(generator);  // Random y position
            p.pose.theta = wrap_to_pi(theta_dist(generator));
            Point<int> cell = global_position_to_grid_cell(Point<double>(p.pose.x, p.pose.y), map);
            if (map.isCellInGrid(cell.x, cell.y) && map.logOdds(cell.x, cell.y) < 0) {
                break;  // Valid pose in free space
            }
        }

        p.weight = sampleWeight;  // Initialize with equal weight
        p.pose.utime = cur_time;
        p.parent_pose = p.pose;
    }

}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose2D_t& odometry)
{
    actionModel_.resetPrevious(odometry);
}


mbot_lcm_msgs::pose2D_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose2D_t& odometry,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    // printf("Updating Particle Filter...\n");
    // auto start_time = std::chrono::high_resolution_clock::now();

    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    auto prior = resamplePosteriorDistribution(map);
    auto proposal = computeProposalDistribution(prior);
    posterior_ = computeNormalizedPosterior(proposal, laser, map);
    /// TODO: Add reinvigoration step
    posteriorPose_ = estimatePosteriorPose(posterior_);

    posteriorPose_.utime = odometry.utime;

    // auto end_time = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    // printf("Duration for PF to update: %f millisecond\n", static_cast<double>(duration));

    return posteriorPose_;
}

mbot_lcm_msgs::pose2D_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose2D_t& odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = proposal;
    }

    posteriorPose_ = odometry;

    return posteriorPose_;
}


mbot_lcm_msgs::pose2D_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid& map,
                                                           const bool keep_best,
                                                           const bool reinvigorate)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    
    /*double sampleWeight = 1.0/kNumParticles_;

    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<> dist(0.0, 0.04);

    for (auto& p : prior) {
        
        Point<int> pCell;
        do{
            p.pose.x = posteriorPose_.x + dist(generator);  // Random x position
            p.pose.y = posteriorPose_.y + dist(generator);  // Random y position
            Point<double> p_point(p.pose.x, p.pose.y);
            pCell = global_position_to_grid_cell(p_point, map);

        }while(!map.isCellInGrid(pCell.x, pCell.y));

        p.pose.theta = posteriorPose_.theta + dist(generator);  // Random theta orientation
        p.weight = sampleWeight;  // Initialize with equal weight
        p.pose.utime = posteriorPose_.utime;
        p.parent_pose = posteriorPose_;
    }*/
    ParticleList prior = posterior_;

    if(keep_best){
        prior = importanceSample(kNumParticles_, prior);
    }
    else{
        prior = lowVarianceSample(kNumParticles_, prior);
    }
    
    if(reinvigorate){
        reinvigoratePriorDistribution(prior);
    }

    /*
    calculateDistributionQuality(prior);
    
    */

    return prior;  // Placeholder
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const bool keep_best,
                                                           const bool reinvigorate)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    /*
    ParticleList prior = posterior_;
    double sampleWeight = 1.0/kNumParticles_;

    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<> dist(0.0, 0.04);

    for(auto &p: prior){
        p.pose.x = posteriorPose_.x + dist(generator);
        p.pose.y = posteriorPose_.y + dist(generator);
        p.pose.theta = posteriorPose_.theta + dist(generator);
        p.weight = sampleWeight;
        p.pose.utime =  posteriorPose_.utime;
        p.parent_pose = posteriorPose_;
    }
    */
    ParticleList prior = posterior_;

    if(keep_best){
        prior = importanceSample(kNumParticles_, prior);
    }
    else{
        prior = lowVarianceSample(kNumParticles_, prior);
    }
    
    if(reinvigorate){
        reinvigoratePriorDistribution(prior);
    }
  
    return prior;  // Placeholder
}


void ParticleFilter::reinvigoratePriorDistribution(ParticleList& prior)
{
    // Augmentation: if sensor model suspects an average particle quality of
    //      less than 15%, invigorate
    if (distribution_quality < 0.15)  // TODO: make 0.15 a parameter
    {
        int count = 0;
        int max_count = floor(quality_reinvigoration_percentage * prior.size());

        std::random_device rd;
        std::default_random_engine generator(rd());
        auto ud01 = std::uniform_real_distribution<double>(0.0, 1.0);
        int step = std::max<int>(1, floor(ud01(generator) * prior.size() / max_count));

        for (int i = 0; i < max_count; i++)
        {
            prior[i*step] = randomPoseGen_.get_particle();
        }

    }

    // // Augmentation: randomize any unreasonable samples
    // if(map != nullptr)
    // {
    //     for (int i = 0; i < prior.size(); i++)
    //     {
    //         const auto& p = prior[i].pose;
    //         if(!map->isCellInGrid(p.x, p.y) ||
    //           (map->isCellInGrid(p.x, p.y) && (map->logOdds(p.x, p.y) >= 0)))
    //         {
    //             std::cout << "\tinvalid sample!!" << ", "
    //                 << !map->isCellInGrid(p.x, p.y) << ", "
    //                 << (map->isCellInGrid(p.x, p.y) && (map->logOdds(p.x, p.y) >= 0.0))
    //                 << std::endl;


    //             prior[i] = randomPoseGen_.get_particle();
    //         }
    //     }
    // }
}


ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    ParticleList proposal;
    for(auto &p : prior){
        proposal.push_back(actionModel_.applyAction(p));
    }
    
    return proposal;  // Placeholder
}


ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList& proposal,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution
    ParticleList posterior;
    double weightSum = 0.0;

    for(auto &p : proposal){
        mbot_lcm_msgs::particle_t p_weighted = p;
        p_weighted.weight = sensorModel_.likelihood(p_weighted, laser, map);
        weightSum += p_weighted.weight;
        posterior.push_back(p_weighted);
    }

    for(auto &p : posterior){
        p.weight /= weightSum;
    }
    
    return posterior;  // Placeholder
}


mbot_lcm_msgs::pose2D_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    // Figure out which pose to take for the posterior pose
    // Weighted average is simple, but could be very bad
    // Maybe only take the best x% and then average.

    ParticleList sortedPosterior = getSortedParticlesByWeight(posterior);

    ParticleList bestPosterior(sortedPosterior.begin(), sortedPosterior.begin() + sortedPosterior.size() * 0.5);

    mbot_lcm_msgs::pose2D_t pose;

    pose = computeParticlesAverage(bestPosterior);

    return pose;
}

mbot_lcm_msgs::pose2D_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    mbot_lcm_msgs::pose2D_t avg_pose;
    avg_pose.x = 0.0;
    avg_pose.y = 0.0;
    avg_pose.theta = 0.0;
    double sum_weight = 0.0;

    // Aux variables to compute theta average
    double theta_x = 0.0;
    double theta_y = 0.0;
    for (auto &&p : particles_to_average)
    {
        avg_pose.x += p.weight * p.pose.x;
        avg_pose.y += p.weight * p.pose.y;
        theta_x += p.weight * std::cos(p.pose.theta);
        theta_y += p.weight * std::sin(p.pose.theta);

        sum_weight += p.weight;
    }
    avg_pose.x /= sum_weight;
    avg_pose.y /= sum_weight;
    theta_x /= sum_weight;
    theta_y /= sum_weight;
    avg_pose.theta = std::atan2(theta_y, theta_x);

    return avg_pose;
}

/*---------------------- User Defined Functions -------------------*/
// Function to sort ParticleList by weight in descending order
ParticleList ParticleFilter::getSortedParticlesByWeight(const ParticleList& particles) {
    ParticleList sorted_particles = particles;  // Copy the original list

    std::sort(sorted_particles.begin(), sorted_particles.end(), [](const mbot_lcm_msgs::particle_t& a, const mbot_lcm_msgs::particle_t& b) {
        return a.weight > b.weight;  // Sort in descending order by weight
    });

    return sorted_particles;  // Return the sorted list
}

void ParticleFilter::calculateDistributionQuality(const ParticleList& particles){
    double weightSum = 0.0;
    double weightSquaredSum = 0.0;

    for (const auto& p : particles) {
        weightSum += p.weight;
        weightSquaredSum += p.weight * p.weight;
    }

    double meanWeight = weightSum / particles.size();

    double weightVariance = (weightSquaredSum / particles.size()) - (meanWeight * meanWeight);
    double maxVariance = 0.25;  // Example value; this could depend on your application

    // Normalize the variance
    double distribution_quality = 1.0 - (weightVariance / maxVariance);
    
    // Ensure the value is between 0 and 1
    if(distribution_quality < 0.0) distribution_quality = 0.0;
    else if(distribution_quality > 1.0) distribution_quality = 1.0;
  
}