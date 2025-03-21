#include "mbot_odometry.h"

int mbot_calculate_odometry(serial_twist2D_t mbot_vel, float dt, serial_pose2D_t *odometry, serial_mbot_imu_t *imu){
    float vx_space = mbot_vel.vx * cos(odometry->theta) - mbot_vel.vy * sin(odometry->theta);
    float vy_space = mbot_vel.vx * sin(odometry->theta) + mbot_vel.vy * cos(odometry->theta);

    odometry->x += vx_space * dt;
    odometry->y += vy_space * dt;
    //odometry->theta += mbot_vel.wz * dt;
    odometry->theta = imu->angles_rpy[2];

    // Normalize theta to be between -pi and pi
    while (odometry->theta > M_PI) odometry->theta -= 2 * M_PI;
    while (odometry->theta <= -M_PI) odometry->theta += 2 * M_PI;

    return 0; // Return 0 to indicate success
}
