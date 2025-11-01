#ifndef ODOMETRY_NODE_HPP
#define ODOMETRY_NODE_HPP

#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h> 

#include "iturover_odometry_assignment/kalman_filter.hpp"

class OdometryFuser {
public:
    OdometryFuser(ros::NodeHandle &nh);
    void run();

private:
    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Subscriber left_sub_, right_sub_, imu_sub_;
    tf::TransformBroadcaster odom_broadcaster_;

    KalmanFilter kf_; 

    const double WHEEL_RADIUS = 0.135;
    const double WHEEL_SEPARATION = 0.89;

    volatile double current_left_rpm_ = 0.0;
    volatile double current_right_rpm_ = 0.0;
    volatile double imu_angular_vel_z_ = 0.0;
    volatile double imu_yaw_ = 0.0;

    ros::Time last_time_;
    bool is_initialized_ = false;
    const double UPDATE_FREQUENCY = 50.0;

    void leftWheelsCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void rightWheelsCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

    void updateFuser(); 

    void publishOdometry(const Eigen::Vector3d &state, double linear_vel, double angular_vel, const ros::Time &time);
};

#endif