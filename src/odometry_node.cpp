#include "iturover_odometry_assignment/odometry_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry_fuser_node");
    ros::NodeHandle nh;
    OdometryFuser fuser(nh);
    fuser.run();
    return 0;
}

OdometryFuser::OdometryFuser(ros::NodeHandle &nh) : nh_(nh) {
    left_sub_ = nh_.subscribe("/drive_system_left_motors_feedbacks", 10, &OdometryFuser::leftWheelsCallback, this);
    right_sub_ = nh_.subscribe("/drive_system_right_motors_feedbacks", 10, &OdometryFuser::rightWheelsCallback, this);
    imu_sub_ = nh_.subscribe("/imu1/data", 10, &OdometryFuser::imuCallback, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 50);
}

void OdometryFuser::leftWheelsCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    current_left_rpm_ = (msg->data[0] + msg->data[1]) / 2.0;
}

void OdometryFuser::rightWheelsCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
	current_right_rpm_ = (msg->data[0] + msg->data[1]) / 2.0;
}

void OdometryFuser::imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    imu_angular_vel_z_ = msg->angular_velocity.z;
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    imu_yaw_ = yaw;
}

void OdometryFuser::updateFuser() {
    ros::Time current_time = ros::Time::now();
    if (!is_initialized_)
	{
        last_time_ = current_time;
        is_initialized_ = true;
        return;
    }
    double dt = (current_time - last_time_).toSec();
    if (dt == 0.0) return;
    double left_vel = (current_left_rpm_ * 2.0 * M_PI / 60.0) * WHEEL_RADIUS;
    double right_vel = (current_right_rpm_ * 2.0 * M_PI / 60.0) * WHEEL_RADIUS;
    double linear_vel = (right_vel + left_vel) / 2.0;
    double angular_vel = (right_vel - left_vel) / WHEEL_SEPARATION;
    kf_.predict(linear_vel, angular_vel, dt);
    Eigen::VectorXd Z(2);
    Z(0) = imu_yaw_;
    Z(1) = imu_angular_vel_z_;
    kf_.update(Z);
    Eigen::Vector3d state = kf_.getState();
    publishOdometry(state, linear_vel, angular_vel, current_time);
    last_time_ = current_time;
}

void OdometryFuser::run() {
    ros::Rate rate(UPDATE_FREQUENCY);
    while (ros::ok())
	{
        updateFuser();
        ros::spinOnce(); 
        rate.sleep();
    }
}

void OdometryFuser::publishOdometry(const Eigen::Vector3d& state, double linear_vel, double angular_vel, const ros::Time& time) {
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state(2));
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = state(0);
    odom_trans.transform.translation.y = state(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster_.sendTransform(odom_trans);
    nav_msgs::Odometry odom;
    odom.header.stamp = time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = state(0);
    odom.pose.pose.position.y = state(1);
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = linear_vel;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = angular_vel;
    Eigen::Matrix3d P_kalman = kf_.getCovariance();
    for(int i = 0; i < 36; i++)
	{
        odom.pose.covariance[i] = 0.0;
        odom.twist.covariance[i] = 0.0;
    }
    odom.pose.covariance[0] = P_kalman(0, 0);
    odom.pose.covariance[7] = P_kalman(1, 1);
    odom.pose.covariance[35] = P_kalman(2, 2);
    odom.pose.covariance[1] = P_kalman(0, 1);
    odom.pose.covariance[6] = P_kalman(1, 0);
    odom_pub_.publish(odom);
}
