#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <memory>

class OptitrackOdometrySubscriber : public rclcpp::Node
{
public:
    OptitrackOdometrySubscriber() : Node("optitrack_odometry_subscriber")
    {
        // QoS setup
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
            qos_profile
        );

        // Create subscriber for OptiTrack odometry
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/optitrackKYJLJW", qos,
            std::bind(&OptitrackOdometrySubscriber::odometry_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "OptiTrack odometry subscriber initialized");
    }

private:
    // Convert quaternion to rotation matrix
    Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Vector4d& q) {
        Eigen::Matrix3d R;
        double w = q(0), x = q(1), y = q(2), z = q(3);
        
        R(0,0) = 1 - 2*y*y - 2*z*z;
        R(0,1) = 2*x*y - 2*w*z;
        R(0,2) = 2*x*z + 2*w*y;
        
        R(1,0) = 2*x*y + 2*w*z;
        R(1,1) = 1 - 2*x*x - 2*z*z;
        R(1,2) = 2*y*z - 2*w*x;
        
        R(2,0) = 2*x*z - 2*w*y;
        R(2,1) = 2*y*z + 2*w*x;
        R(2,2) = 1 - 2*x*x - 2*y*y;
        
        return R;
    }

    // Convert rotation matrix to quaternion
    Eigen::Vector4d rotationMatrixToQuaternion(const Eigen::Matrix3d& R) {
        Eigen::Vector4d q;
        double trace = R.trace();
        
        if (trace > 0) {
            double s = 0.5 / sqrt(trace + 1.0);
            q(0) = 0.25 / s;
            q(1) = (R(2,1) - R(1,2)) * s;
            q(2) = (R(0,2) - R(2,0)) * s;
            q(3) = (R(1,0) - R(0,1)) * s;
        } else {
            if (R(0,0) > R(1,1) && R(0,0) > R(2,2)) {
                double s = 2.0 * sqrt(1.0 + R(0,0) - R(1,1) - R(2,2));
                q(0) = (R(2,1) - R(1,2)) / s;
                q(1) = 0.25 * s;
                q(2) = (R(0,1) + R(1,0)) / s;
                q(3) = (R(0,2) + R(2,0)) / s;
            } else if (R(1,1) > R(2,2)) {
                double s = 2.0 * sqrt(1.0 + R(1,1) - R(0,0) - R(2,2));
                q(0) = (R(0,2) - R(2,0)) / s;
                q(1) = (R(0,1) + R(1,0)) / s;
                q(2) = 0.25 * s;
                q(3) = (R(1,2) + R(2,1)) / s;
            } else {
                double s = 2.0 * sqrt(1.0 + R(2,2) - R(0,0) - R(1,1));
                q(0) = (R(1,0) - R(0,1)) / s;
                q(1) = (R(0,2) + R(2,0)) / s;
                q(2) = (R(1,2) + R(2,1)) / s;
                q(3) = 0.25 * s;
            }
        }
        
        return q;
    }

    // Convert NWU to NED coordinate system
    Eigen::Vector4d convertNWUtoNED(const Eigen::Vector4d& q_nwu) {
        // NWU to NED transformation matrix
        Eigen::Matrix3d T_nwu_to_ned;
        T_nwu_to_ned << 1,  0,  0,
                        0, -1,  0,
                        0,  0, -1;

        // Convert quaternion to rotation matrix
        Eigen::Matrix3d R_nwu = quaternionToRotationMatrix(q_nwu);
        
        // Transform rotation matrix from NWU to NED
        Eigen::Matrix3d R_ned = T_nwu_to_ned * R_nwu * T_nwu_to_ned;
        
        // Convert back to quaternion
        return rotationMatrixToQuaternion(R_ned);
    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract position
        position_ << msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.position.z;

        // Extract orientation (quaternion) in NWU
        orientation_nwu_ << msg->pose.pose.orientation.w,
                           msg->pose.pose.orientation.x,
                           msg->pose.pose.orientation.y,
                           msg->pose.pose.orientation.z;

        // Convert orientation from NWU to NED
        orientation_ned_ = convertNWUtoNED(orientation_nwu_);

        // Extract linear velocity
        velocity_ << msg->twist.twist.linear.x,
                    msg->twist.twist.linear.y,
                    msg->twist.twist.linear.z;

        // Extract angular velocity
        angular_velocity_ << msg->twist.twist.angular.x,
                           msg->twist.twist.angular.y,
                           msg->twist.twist.angular.z;

        // Log the received data (throttled to avoid flooding)
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Position: [%.2f, %.2f, %.2f], Velocity: [%.2f, %.2f, %.2f]\n"
            "NWU Quaternion: [%.2f, %.2f, %.2f, %.2f]\n"
            "NED Quaternion: [%.2f, %.2f, %.2f, %.2f]",
            position_(0), position_(1), position_(2),
            velocity_(0), velocity_(1), velocity_(2),
            orientation_nwu_(0), orientation_nwu_(1), orientation_nwu_(2), orientation_nwu_(3),
            orientation_ned_(0), orientation_ned_(1), orientation_ned_(2), orientation_ned_(3));
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    
    // State variables
    Eigen::Vector3d position_;
    Eigen::Vector4d orientation_nwu_;  // NWU coordinate system
    Eigen::Vector4d orientation_ned_;  // NED coordinate system
    Eigen::Vector3d velocity_;
    Eigen::Vector3d angular_velocity_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OptitrackOdometrySubscriber>());
    rclcpp::shutdown();
    return 0;
} 