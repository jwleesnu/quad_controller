#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <Eigen/Dense>
#include <memory>

class RPYPublisher : public rclcpp::Node
{
public:
    RPYPublisher() : Node("rpy_publisher")
    {
        // QoS setup
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.reliable();
        qos.durability_volatile();

        // Create publisher for RPY values
        publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/rpy_values", qos);

        // Create timer to publish at fixed rate (e.g., 100Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 100Hz
            std::bind(&RPYPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "RPY publisher initialized");
    }

private:
    void timer_callback()
    {
        auto msg = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
        
        // Set header
        msg->header.stamp = this->now();
        msg->header.frame_id = "base_link";  // or your desired frame_id

        // Set RPY values (in radians)
        msg->vector.x = roll_;    // Roll
        msg->vector.y = pitch_;   // Pitch
        msg->vector.z = yaw_;     // Yaw

        // Publish the message
        publisher_->publish(*msg);

        // Log the published values (throttled to avoid flooding)
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Published RPY: [%.2f, %.2f, %.2f] (rad)",
            roll_, pitch_, yaw_);
    }

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Example RPY values (in radians)
    double roll_ = 0.0;    // Roll angle
    double pitch_ = 0.0;   // Pitch angle
    double yaw_ = 0.0;     // Yaw angle
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RPYPublisher>());
    rclcpp::shutdown();
    return 0;
} 