#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

class VehicleOdometryListener : public rclcpp::Node
{
public:
	explicit VehicleOdometryListener() : Node("twist_listener_node")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
		subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos,
		[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
			std::cout << "\n=============================" << std::endl;
			std::cout << "RECEIVED Vehicle Twist DATA"   << std::endl;
			std::cout << "============================="   << std::endl;
			std::cout << "Timestamp: "          << msg->timestamp    << std::endl;
      
			std::cout << "velocity[0]: " << msg->velocity[0]  << std::endl;
			std::cout << "velocity[1]: " << msg->velocity[1]  << std::endl;
			std::cout << "velocity[2]: " << msg->velocity[2]  << std::endl;

			std::cout << "angular_velocity[0]: " << msg->angular_velocity[0] << std::endl;
			std::cout << "angular_velocity[1]: " << msg->angular_velocity[1] << std::endl;
			std::cout << "angular_velocity[2]: " << msg->angular_velocity[2] << std::endl;

		});
	}

private:
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;

};

int main(int argc, char *argv[])
{
	std::cout << "Starting vehicle twist listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VehicleOdometryListener>());

	rclcpp::shutdown();
	return 0;
} 