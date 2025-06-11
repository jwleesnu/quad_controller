#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <chrono>
#include <iostream>
#include <string>
#include <std_msgs/msg/int32.hpp>
#include "so3_utils.hpp"
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
// optitrack odometry 받아와서 --> pose_callback 함수 바꾸기. (position, orientation, linear velocity)
// IMU angular rate를 받아올 수 있는가?

class MotorControl : public rclcpp::Node
{
public:
    MotorControl(std::string px4_namespace) :
		Node("motor_control_srv"),
      	state_{State::init},
      	service_result_{0},
      	service_done_{false}
	{
		// uxrQoS_t qos_pub = {
		// 	.durability = UXR_DURABILITY_VOLATILE,
		// 	.reliability = UXR_RELIABILITY_BEST_EFFORT,
		// 	.history = UXR_HISTORY_KEEP_LAST,
		// 	.depth = queue_depth,
		// };

		// uxrQoS_t qos_sub = {
		// 	.durability = UXR_DURABILITY_TRANSIENT_LOCAL,
		// 	.reliability = UXR_RELIABILITY_BEST_EFFORT,
		// 	.history = UXR_HISTORY_KEEP_LAST,
		// 	.depth = 0,
		// };

        // QoS setup
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos_sub = rclcpp::QoS(
            rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
            qos_profile
        );

		rclcpp::QoS qos_pub(rclcpp::KeepLast(10));  // 최신 10개 메시지 유지
		qos_pub.reliable();  // 신뢰성 높은 전송
		qos_pub.durability_volatile();  // 데이터  지속성 없음

      //   Subscribe
      vehicle_odometry_subscriber_ = this->create_subscription<VehicleOdometry>(
              px4_namespace + "in/vehicle_visual_odometry", qos_sub,
              std::bind(&MotorControl::odometry_callback, this, std::placeholders::_1));
      imu_subscriber_ = this->create_subscription<SensorCombined>(
              px4_namespace + "out/sensor_combined", qos_sub,
              std::bind(&MotorControl::imu_callback, this, std::placeholders::_1));
      // optitrack_odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      //       "/optitrackKYJLJW", qos_sub,
      //       std::bind(&MotorControl::optitrack_odometry_callback, this, std::placeholders::_1));
      ui_command_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
              "ui_command", qos_sub,
              std::bind(&MotorControl::ui_command_callback, this, std::placeholders::_1));
      
      // Publish
      offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(px4_namespace+"in/offboard_control_mode", qos_pub);
      actuator_motors_publisher_ = this->create_publisher<ActuatorMotors>(px4_namespace+"in/actuator_motors", qos_pub);
      // thrust_publisher_ = this->create_publisher<VehicleThrustSetpoint>(px4_namespace+"in/vehicle_thrust_setpoint", 10);
      // torque_publisher_ = this->create_publisher<VehicleTorqueSetpoint>(px4_namespace+"in/vehicle_torque_setpoint", 10);
       
      // Logging
      pos_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/record/pos", qos_pub);
      rpy_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/record/rpy", qos_pub);
      vel_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/record/vel", qos_pub);
      omg_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/record/omg", qos_pub);
      pos_des_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/record/pos_des", qos_pub);
      vel_des_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/record/vel_des", qos_pub);
      omg_des_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/record/omg_des", qos_pub);
      rpy_des_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/record/rpy_des", qos_pub);

      // Client
      vehicle_command_client_ = this->create_client<px4_msgs::srv::VehicleCommand>(px4_namespace+"vehicle_command");

      // Wait for PX4 connection
      RCLCPP_INFO(this->get_logger(), "Starting Direct Motor Control example with PX4 services");
      RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for " << px4_namespace << "vehicle_command service");
      while (!vehicle_command_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
         }
         RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
      }
      load_parameters();

		Flightflag = false;

		pos_des(0,0) = 0.0;
		pos_des(1,0) = 0.0;
		pos_des(2,0) = 0.0;

		vel_des(0,0) = 0.0;
		vel_des(1,0) = 0.0;
		vel_des(2,0) = 0.0;

		Rotation_matrix_des.setIdentity();
		prev_Rotation_matrix_des.setIdentity();

		omega_des(0,0) = 0.0;
		omega_des(1,0) = 0.0;
		omega_des(2,0) = 0.0;

		omega_des_dot(0,0) = 0.0;
		omega_des_dot(1,0) = 0.0;
		omega_des_dot(2,0) = 0.0;

		prev_omega_des(0,0) = 0.0;
		prev_omega_des(1,0) = 0.0;
		prev_omega_des(2,0) = 0.0;

		offboard_cm = 0;

		Thrust_Torque(0,0) = 0.0;
		Thrust_Torque(1,0) = 0.0;
		Thrust_Torque(2,0) = 0.0;
		Thrust_Torque(3,0) = 0.0;
		Thrust_motor(1,0) = 0.0;
		Thrust_motor(2,0) = 0.0;
		Thrust_motor(3,0) = 0.0;

      e_int.setZero();

		e3.setZero();
      e3(2,0) = 1.0;

      timestamp_sec = 0;
      prev_timestamp_sec = 0;

      heading_axis_direction_B1.setZero();
      heading_axis_direction_B1(1,0) = 1.0;

      // Main timer callback function
      timer_ = this->create_wall_timer(2.5ms, std::bind(&MotorControl::timer_callback, this));
    }

   	void switch_to_offboard_mode();
   	void arm();
   	void disarm();
	   void declare_parameters();
      void load_parameters();
   	void initialize_des();

private:

    enum class State{
      	init,
      	offboard_requested,
      	offboard_entered,
      	wait_for_stable_offboard_mode,
      	arm_requested,
      	armed,
    	flight
   	} state_;
   	uint8_t service_result_;
   	bool service_done_;
   	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<ActuatorMotors>::SharedPtr actuator_motors_publisher_;
	// rclcpp::Publisher<VehicleThrustSetpoint>::SharedPtr thrust_publisher_;
	// rclcpp::Publisher<VehicleTorqueSetpoint>::SharedPtr torque_publisher_;
   
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pos_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr vel_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr omg_publisher_;

	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pos_des_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_des_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr vel_des_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr omg_des_publisher_;

	rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;
	rclcpp::Subscription<SensorCombined>::SharedPtr imu_subscriber_;
	// rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr optitrack_odometry_subscriber_;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ui_command_subscriber_;
	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;

   double m_b;
   Eigen::Matrix<double, 3, 3> J_b;
   double g;
   double T_max;
   double t_max;
   Eigen::Matrix<double, 3, 1> Tau_max;

   Eigen::Matrix<double, 3, 3> K_p, K_v, K_i, K_Rot, K_omega;
   Eigen::Matrix<double, 3, 1> pos, vel, pos_des, vel_des;
	Eigen::Matrix<double, 3, 3> Rotation_matrix, Rotation_matrix_des, prev_Rotation_matrix_des;
	Eigen::Matrix<double, 3, 1> omega, omega_des, omega_des_dot, prev_omega_des;
   Eigen::Matrix<double, 3, 1> e_pos, e_vel, e_int, e_Rot, e_omega;
   Eigen::Matrix<double, 3, 1> b1, b2, b3_nume, b3, e3, heading_axis_direction_B1;
   Eigen::Matrix<double, 4, 4> W, W_inv;

	Eigen::Matrix<double, 4, 1> Thrust_Torque;
	Eigen::Matrix<double, 4, 1> Thrust_motor;

	bool Flightflag;

	double prev_timestamp_sec;
	double timestamp_sec;


	double dz_takeoff, dxy, dz, drot;
	double offboard_cm;

	void publish_offboard_control_mode(double offboard_cm);
	void publish_actuator_motors();
	// void publish_thrust();
	// void publish_torque();

   void record_state();

	void odometry_callback(const VehicleOdometry::SharedPtr msg);
	void imu_callback(const SensorCombined::SharedPtr msg);
	// void optitrack_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void request_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
	void timer_callback(void);
	void compute_actuator_motors();
	void ui_command_callback(const std_msgs::msg::Int32::SharedPtr msg);
};

void MotorControl::load_parameters()
{
   // Initialize parameters
	this->declare_parameter<double>("m_b", 1.0);
	this->declare_parameter<std::vector<double>>("K_pos", {1.0, 1.0, 1.0});
	this->declare_parameter<std::vector<double>>("K_vel", {1.0, 1.0, 1.0});
	this->declare_parameter<std::vector<double>>("K_int", {1.0, 1.0, 1.0});
	this->declare_parameter<std::vector<double>>("K_Rot", {1.0, 1.0, 1.0});
	this->declare_parameter<std::vector<double>>("K_omega", {1.0, 1.0, 1.0});
	this->declare_parameter<double>("g", 9.81);
	this->declare_parameter<double>("J_bx", 0.01);
	this->declare_parameter<double>("J_by", 0.01);
	this->declare_parameter<double>("J_bz", 0.01);
	this->declare_parameter<double>("T_max", 1.0);
	this->declare_parameter<double>("Tau_max_x", 1.0);
	this->declare_parameter<double>("Tau_max_y", 1.0);
	this->declare_parameter<double>("Tau_max_z", 1.0);
	this->declare_parameter<double>("d", 1.0);
	this->declare_parameter<double>("c_tau_f", 1.0);
	this->declare_parameter<double>("t_max", 1.0);
	this->declare_parameter<double>("dz_takeoff", 1.0);
	this->declare_parameter<double>("dxy", 1.0);
	this->declare_parameter<double>("dz", 1.0);
	this->declare_parameter<double>("drot", 1.0);
   
   // Load parameters
   m_b = this->get_parameter("m_b").as_double();
   J_b = Eigen::Matrix3d::Zero();
   J_b(0, 0) = this->get_parameter("J_bx").as_double();
   J_b(1, 1) = this->get_parameter("J_by").as_double();
   J_b(2, 2) = this->get_parameter("J_bz").as_double();
   g = this->get_parameter("g").as_double();
	T_max = this->get_parameter("T_max").as_double();
	Tau_max(0, 0) = this->get_parameter("Tau_max_x").as_double();
	Tau_max(1, 0) = this->get_parameter("Tau_max_y").as_double();
	Tau_max(2, 0) = this->get_parameter("Tau_max_z").as_double();
	t_max = this->get_parameter("t_max").as_double();
	dz_takeoff = this->get_parameter("dz_takeoff").as_double();
	dxy = this->get_parameter("dxy").as_double();
	dz = this->get_parameter("dz").as_double();
	drot = this->get_parameter("drot").as_double();
	drot = drot * M_PI / 180.0;
   std::vector<double> k_pos = this->get_parameter("K_pos").as_double_array();
   K_p = Eigen::Matrix3d::Zero();
   for (int i = 0; i < 3; ++i) {
      K_p(i, i) = k_pos[i];
   }
   std::vector<double> k_vel = this->get_parameter("K_vel").as_double_array();
   K_v = Eigen::Matrix3d::Zero();
   for (int i = 0; i < 3; ++i) {
      K_v(i, i) = k_vel[i];
   }
   std::vector<double> k_int = this->get_parameter("K_int").as_double_array();
   K_i = Eigen::Matrix3d::Zero();
   for (int i = 0; i < 3; ++i) {
      K_i(i, i) = k_int[i];
   }
   std::vector<double> k_rot = this->get_parameter("K_Rot").as_double_array();
   K_Rot = Eigen::Matrix3d::Zero();
   for (int i = 0; i < 3; ++i) {
      K_Rot(i, i) = k_rot[i];
   }
   std::vector<double> k_omega = this->get_parameter("K_omega").as_double_array();
   K_omega = Eigen::Matrix3d::Zero();
   for (int i = 0; i < 3; ++i) {
      K_omega(i, i) = k_omega[i];
   }

	// Mixer
	W(0,0) = 1.0;
	W(0,1) = 1.0;
	W(0,2) = 1.0;
	W(0,3) = 1.0;
	W(1,0) = -this->get_parameter("d").as_double();
	W(1,1) = this->get_parameter("d").as_double();
	W(1,2) = this->get_parameter("d").as_double();
	W(1,3) = -this->get_parameter("d").as_double();
	W(2,0) = this->get_parameter("d").as_double();
	W(2,1) = -this->get_parameter("d").as_double();
	W(2,2) = this->get_parameter("d").as_double();
	W(2,3) = -this->get_parameter("d").as_double();
	W(3,0) = this->get_parameter("c_tau_f").as_double();
	W(3,1) = this->get_parameter("c_tau_f").as_double();
	W(3,2) = -this->get_parameter("c_tau_f").as_double();
	W(3,3) = -this->get_parameter("c_tau_f").as_double();
   W_inv = W.inverse();
	RCLCPP_INFO(this->get_logger(), "load parameters");
}

void MotorControl::switch_to_offboard_mode(){
   RCLCPP_INFO(this->get_logger(), "requesting switch to Offboard mode");
   request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
}

void MotorControl::arm()
{
   RCLCPP_INFO(this->get_logger(), "requesting arm");
   request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

void MotorControl::disarm()
{
   RCLCPP_INFO(this->get_logger(), "requesting disarm");
   request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}

void MotorControl::initialize_des()
{
   RCLCPP_INFO(this->get_logger(), "Initializing desired position and attitude");
   if (timestamp_sec == 0){
      RCLCPP_WARN(this->get_logger(), "Odometry data not yet received. Waiting for valid data...");
      return;
   }
   pos_des = pos;
   heading_axis_direction_B1 = Rotation_matrix.col(0);
   // RCLCPP_INFO(this->get_logger(), "Desired position initialized to: [%.3f, %.3f, %.3f]", pos_des(0, 0), pos_des(1, 0), pos_des(2, 0));
    // RCLCPP_INFO(this->get_logger(), "Desired attitude initialized to: [%.3f, %.3f, %.3f]", heading_axis_direction_B1(0,0), heading_axis_direction_B1(1,0), heading_axis_direction_B1(2,0));
}

void MotorControl::publish_offboard_control_mode(double offboard_cm)
{
	OffboardControlMode msg{};
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
   msg.thrust_and_torque = false;
   if (offboard_cm == 0){
		msg.position = true;
      msg.direct_actuator = false;
   }
	else{
      msg.position = false;
      msg.direct_actuator = true;
   }
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Published Offboard Control Mode");
}

void MotorControl::publish_actuator_motors()
{
	ActuatorMotors msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	msg.timestamp_sample = this->get_clock()->now().nanoseconds() / 1000;
   msg.control[0] = Thrust_motor(0,0);
   msg.control[1] = Thrust_motor(1,0);
   msg.control[2] = Thrust_motor(2,0);
   msg.control[3] = Thrust_motor(3,0);
   //  msg.control[0] = 0.0;
   //  msg.control[1] = 0.0;
   //  msg.control[2] = 0.0;
   //  msg.control[3] = 0.0;
    actuator_motors_publisher_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Published actuator motors");
   // RCLCPP_INFO(this->get_logger(), "PWM: %.3f, %.3f, %.3f, %.3f", Thrust_motor(0,0), Thrust_motor(1,0), Thrust_motor(2,0), Thrust_motor(3,0));
}



// void MotorControl::publish_thrust()
// {
//     px4_msgs::msg::VehicleThrustSetpoint msg{};
//     msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
//     msg.xyz[0] = 0;
//     msg.xyz[1] = 0;
//     msg.xyz[2] = -Thrust_Torque(0, 0) / T_max;
//     thrust_publisher_->publish(msg);
// }

// void MotorControl::publish_torque()
// {
//     px4_msgs::msg::VehicleTorqueSetpoint msg{};
//     msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
//     msg.xyz[0] = Thrust_Torque(1, 0) / Tau_max(0,0);
//     msg.xyz[1] = Thrust_Torque(2, 0) / Tau_max(1,0);
//     msg.xyz[2] = Thrust_Torque(3, 0) / Tau_max(2,0);
//     torque_publisher_->publish(msg);
// }

void MotorControl::record_state()
{
   geometry_msgs::msg::Vector3Stamped msg1{};
   msg1.header.stamp = this->get_clock()->now();
   msg1.header.frame_id = "base_link";
   msg1.vector.x = pos(0,0);
   msg1.vector.y = pos(1,0);
   msg1.vector.z = pos(2,0);
   pos_publisher_->publish(msg1);

   geometry_msgs::msg::Vector3Stamped msg2{};
   msg2.header.stamp = this->get_clock()->now();
   msg2.header.frame_id = "base_link";
   Eigen::Matrix<double, 3, 1> rpy = so3::R2rpy(Rotation_matrix);
   msg2.vector.x = rpy(0,0);
   msg2.vector.y = rpy(1,0);
   msg2.vector.z = rpy(2,0);
   rpy_publisher_->publish(msg2);

   geometry_msgs::msg::Vector3Stamped msg3{};
   msg3.header.stamp = this->get_clock()->now();
   msg3.header.frame_id = "base_link";
   msg3.vector.x = vel(0,0);
   msg3.vector.y = vel(1,0);
   msg3.vector.z = vel(2,0);
   vel_publisher_->publish(msg3);

   geometry_msgs::msg::Vector3Stamped msg4{};
   msg4.header.stamp = this->get_clock()->now();
   msg4.header.frame_id = "base_link";
   msg4.vector.x = omega(0,0);
   msg4.vector.y = omega(1,0);
   msg4.vector.z = omega(2,0);
   omg_publisher_->publish(msg4);


   geometry_msgs::msg::Vector3Stamped msg5{};
   msg5.header.stamp = this->get_clock()->now();
   msg5.header.frame_id = "base_link";
   msg5.vector.x = pos_des(0,0);
   msg5.vector.y = pos_des(1,0);
   msg5.vector.z = pos_des(2,0);
   pos_des_publisher_->publish(msg5);

   geometry_msgs::msg::Vector3Stamped msg6{};
   msg6.header.stamp = this->get_clock()->now();
   msg6.header.frame_id = "base_link";
   Eigen::Matrix<double, 3, 1> rpy_des = so3::R2rpy(Rotation_matrix_des);
   msg6.vector.x = rpy_des(0,0);
   msg6.vector.y = rpy_des(1,0);
   msg6.vector.z = rpy_des(2,0);
   rpy_des_publisher_->publish(msg6);

   geometry_msgs::msg::Vector3Stamped msg7{};
   msg7.header.stamp = this->get_clock()->now();
   msg7.header.frame_id = "base_link";
   msg7.vector.x = vel_des(0,0);
   msg7.vector.y = vel_des(1,0);
   msg7.vector.z = vel_des(2,0);
   vel_des_publisher_->publish(msg7);

   geometry_msgs::msg::Vector3Stamped msg8{};
   msg8.header.stamp = this->get_clock()->now();
   msg8.header.frame_id = "base_link";
   msg8.vector.x = omega_des(0,0);
   msg8.vector.y = omega_des(1,0);
   msg8.vector.z = omega_des(2,0);
   omg_des_publisher_->publish(msg8);
}

// void MotorControl::publish_rpy_des()
// {
//    geometry_msgs::msg::Vector3Stamped msg{};
//    msg.header.stamp = this->get_clock()->now();
//    msg.header.frame_id = "base_link";
//    Eigen::Matrix<double, 3, 1> rpy_des = so3::R2rpy(Rotation_matrix_des);
//    msg.vector.x = rpy_des(0,0) * 180.0 / M_PI;
//    msg.vector.y = rpy_des(1,0) * 180.0 / M_PI;
//    msg.vector.z = rpy_des(2,0) * 180.0 / M_PI;
//    rpy_des_publisher_->publish(msg);
// }

void MotorControl::compute_actuator_motors()
{
   double timestamp = timestamp_sec * 1e-6;
   double prev_timestamp = prev_timestamp_sec * 1e-6;

   if (timestamp != prev_timestamp){

      double DT;
      if (prev_timestamp==0){
         DT = 0.01;
      }
      else{
         DT = (timestamp_sec - prev_timestamp_sec)*1e-6;
      }

      e_pos = pos - pos_des;
      e_vel = vel - vel_des;
      e_int+= e_pos * DT;
     
      b3_nume = (-K_p*e_pos -K_v*e_vel -K_i*e_int -m_b*g*e3);
      b3 = -b3_nume / sqrt(b3_nume.dot(b3_nume));
      b2 = b3.cross(heading_axis_direction_B1);

      b2 = b2/sqrt(b2.dot(b2));
      b1 = b2.cross(b3);
      b1 = b1/sqrt(b1.dot(b1));


      prev_timestamp_sec = timestamp_sec;

      Rotation_matrix_des.block<3,1>(0,0) = b1;
      Rotation_matrix_des.block<3,1>(0,1) = b2;
      Rotation_matrix_des.block<3,1>(0,2) = b3;

      Eigen::Matrix<double, 3, 3> temp_R;

      if (prev_timestamp==0){
         prev_Rotation_matrix_des.block<3,1>(0,0) = b1;
         prev_Rotation_matrix_des.block<3,1>(0,1) = b2;
         prev_Rotation_matrix_des.block<3,1>(0,2) = b3;
         temp_R = prev_Rotation_matrix_des.transpose() * Rotation_matrix_des;
      }
      else{
         if(DT == 0){
            RCLCPP_WARN(this->get_logger(), "DT: %.3f, ts:%f", DT*1000, timestamp);
         }
         
         temp_R = prev_Rotation_matrix_des.transpose() * Rotation_matrix_des;

         prev_Rotation_matrix_des = Rotation_matrix_des;
      }


      omega_des = so3::vee(temp_R.log()/DT);
     
      if (prev_omega_des(0,0)==0 && prev_omega_des(1,0)==0 && prev_omega_des(2,0)==0){
         omega_des_dot(0,0) = 0;
         omega_des_dot(1,0) = 0;
         omega_des_dot(2,0) = 0;
      }
      else{
         omega_des_dot = (omega_des - prev_omega_des)/DT;
      }
      // omega_des_dot.setZero(); // Modified! 주의

      prev_omega_des = omega_des;

      e_Rot = so3::vee(0.5*(Rotation_matrix_des.transpose()*Rotation_matrix - Rotation_matrix.transpose()*Rotation_matrix_des));
      e_omega = omega - Rotation_matrix.transpose()*Rotation_matrix_des*omega_des;
     
      Thrust_Torque(0,0) = -b3_nume.transpose()*Rotation_matrix*e3;
     
      Eigen::Matrix<double, 3, 1> temp_RHS;
      temp_RHS = -K_Rot*e_Rot -K_omega*e_omega +omega.cross(J_b*omega) -J_b*(so3::hat(omega)*Rotation_matrix.transpose()*Rotation_matrix_des*omega_des - Rotation_matrix.transpose()*Rotation_matrix_des*omega_des_dot);

      RCLCPP_INFO(this->get_logger(), "Thrust : %.3f, Torque : %.3f %.3f %.3f", Thrust_Torque(0,0),temp_RHS(0,0),temp_RHS(1,0),temp_RHS(2,0));
      // temp_RHS = -K_omega.inverse()*K_Rot*e_Rot + Rotation_matrix.transpose()*Rotation_matrix_des*omega_des;
      // RCLCPP_INFO(this->get_logger(), "Thrust : %.3f, Torque : %.3f %.3f %.3f", Thrust_Torque(0,0),temp_RHS(0,0),temp_RHS(1,0),temp_RHS(2,0));
      // RCLCPP_INFO(this->get_logger(), "pos : %.3f, %.3f, %.3f", pos(0,0), pos(1,0), pos(2,0));
      // RCLCPP_INFO(this->get_logger(), "pos_des : %.3f, %.3f, %.3f", pos_des(0,0), pos_des(1,0), pos_des(2,0));
      // RCLCPP_INFO(this->get_logger(), "vel : %.3f, %.3f, %.3f", vel(0,0), vel(1,0), vel(2,0));
      // RCLCPP_INFO(this->get_logger(), "vel_des : %.3f, %.3f, %.3f", vel_des(0,0), vel_des(1,0), vel_des(2,0));
      // RCLCPP_INFO(this->get_logger(), "e_pos : %.3f, %.3f, %.3f", e_pos(0,0), e_pos(1,0), e_pos(2,0));
      // RCLCPP_INFO(this->get_logger(), "e_vel : %.3f, %.3f, %.3f", e_vel(0,0), e_vel(1,0), e_vel(2,0));
      // Eigen::Matrix3d T_NED_to_ENU = (Eigen::Matrix3d() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished();
      // Eigen::Matrix3d T_FRD_to_FLU = (Eigen::Matrix3d() << 1, 0, 0, 0, -1, 0, 0, 0, -1).finished();
      // temp_RHS = temp_RHS;

      // Thrust_Torque(0,0) = Thrust_Torque(0,0)/T_max;
      // Thrust_Torque(1,0) = temp_RHS(0,0)/Tau_max(0,0);
      // Thrust_Torque(2,0) = temp_RHS(1,0)5/Tau_max(1,0);
      // Thrust_Torque(3,0) = temp_RHS(2,0)/Tau_max(2,0);

      Thrust_Torque(1,0) = temp_RHS(0,0);
      Thrust_Torque(2,0) = temp_RHS(1,0);
      Thrust_Torque(3,0) = temp_RHS(2,0);

      // Eigen::Matrix<double, 3, 1> rpy;
      // Eigen::Matrix<double, 3, 1> rpy_des;
      // rpy = so3::R2rpy(Rotation_matrix);
      // rpy_des = so3::R2rpy(Rotation_matrix_des);
      // RCLCPP_INFO(this->get_logger(), "\n%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f,", pos(0,0), pos(1,0), pos(2,0),rpy(0,0), rpy(1,0), rpy(2,0), rpy_des(0,0), rpy_des(1,0), rpy_des(2,0));
      // RCLCPP_INFO(this->get_logger(),"\nPos: %.3f %.3f %.3f, Vel: %.3f %.3f %.3f, Omg: %.3f %.3f %.3f", pos(0,0), pos(1,0), pos(2,0), vel(0,0), vel(1,0), vel(2,0), omega(0,0), omega(1,0), omega(2,0));
      // RCLCPP_INFO(this->get_logger(),"\n%.3f %.3f %.3f\n%.3f %.3f %.3f\n%.3f %.3f %.3f", Rotation_matrix(0,0),Rotation_matrix(0,1),Rotation_matrix(0,2),Rotation_matrix(1,0),Rotation_matrix(1,1),Rotation_matrix(1,2),Rotation_matrix(2,0),Rotation_matrix(2,1),Rotation_matrix(2,2));

		Thrust_motor = W_inv * Thrust_Torque;
		Thrust_motor(0,0) = Thrust_motor(0,0) / t_max;
		Thrust_motor(1,0) = Thrust_motor(1,0) / t_max;
		Thrust_motor(2,0) = Thrust_motor(2,0) / t_max;
		Thrust_motor(3,0) = Thrust_motor(3,0) / t_max;
		if (std::isnan(Thrust_motor(0,0)) || std::isnan(Thrust_motor(1,0)) || std::isnan(Thrust_motor(2,0)) || std::isnan(Thrust_motor(3,0))){
			RCLCPP_WARN(this->get_logger(), "PWM: %.3f, %.3f, %.3f, %.3f", Thrust_motor(0,0), Thrust_motor(1,0), Thrust_motor(2,0), Thrust_motor(3,0));
		}
      RCLCPP_INFO(this->get_logger(), "PWM: %.3f, %.3f, %.3f, %.3f", Thrust_motor(0,0), Thrust_motor(1,0), Thrust_motor(2,0), Thrust_motor(3,0));
   	}
}

void MotorControl::odometry_callback(const VehicleOdometry::SharedPtr msg)
{
   // position, quaternion, velocity
   timestamp_sec = msg->timestamp;
   pos(0,0) = msg->position[0];
   pos(1,0) = msg->position[1];
   pos(2,0) = msg->position[2];

   Eigen::Quaternion<double> quat;
   quat.w() = msg->q[0];
   quat.x() = msg->q[1];
   quat.y() = msg->q[2];
   quat.z() = msg->q[3];
   Rotation_matrix = quat.toRotationMatrix();

   vel(0,0) = msg->velocity[0];
   vel(1,0) = msg->velocity[1];
   vel(2,0) = msg->velocity[2];

}

void MotorControl::imu_callback(const SensorCombined::SharedPtr msg)
{
   omega(0,0) = msg->gyro_rad[0];
   omega(1,0) = msg->gyro_rad[1];
   omega(2,0) = msg->gyro_rad[2];
}

// void MotorControl::optitrack_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
// {
//    timestamp_sec = msg->header.stamp.sec;
//    timestamp_nsec = msg->header.stamp.nanosec;

//    pos(0,0) = msg->pose.pose.position.x;
//    pos(1,0) = -msg->pose.pose.position.y;
//    pos(2,0) = -msg->pose.pose.position.z;

//    Eigen::Quaternion<double> quat;
//    quat.w() = msg->pose.pose.orientation.w;
//    quat.x() = msg->pose.pose.orientation.x;
//    quat.y() = -msg->pose.pose.orientation.y;
//    quat.z() = -msg->pose.pose.orientation.z;
//    Rotation_matrix = quat.toRotationMatrix();
   
//    vel(0,0) = msg->twist.twist.linear.x;
//    vel(1,0) = -msg->twist.twist.linear.y;
//    vel(2,0) = -msg->twist.twist.linear.z;

//    // pos(0,0) = msg->position[1];
//    // pos(1,0) = msg->position[0];
//    // pos(2,0) = -msg->position[2];

//    // Eigen::Quaternion<double> quat;
//    // quat.w() = msg->q[0];
//    // quat.x() = msg->q[1];
//    // quat.y() = msg->q[2];
//    // quat.z() = msg->q[3];
//    // Eigen::Matrix3d T_NED_to_ENU = (Eigen::Matrix3d() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished();
//    // Eigen::Matrix3d T_FRD_to_FLU = (Eigen::Matrix3d() << 1, 0, 0, 0, -1, 0, 0, 0, -1).finished();
//    // Eigen::Matrix3d R_FRD_NED = quat.toRotationMatrix();
//    // Rotation_matrix = T_NED_to_ENU * R_FRD_NED * T_FRD_to_FLU;
   
//    // vel(0,0) = msg->velocity[1];
//    // vel(1,0) = msg->velocity[0];
//    // vel(2,0) = -msg->velocity[2];

//    // omega(0,0) = msg->angular_velocity[0];
//    // omega(1,0) = -msg->angular_velocity[1];
//    // omega(2,0) = -msg->angular_velocity[2];

// }

void MotorControl::request_vehicle_command(uint16_t command, float param1, float param2)
{
   auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

   VehicleCommand msg{};
   msg.param1 = param1;
   msg.param2 = param2;
   msg.command = command;
   msg.target_system = 1;
   msg.target_component = 1;
   msg.source_system = 1;
   msg.source_component = 1;
   msg.from_external = true;
   msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
   request->request = msg;

   service_done_ = false;
   auto result = vehicle_command_client_->async_send_request(request, std::bind(&MotorControl::response_callback, this, std::placeholders::_1));
   RCLCPP_INFO(this->get_logger(), "Command send");
}

void MotorControl::ui_command_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
   int command = msg->data;
   
   Eigen::Matrix<double, 3, 3> Rot_CW, Rot_CCW;
   Rot_CW.setIdentity();
   Rot_CW(0,0) = cos(drot);
   Rot_CW(0,1) =-sin(drot);
   Rot_CW(1,0) = sin(drot);
   Rot_CW(1,1) = cos(drot);
   Rot_CCW.setIdentity();
   Rot_CCW(0,0) = cos(drot);
   Rot_CCW(0,1) = sin(drot);
   Rot_CCW(1,0) =-sin(drot);
   Rot_CCW(1,1) = cos(drot);
   switch (command){
      case 1:
         RCLCPP_WARN(this->get_logger(), "UI_COMMAND: Kill-Switch Engaged");
            request_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_FLIGHTTERMINATION, 1.0, 0.0);
            break;
      case 2:
         RCLCPP_INFO(this->get_logger(), "UI_COMMAND: Switch to Offboard Mode");
         switch_to_offboard_mode();
         state_ = State::offboard_requested;
         break;
      case 3:
         RCLCPP_INFO(this->get_logger(), "UI_COMMAND: Arming");
         state_ = State::wait_for_stable_offboard_mode;
         break;
      case 4:
         RCLCPP_INFO(this->get_logger(), "UI_COMMAND: Takeoff");
         Flightflag = true;
         pos_des(2,0) -= dz_takeoff;
         break;
      // case 5:
      //    RCLCPP_INFO(this->get_logger(), "UI_COMMAND: Disarming");
      //    disarm();
      //    state_ = State::offboard_requested;
      //    break;
      case 11:
         RCLCPP_INFO(this->get_logger(), "UI_COMMAND: Forward");
         pos_des(0,0) += dxy;
         break;
      case 12:
         RCLCPP_INFO(this->get_logger(), "UI_COMMAND: Backward");
         pos_des(0,0) -= dxy;
         break;
      case 13:
         RCLCPP_INFO(this->get_logger(), "UI_COMMAND: Left");
         pos_des(1,0) -= dxy;
         break;
      case 14:
         RCLCPP_INFO(this->get_logger(), "UI_COMMAND: Right");
         pos_des(1,0) += dxy;
         break;
      case 15:
         RCLCPP_INFO(this->get_logger(), "UI_COMMAND: Up");
         pos_des(2,0) -= dz;
         break;
      case 16:
         RCLCPP_INFO(this->get_logger(), "UI_COMMAND: Down");
         pos_des(2,0) += dz;
         break;
      case 17:
         RCLCPP_INFO(this->get_logger(), "UI_COMMAND: CW");
         heading_axis_direction_B1 = Rot_CW * heading_axis_direction_B1;
         break;
      case 18:
         RCLCPP_INFO(this->get_logger(), "UI_COMMAND: CCW");
         heading_axis_direction_B1 = Rot_CCW * heading_axis_direction_B1;
         break;
      // 1 : Emergency, 2 : Offboard, 3 : Arming, 4 : Takeoff
      // 11 : Foward, 12 : Backward, 13 : Left, 14 : Right, 15 : Up, 16 : Down, 17 : CW, 18 : CCW

   }
}

void MotorControl::timer_callback(void){
   static uint8_t num_of_steps = 0;
   static uint8_t num_of_steps_arm = 0;

   // offboard_control_mode needs to be paired with actuator_motors
   if (state_!=State::init){
      publish_offboard_control_mode(offboard_cm);
   }

   switch (state_)
   {
   case State::init :
      break;
   case State::offboard_requested :
      if(service_done_){
         if (service_result_==0){
            RCLCPP_INFO(this->get_logger(), "Entered offboard mode");
            state_ = State::offboard_entered;        
         }
         else{
            RCLCPP_ERROR(this->get_logger(), "Failed to enter offboard mode, exiting");
            rclcpp::shutdown();
         }
      }
      break;
   case State::offboard_entered :
      break;
   case State::wait_for_stable_offboard_mode :
      if (++num_of_steps>10){
         arm();
         state_ = State::arm_requested;
      }
      break;
   case State::arm_requested :
      if(service_done_){
         if (service_result_==0){
            RCLCPP_INFO(this->get_logger(), "Vehicle is armed");
            initialize_des();
            state_ = State::armed;
         }
         else{
            RCLCPP_ERROR(this->get_logger(), "Failed to arm, exiting");
            rclcpp::shutdown();
         }
      }
      break;
   case State::armed :
   		if (++num_of_steps_arm>10){
            if(num_of_steps_arm==11){
            RCLCPP_INFO(this->get_logger(), "Switch to Offboard Arming");
            }
            offboard_cm = 1;
            Thrust_motor(0,0) = 0.1;	// Minimal Arm PWM
            Thrust_motor(1,0) = 0.1;
            Thrust_motor(2,0) = 0.1;
            Thrust_motor(3,0) = 0.1;
            publish_actuator_motors();
            // publish_thrust();
            // publish_torque();
            record_state();
            if(Flightflag == true){
               state_ = State::flight;
            }
		}
      break;
   case State::flight :
      compute_actuator_motors();
      publish_actuator_motors();
      // publish_thrust();
      // publish_torque();  
      record_state();
      break;
   default:
      // compute_actuator_motors();
      // publish_thrust();
      // publish_torque();
      // publish_actuator_motors();
      break;
   }
}

void MotorControl::response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
     auto reply = future.get()->reply;
     service_result_ = reply.result;
      switch (service_result_)
      {
      case reply.VEHICLE_CMD_RESULT_ACCEPTED:
         RCLCPP_INFO(this->get_logger(), "command accepted");
         break;
      case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
         RCLCPP_WARN(this->get_logger(), "command temporarily rejected");
         break;
      case reply.VEHICLE_CMD_RESULT_DENIED:
         RCLCPP_WARN(this->get_logger(), "command denied");
         break;
      case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
         RCLCPP_WARN(this->get_logger(), "command unsupported");
         break;
      case reply.VEHICLE_CMD_RESULT_FAILED:
         RCLCPP_WARN(this->get_logger(), "command failed");
         break;
      case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
         RCLCPP_WARN(this->get_logger(), "command in progress");
         break;
      case reply.VEHICLE_CMD_RESULT_CANCELLED:
         RCLCPP_WARN(this->get_logger(), "command cancelled");
         break;
      default:
         RCLCPP_WARN(this->get_logger(), "command reply unknown");
         break;
      }
      service_done_ = true;
    }
    else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}

int main(int argc, char *argv[])
{
   setvbuf(stdout, NULL, _IONBF, BUFSIZ);
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<MotorControl>("/fmu/"));

   rclcpp::shutdown();
   return 0;
}

// For real hardware, launch file should be
      //   <param name="K_pos" value="[5.0, 5.0, 15.0]"/> <!---->
      //   <param name="K_vel" value="[0.3, 0.3, 0.8]"/> <!--[3.0, 3.0, 8.0]-->
      //   <param name="K_Rot" value="[0.5, 0.5, 0.1]"/> <!--[0.5, 0.5, 1.0]-->
      //   <param name="K_omega" value="[0.05, 0.05, 0.01]"/> <!--[0.5, 0.5, 1.0]-->

      //   <param name="m_b" value="2.15"/> <!--2.3643-->
      //   <param name="J_bx" value="0.023847"/>
      //   <param name="J_by" value="0.023950"/>
      //   <param name="J_bz" value="0.044000"/>
      //   <param name="g" value="9.81"/>

      //   <param name="T_max" value="34.0"/> <!--34.0-->
      //   <param name="Tau_max_x" value="2.97"/> <!---->
      //   <param name="Tau_max_y" value="2.97"/> <!---->
      //   <param name="Tau_max_z" value="0.274"/> <!---->
      //   <param name="d" value="0.2"/> <!--0.174-->
      //   <param name="c_tau_f" value="0.016"/> <!--0.016-->
      //   <param name="t_max" value="10.0"/> <!--8.54858-->

      //   <param name="dz_takeoff" value="0.25"/>
      //   <param name="dxy" value="0.1"/>
      //   <param name="dz" value="0.1"/>
      //   <param name="drot" value="5.0"/>