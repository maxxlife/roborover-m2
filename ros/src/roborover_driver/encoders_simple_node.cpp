#include <chrono>
#include <functional>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"


int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("encoders");
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_angle_pub;
	rclcpp::TimerBase::SharedPtr encoders_timer;
	std_msgs::msg::Float64 left_wheel_angle_msg;
	
	double left_wheel_angle=5.0;

	while (rclcpp::ok()) {
		left_wheel_angle_pub = node->create_publisher<std_msgs::msg::Float64>("/roborover/left_wheel/angle", 1);
		left_wheel_angle_msg.data=left_wheel_angle;
		left_wheel_angle_pub->publish(left_wheel_angle_msg);
		rclcpp::spin_some(node);
		RCLCPP_INFO_ONCE(node->get_logger(), "Jebanyi Loch");
	}

	return 0;
}

