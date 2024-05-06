#include <chrono>
#include <functional>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "encoders_lib.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"


//typedef std::chrono::steady_clock time_source;
using namespace std::chrono_literals;

class EncodersPair {
public:
	EncodersPair(double update_rate);
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("encoders");
private:
	
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_angle_pub;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_angle_pub;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_velocity_pub;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_velocity_pub;
	
	//rclcpp::Time encoders_timer;
	rclcpp::TimerBase::SharedPtr encoders_timer;
	//std::shared_ptr<rclcpp::CallbackGroup> timer_cb_group_;
	
	std_msgs::msg::Float64 left_wheel_angle_msg;
	std_msgs::msg::Float64 right_wheel_angle_msg;
	std_msgs::msg::Float64 left_wheel_velocity_msg;
	std_msgs::msg::Float64 right_wheel_velocity_msg;

	EncoderWiringPi encoder_left;
	EncoderWiringPi encoder_right;

	double left_wheel_angle;
	double right_wheel_angle;
	double left_wheel_velocity;
	double right_wheel_velocity;
	double left_wheel_position;
	double right_wheel_position;
	
	//time_source::time_point last_time;

	void encodersCallback(const rclcpp::TimerBase& event);

};

EncodersPair::EncodersPair(double update_rate)
	: encoder_left(ENCODER_1_PIN_A, ENCODER_1_PIN_B, &EncoderWiringPiISR::encoderISR1, &EncoderWiringPiISR::encoderPosition1)
	, encoder_right(ENCODER_2_PIN_A, ENCODER_2_PIN_B, &EncoderWiringPiISR::encoderISR2, &EncoderWiringPiISR::encoderPosition2) {
	left_wheel_angle_pub = node->create_publisher<std_msgs::msg::Float64>("/roborover/left_wheel/angle", 1000);
	right_wheel_angle_pub = node->create_publisher<std_msgs::msg::Float64>("/roborover/right_wheel/angle", 1);
	left_wheel_velocity_pub = node->create_publisher<std_msgs::msg::Float64>("/roborover/left_wheel/current_velocity", 1);
	right_wheel_velocity_pub = node->create_publisher<std_msgs::msg::Float64>("/roborover/right_wheel/current_velocity", 1);
	left_wheel_angle_msg.data = 10.0;
	RCLCPP_INFO(node->get_logger(), "Publishing: '%f'", left_wheel_angle_msg.data);
	left_wheel_angle_pub->publish<std_msgs::msg::Float64>(left_wheel_angle_msg);

	encoders_timer = node->create_wall_timer(10ms, std::bind(&EncodersPair::encodersCallback, this));
	//encoders_timer = node.createTimer(ros::Duration(update_rate), &EncodersPair::encodersCallback, this);
	//encoders_timer = node->create_wall_timer(10ms, &EncodersPair::encodersCallback, this);
	//timer_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
	//encoders_timer = create_wall_timer(5, std::bind(&EncodersPair::encodersCallback, this));
	//encoders_timer = rclcpp::create_wall_timer();

}

void EncodersPair::encodersCallback(const rclcpp::TimerBase& event) {
	printf("callback\n");
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  EncodersPair encoders_pair(0.01);
  rclcpp::spin(encoders_pair.node);
  rclcpp::shutdown();

	return 0;
}
