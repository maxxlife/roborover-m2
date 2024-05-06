#include <chrono>
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "encoders_lib.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>


//typedef std::chrono::steady_clock time_source;
using namespace std::chrono_literals;
typedef std::chrono::steady_clock time_source;

class EncodersPair : public rclcpp::Node
{
public:
  EncodersPair()
  : Node("encoders"), encoder_left(ENCODER_1_PIN_A, ENCODER_1_PIN_B, &EncoderWiringPiISR::encoderISR1, &EncoderWiringPiISR::encoderPosition1)
  , encoder_right(ENCODER_2_PIN_A, ENCODER_2_PIN_B, &EncoderWiringPiISR::encoderISR2, &EncoderWiringPiISR::encoderPosition2)
  {
    left_wheel_angle_pub = this->create_publisher<std_msgs::msg::Float64>("/roborover/left_wheel/angle", 10);
    encoders_timer = this->create_wall_timer(0.01ms, std::bind(&EncodersPair::encodersCallback, this));
  }

  private:
  	EncoderWiringPi encoder_left;
    EncoderWiringPi encoder_right;
    std_msgs::msg::Float64 left_wheel_angle_msg = std_msgs::msg::Float64();
    double left_wheel_angle;
    rclcpp::TimerBase::SharedPtr encoders_timer;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_angle_pub;
    time_source::time_point last_time;
    void encodersCallback();
};

void EncodersPair::encodersCallback()
{
    time_source::time_point this_time = time_source::now();
    std::chrono::duration<double> elapsed_duration = this_time - last_time;
    printf("elapsed %f\n", elapsed_duration.count());
    left_wheel_angle = -1 * encoder_left.getAngle();
    left_wheel_angle_msg.data = left_wheel_angle;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", left_wheel_angle_msg.data);
    left_wheel_angle_pub->publish(left_wheel_angle_msg);
}

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EncodersPair>());
	rclcpp::shutdown();
	return 0;
}
