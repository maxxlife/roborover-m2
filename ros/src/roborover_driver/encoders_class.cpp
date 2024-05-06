#include <chrono>
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"
//#include "roborover_driver/encoders_lib.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>


//typedef std::chrono::steady_clock time_source;
using namespace std::chrono_literals;

class EncodersPair : public rclcpp::Node
{
public:
  EncodersPair()
  : Node("encoders"), count_(0)
  {
    left_wheel_angle_pub = this->create_publisher<std_msgs::msg::Float64>("/roborover/left_wheel/angle", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&EncodersPair::timer_callback, this));
  }

  private:
    void timer_callback()
    {
      auto left_wheel_angle_msg = std_msgs::msg::Float64();
      left_wheel_angle_msg.data = 10.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", left_wheel_angle_msg.data);
      left_wheel_angle_pub->publish(left_wheel_angle_msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_angle_pub;
    size_t count_;
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EncodersPair>());
	rclcpp::shutdown();
	return 0;
}
