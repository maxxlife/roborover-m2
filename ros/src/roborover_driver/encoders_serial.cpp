#include <chrono>
#include <functional>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "serialib.h"

#define SERIAL_PORT "/dev/ttyACM0"

using namespace std;
// Serial object
serialib serial;
char buffer[26];
// char a[];

std::vector<std::string> split(const std::string &s, char delim)
{
    std::vector<std::string> result;
    std::stringstream ss(s);
    std::string item;

    while (getline(ss, item, delim))
    {
        result.push_back(item);
    }

    return result;
}

int main(int argc, char **argv)
{
    // Connection to serial port
    char errorOpening = serial.openDevice(SERIAL_PORT, 115200);

    // If connection fails, return the error code otherwise, display a success message
    if (errorOpening != 1)
        return errorOpening;
    // RCLCPP_INFO_ONCE(node->get_logger(), "Successful connection to Serial Port");
    printf("Successful connection to %s\n", SERIAL_PORT);
    std::string str;
    // Create the string
    char buffer_write[11] = "60,40\n";//left, right

    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("encoders");
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_velocity_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_angle_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_velocity_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_angle_pub;
    rclcpp::TimerBase::SharedPtr encoders_timer;
    std_msgs::msg::Float64 left_wheel_velocity_msg;
    std_msgs::msg::Float64 left_wheel_angle_msg;
    std_msgs::msg::Float64 right_wheel_velocity_msg;
    std_msgs::msg::Float64 right_wheel_angle_msg;

    double left_wheel_velocity;
    double left_wheel_angle;
    double right_wheel_velocity;
    double right_wheel_angle;
    unsigned char received[1];
    // char *a;
    while (rclcpp::ok())
    {
        /*serial.readBytes(received, 1, 2000, 1000);
        if (received[0] == 'P')
        {
            printf("P received\n");
        }*/
        // std::this_thread::sleep_for(std::chrono::milliseconds(5));
        // serial.readChar(a, 1000);
        if (serial.available())
        {
            serial.readString(buffer, '\r\n', 27, 2000);
            //21 is string length "0.00,0.00,0.00,0.00\r\n"
            size_t len = strlen(buffer);
            //printf("String length %d\n", len);
            if (len >=21)
            {
                // Read the bytes

                // std::this_thread::sleep_for(std::chrono::milliseconds(5));
                // serial.writeString(buffer_write);
                // serial.flushReceiver();
                // serial.writeChar('P');
                // serial.flushReceiver();
                printf("String read: %s", buffer);
                str = buffer;
                std::vector<std::string> v = split(str, ',');
                left_wheel_velocity = stod(v[0]);
                left_wheel_angle = stod(v[1]);
                right_wheel_velocity = stod(v[2]);
                right_wheel_angle = stod(v[3]);
                // printf("left_wheel_speed %f\n", left_wheel_speed);
                left_wheel_velocity_pub = node->create_publisher<std_msgs::msg::Float64>("/roborover/left_wheel/current_velocity", 1);
                left_wheel_angle_pub = node->create_publisher<std_msgs::msg::Float64>("/roborover/left_wheel/angle", 1);
                right_wheel_velocity_pub = node->create_publisher<std_msgs::msg::Float64>("/roborover/right_wheel/current_velocity", 1);
                right_wheel_angle_pub = node->create_publisher<std_msgs::msg::Float64>("/roborover/right_wheel/angle", 1);
            
                left_wheel_velocity_msg.data = left_wheel_velocity;
                left_wheel_angle_msg.data = left_wheel_angle;
                right_wheel_velocity_msg.data = right_wheel_velocity;
                right_wheel_angle_msg.data = right_wheel_angle;

                left_wheel_velocity_pub->publish(left_wheel_velocity_msg);
                left_wheel_angle_pub->publish(left_wheel_angle_msg);
                right_wheel_velocity_pub->publish(right_wheel_velocity_msg);
                right_wheel_angle_pub->publish(right_wheel_angle_msg);
                rclcpp::spin_some(node);
                RCLCPP_INFO_ONCE(node->get_logger(), "Running encoders node");
            }
        }

        serial.writeString(buffer_write);

        // std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Close the serial device
    serial.closeDevice();

    return 0;
}
