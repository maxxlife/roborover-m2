#include <rclcpp/rclcpp.hpp>
#include "serialib.h"
#include "serialib.cpp"
#include <unistd.h>
#include <stdio.h>

#define ENCODER_1_PIN_A 0
#define ENCODER_1_PIN_B 1
#define ENCODER_2_PIN_A 2
#define ENCODER_2_PIN_B 3
#define PULSES_PER_REVOLUTION 1920
#define SERIAL_PORT "/dev/ttyACM0"

namespace EncoderWiringPiISR
{

    // Connection to serial port
    serialib serial;
    char buffer[6];
    char errorOpening = serial.openDevice(SERIAL_PORT, 115200);

   uint8_t digitalRead(uint8_t pin) {
        serial.readString(buffer, '\r\n', 6, 2000);
        // printf("String read: %s", buffer);
        uint8_t pin_val = buffer[pin] - '0';
        
        return pin_val;
    }
    // printf("Successful connection to %s\n", SERIAL_PORT);

    volatile long encoderPosition1;
    volatile long encoderPosition2;
    volatile uint8_t encoderState1;
    volatile uint8_t encoderState2;

    void encoderISR(const int pinA, const int pinB, volatile long &encoderPosition, volatile uint8_t &encoderState)
    {
        uint8_t valA = digitalRead(pinA);
        uint8_t valB = digitalRead(pinB);
        printf("valA is %s , valB is %s\n", valA, valB);
        uint8_t s = encoderState & 3;
        if (valA)
            s |= 4;
        if (valB)
            s |= 8;
        encoderState = (s >> 2);
        if (s == 1 || s == 7 || s == 8 || s == 14)
            encoderPosition++;
        else if (s == 2 || s == 4 || s == 11 || s == 13)
            encoderPosition--;
        else if (s == 3 || s == 12)
            encoderPosition += 2;
        else if (s == 6 || s == 9)
            encoderPosition -= 2;
    }

    void encoderISR1(void)
    {
        encoderISR(ENCODER_1_PIN_A, ENCODER_1_PIN_B, encoderPosition1, encoderState1);
    }

    void encoderISR2(void)
    {
        encoderISR(ENCODER_2_PIN_A, ENCODER_2_PIN_B, encoderPosition2, encoderState2);
    }
}

class EncoderWiringPi
{
public:
    EncoderWiringPi(const int &pinA, const int &pinB, void (*isrFunction)(void), volatile long *encoderPosition);
    double getAngle();

private:
    int _pinA;
    int _pinB;
    volatile long *_encoderPosition;
    double _initial_angle;
    double ticks2Angle(long position);
};

EncoderWiringPi::EncoderWiringPi(const int &pinA, const int &pinB, void (*isrFunction)(void), volatile long *encoderPosition)
{
    _encoderPosition = encoderPosition;
    _pinA = pinA;
    _pinB = pinB;

    _initial_angle = ticks2Angle(*_encoderPosition);
    // RCLCPP_INFO("Encoder wiringPi: ISR setup");
}

double EncoderWiringPi::getAngle()
{
    double current_angle = ticks2Angle(*_encoderPosition);
    return current_angle - _initial_angle;
}

double EncoderWiringPi::ticks2Angle(long position)
{
    return position * ((double)2 * M_PI / PULSES_PER_REVOLUTION / 2);
}
