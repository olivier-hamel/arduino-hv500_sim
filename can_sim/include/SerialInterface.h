#pragma once

#include <Arduino.h>
#include <Hv500CanNode.h>
#include <MotorSimulator.h>

class SerialInterface
{
    
public:
    static constexpr uint8_t MOTOR_COUNT = 4;

    SerialInterface(Hv500CanNode& hv500, MotorSimulator* motors[MOTOR_COUNT], uint32_t baud = 115200);

    void begin();
    void update();
    void logCommand(const Hv500CanNode::Command& cmd);

private :
    enum class MenuState {
        Main, 
        Status,
        InjectFault,
        ClearFault
    };

    void printMainMenu();
    void printMotorStatus();
    void handleInput(char c);

    HardwareSerial& serial_ = Serial;
    Hv500CanNode& hv500_;
    MotorSimulator** motors_;
    uint32_t baud_;

    MenuState state_ = MenuState::Main;
    uint32_t lastStatusPrint_ = 0;
    static constexpr uint32_t STATUS_INTERVAL_MS = 500;
};