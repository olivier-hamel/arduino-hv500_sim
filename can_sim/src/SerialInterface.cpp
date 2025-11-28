#include <SerialInterface.h>

namespace
{
const char* nodeLabel(uint8_t node) {
    switch (node) {
        case Hv500CanNode::NODE_FL : return "FL";
        case Hv500CanNode::NODE_FR : return "FR";
        case Hv500CanNode::NODE_RL : return "RL";
        case Hv500CanNode::NODE_RR : return "RR";
        default: return "??";
    }
}
} // namespace

SerialInterface::SerialInterface(Hv500CanNode& hv500, MotorSimulator* motors[MOTOR_COUNT], Throttle& throttle, uint32_t baud) :
    hv500_(hv500), motors_(motors), throttle_(throttle), baud_(baud) {}

void SerialInterface::begin() {
    serial_.begin(baud_);
    while (!serial_)
    {
        // wait for connection
    }

    serial_.println("Starting...");
    delay(2000);

    
    if (!hv500_.begin(CAN_500KBPS, MCP_16MHZ))
    {
        serial_.println("Failed to initalize MCP2515");
        while (true)
        {
            // No need to go further
        }
    }
    
    serial_.println("Hv500 Motor simulator ready");
    delay(1500);
    printMainMenu();
}

void SerialInterface::update() {
    while (serial_.available()) // Stuck here potentiel**
    {
        char c = static_cast<char>(serial_.read());
        handleInput(c);
    }

    if (state_ == MenuState::Status)
    {
        uint32_t now = millis();
        if (now - lastStatusPrint_ >= STATUS_INTERVAL_MS)
        {
            printMotorStatus();
            lastStatusPrint_ = now;
        }
        
    }
}

void SerialInterface::logCommand(const Hv500CanNode::Command& cmd) {
    serial_.print("[RX] ");
    serial_.print(nodeLabel(cmd.node));
    serial_.print(" ");

    switch (cmd.type)
    {
        case Hv500CanNode::CommandType::DriveEnable :
            serial_.print(" DriveEnable = ");
            serial_.println(cmd.drive_enable ? "ON" : "OFF");
            break;
        
        case Hv500CanNode::CommandType::RelativeCurrent :
            serial_.print(" RelCurrent = ");
            serial_.println(cmd.relative_current, 3);
            break;

        case Hv500CanNode::CommandType::TargetErpm:
            serial_.print(" TargetERPM = ");
            serial_.println(cmd.target_erpm);
            break;
            
        case Hv500CanNode::CommandType::ReadWriteParam:
            serial_.print(" Param addr = ");
            serial_.print(cmd.parameter_address);
            serial_.print(" W=");
            serial_.print(cmd.write ? 1 : 0);
            serial_.print(" data=");
            serial_.println(cmd.parameter_data);
            break;
    }
}

void SerialInterface::printMainMenu() {
    serial_.println();
    serial_.println("=== HV500 Simulator Menu ===");
    serial_.println("1 - Show motor status (live)");
    serial_.println("2 - Inject fault on motor");
    serial_.println("3 - Clear all faults");
    serial_.println("0 - Return to menu / stop status");
    serial_.println("============================");
    serial_.println();
}

void SerialInterface::printMotorStatus() {
    serial_.println("===     Motor Status     ===");

    for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
    {
        MotorSimulator* m = motors_[i];
        serial_.println("-------");
        serial_.print("MOTOR : ");
        serial_.print(nodeLabel(m->node()));
        serial_.print(" Input Throttle= ");
        serial_.print(throttle_.read());
        serial_.println("%");

        serial_.print(nodeLabel(m->node()));
        serial_.print(" : en= ");
        serial_.print(m->getDriveEnabled() ? "true" : "false");
        serial_.print(" RelCurrent= ");
        serial_.print(m->getRelativeCurrent(), 2);
        serial_.print(" tgtRPM= ");
        serial_.print(m->getTargetErpm());
        serial_.print(" curRPM= ");
        serial_.print(m->getCurrentSpeed());
        serial_.print(" angle= ");
        serial_.print(m->getElectricalAngle(), 1);
        if (m->hasFault())
        {
            serial_.print(" FAULT");
        }
        serial_.println();   
    }
    serial_.println();   
}

void SerialInterface::handleInput(char c) {
    switch (c)
    {
        case '0' :
            state_ = MenuState::Main;
            printMainMenu();
            break;
        
        case '1':
            state_ = MenuState::Status;
            serial_.println("Entering live status mode. Press 0 to exit.");
            lastStatusPrint_ = 0;
            break;

        case '2' : 
            // send an fault on motor 0
            motors_[0]->setFault(0x0001, 0, 0, 0);
            serial_.println("Fault injected on FL.");
            break;

        case '3' : 
            for (uint8_t i = 0; i < MOTOR_COUNT; i++)
            {
                motors_[i]->clearFaults();
            }
            serial_.println("All faults cleared");
            break;

        default :
            serial_.println("unknown input");
            break;   
    }
}