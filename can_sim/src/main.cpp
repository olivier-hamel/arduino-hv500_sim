#include <Arduino.h>

#include <Hv500CanNode.h>
#include <MotorSimulator.h>
#include <SerialInterface.h>


// -- Config --
constexpr uint8_t CAN_CS_PIN = 10;
constexpr uint32_t TELEMETRY_PERIOD_MS = 100;
constexpr uint16_t SIMULATION_PERIOD_MS = 10;

Hv500CanNode hv500(CAN_CS_PIN);

MotorSimulator motorFL(Hv500CanNode::NODE_FL);
MotorSimulator motorFR(Hv500CanNode::NODE_FR);
MotorSimulator motorRL(Hv500CanNode::NODE_RL);
MotorSimulator motorRR(Hv500CanNode::NODE_RR);

MotorSimulator* motors[SerialInterface::MOTOR_COUNT] = {&motorFL, &motorFR, &motorRL, &motorRR};

SerialInterface serialUI(hv500, motors, 115200);

uint32_t lastTelemetry  = 0;
uint32_t lastSimulation = 0;

void sendAllTelemetry();
void dispatchCommand(const Hv500CanNode::Command& cmd);

void setup() {
    serialUI.begin();
}

void loop() {
    const uint32_t now = millis();

    // 1. Update motor simulation
    if (now - lastSimulation >= SIMULATION_PERIOD_MS)
    {
        const uint32_t dt = now - lastSimulation;
        for (uint8_t i = 0; i < SerialInterface::MOTOR_COUNT; ++i)
        {
            motors[i]->update(dt);
        }
        lastSimulation = now;        
    }

    // 2. Send telemtry periodically
    if (now - lastTelemetry >= TELEMETRY_PERIOD_MS) {
        sendAllTelemetry();
        lastTelemetry = now;
    }

    // 3. Poll for incoming CAN commands
    Hv500CanNode::Command cmd;
    while (hv500.pollCommand(cmd))
    {
        dispatchCommand(cmd);
        serialUI.logCommand(cmd);
    }

    // 4. Handle serial menu input
    serialUI.update();   
}


void sendAllTelemetry() {
  for (uint8_t i = 0; i < SerialInterface::MOTOR_COUNT; ++i) {
    MotorSimulator* m = motors[i];

    Hv500CanNode::InternalStates states{};
    m->fillInternalStates(states);
    hv500.sendInternalStates(states);

    Hv500CanNode::CurrentInfo currents{};
    m->fillCurrentInfo(currents);
    hv500.sendCurrentInfo(currents);

    Hv500CanNode::VoltageInfo voltages{};
    m->fillVoltageInfo(voltages);
    hv500.sendVoltageInfo(voltages);

    Hv500CanNode::FaultCodes faults{};
    m->fillFaultCodes(faults);
    hv500.sendFaultCodes(faults);

    Hv500CanNode::MotorPositionInfo pos{};
    m->fillMotorPositionInfo(pos);
    hv500.sendMotorPositionInfo(pos);
  }
}

void dispatchCommand(const Hv500CanNode::Command& cmd) {
    if (cmd.type == Hv500CanNode::CommandType::ReadWriteParam)
    {
        for (uint8_t i = 0; i < SerialInterface::MOTOR_COUNT; ++i)
        {
            Hv500CanNode::Command copy = cmd;
            copy.node = motors[i]->node();
            motors[i]->applyCommand(copy);
        }
    } else {
        for (uint8_t i = 0; i < SerialInterface::MOTOR_COUNT; ++i)
        {
            motors[i]->applyCommand(cmd);
        }
        
    }
}