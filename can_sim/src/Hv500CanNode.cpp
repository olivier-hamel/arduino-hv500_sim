#include <Hv500CanNode.h>

#include <SPI.h>
#include <string.h>
#include <math.h>
#include <limits.h>

namespace {
constexpr float RELCURRENT_SCALE = 0.1f;
constexpr float CURRENT_SCALE    = 0.1f;
constexpr float VOLTAGE_SCALE    = 0.1f;
constexpr float ANGLE_SCALE      = 0.1f;
constexpr float FREQUENCY_SCALE  = 0.1f;

inline int16_t floatToInt16(float value, float scale) {
    const float scaled = value / scale; 

    if (scaled > static_cast<float>(INT16_MAX))
    {
        return INT16_MAX;
    }

    if (scaled < static_cast<float>(INT16_MIN))
    {
        return INT16_MIN;
    }

    return static_cast<int16_t>(lrintf(scaled));        
}

inline uint16_t floatToUint16(float value, float scale) {
    const float scaled = value / scale;
    if (scaled < 0.0f) 
    {
        return 0;
    }
    if (scaled > static_cast<float>(UINT16_MAX)) 
    {
        return UINT16_MAX;
    }
    return static_cast<uint16_t>(lrintf(scaled));
}

inline void writeUint16BE(uint8_t* buffer, uint16_t value) {
    buffer[0] = static_cast<uint8_t>(value >> 8);
    buffer[1] = static_cast<uint8_t>(value);
}

inline void writeInt16BE(uint8_t* buffer, int16_t value) {
    buffer[0] = static_cast<uint8_t>(value >> 8);
    buffer[1] = static_cast<uint8_t>(value & 0xFF);
}

inline void writeInt16LE(uint8_t* buffer, int16_t value) {
    buffer[0] = static_cast<uint8_t>(value);
    buffer[1] = static_cast<uint8_t>(value >> 8);
}

inline void writeUint16LE(uint8_t* buffer, uint16_t value) {
    buffer[0] = static_cast<uint8_t>(value);
    buffer[1] = static_cast<uint8_t>(value >> 8);
}

inline int16_t readInt16BE(const uint8_t* buffer) {
    return static_cast<int16_t>((buffer[0] << 8) | buffer[1]);
}

inline int32_t readInt32BE(const uint8_t* buffer) {
    return static_cast<int32_t>((static_cast<uint32_t>(buffer[0]) << 24) |
                                (static_cast<uint32_t>(buffer[1]) << 16) |
                                (static_cast<uint32_t>(buffer[2]) << 8) |
                                static_cast<uint32_t>(buffer[3]));
}

inline uint16_t readUint16LE(const uint8_t* buffer) {
    return static_cast<uint16_t>((buffer[1] << 8) | buffer[0]);
}

inline int16_t readInt16LE(const uint8_t* buffer) {
    return static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
}

}   // namespace

Hv500CanNode::Hv500CanNode(uint8_t cs_pin) : mcp2515_(cs_pin) {}

bool Hv500CanNode::begin(CAN_SPEED speed, CAN_CLOCK clock) {
    SPI.begin();
    mcp2515_.reset();
    if (mcp2515_.setBitrate(speed, clock) != MCP2515::ERROR_OK)
    {
        return false;
    }
    if (mcp2515_.setNormalMode() != MCP2515::ERROR_OK)
    {
        return false;
    }
    initialized_ = true;
    return true;    
}

bool Hv500CanNode::sendInternalStates(const InternalStates& msg) {
    uint8_t buffer[8];
    packInternalStates(msg, buffer);
    return sendFrame(INTERNAL_STATES_ID, 8, buffer);
}

bool Hv500CanNode::sendCurrentInfo(const CurrentInfo& msg) {
    uint8_t buffer[8];
    packCurrentInfo(msg, buffer);
    return sendFrame(CURRENT_INFO_ID, 8, buffer);
}

bool Hv500CanNode::sendVoltageInfo(const VoltageInfo& msg) {
    uint8_t buffer[8];
    packVoltageInfo(msg, buffer);
    return sendFrame(VOLTAGE_INFO_ID, 8, buffer);
}

bool Hv500CanNode::sendFaultCodes(const FaultCodes& msg) {
    uint8_t buffer[8];
    packFaultCodes(msg, buffer);
    return sendFrame(FAULT_CODES_ID, 8, buffer);
}

bool Hv500CanNode::sendMotorPositionInfo(const MotorPositionInfo& msg) {
    uint8_t buffer[8];
    packMotorPositionInfo(msg, buffer);
    return sendFrame(MOTOR_POSITION_ID, 8, buffer);
}

bool Hv500CanNode::pollCommand(Command &out) {
    if (!initialized_)
    {
        return false;
    }
    
    can_frame frame{};
    const auto err = mcp2515_.readMessage(&frame);
    if (err != MCP2515::ERROR_OK) {
        return false;
    }

    if (unpackDriveEnable(frame, out))
    {
        return true;
    }
    if (unpackRelativeCurrent(frame, out))
    {
        return true;
    }
    if (unpackTargetErpm(frame, out))
    {
        return true;
    }
    if (unpackReadWriteParam(frame, out))
    {
        return true;
    }

    return false;
}

bool Hv500CanNode::sendFrame(uint16_t id, uint8_t dlc, const uint8_t* data) {
    if (!initialized_)
    {
        return false;
    }

    can_frame frame{};
    frame.can_id = id & STANDARD_ID_MASK;
    frame.can_dlc = dlc;
    memcpy(frame.data, data, dlc);

    return (mcp2515_.sendMessage(&frame) == MCP2515::ERROR_OK);
}

uint8_t Hv500CanNode::nodeFromOffset(uint8_t offset) {
    return static_cast<uint8_t>(NODE_MIN + offset);
}

bool Hv500CanNode::decodeNodeFromId(uint16_t id, uint16_t base, uint8_t& node) {
    if (id < base) {
        return false;
    }

    const uint16_t offset = id - base;
    if (offset > MAX_NODE_COUNT)
    {
        return false;
    }

    node = nodeFromOffset(static_cast<uint8_t>(offset));
    return true;    
}

void Hv500CanNode::packInternalStates(const InternalStates& msg, uint8_t (&buffer)[8]) {
    memset(buffer, 0, sizeof(buffer));

    buffer[0] = static_cast<uint8_t>(msg.vsm_state & 0xFF);
    buffer[1] = static_cast<uint8_t>((msg.vsm_state >> 8) & 0xFF);
    buffer[2] = msg.inverter_state;

    buffer[3] |= (msg.relay_1_status ? 1U : 0U) << 0;
    buffer[3] |= (msg.relay_2_status ? 1U : 0U) << 1;
    buffer[3] |= (msg.relay_3_status ? 1U : 0U) << 2;
    buffer[3] |= (msg.relay_4_status ? 1U : 0U) << 3;
    buffer[3] |= (msg.relay_5_status ? 1U : 0U) << 4;
    buffer[3] |= (msg.relay_6_status ? 1U : 0U) << 5;

    buffer[4] |= (msg.inverter_run_mode ? 1U : 0U) << 0;
    buffer[4] |= (msg.inverter_discharge_state & 0x7U) << 5;

    buffer[5] |= (msg.inverter_command_mode ? 1U : 0U) << 0;

    buffer[6] |= (msg.inverter_enable_state ? 1U : 0U) << 0;
    buffer[6] |= (msg.inverter_enable_lockout ? 1U : 0U) << 7;

    buffer[7] |= (msg.direction_command ? 1U : 0U) << 0;
    buffer[7] |= (msg.bms_active ? 1U : 0U) << 1;
    buffer[7] |= (msg.bms_torque_limiting ? 1U : 0U) << 2;
}


void Hv500CanNode::packCurrentInfo(const CurrentInfo& msg, uint8_t (&buffer)[8]) {
    memset(buffer, 0, sizeof(buffer));
    writeInt16BE(&buffer[0], floatToInt16(msg.phase_a_current, CURRENT_SCALE));
    writeInt16BE(&buffer[2], floatToInt16(msg.phase_b_current, CURRENT_SCALE));
    writeInt16BE(&buffer[4], floatToInt16(msg.phase_c_current, CURRENT_SCALE));
    writeInt16BE(&buffer[6], floatToInt16(msg.dc_bus_current, CURRENT_SCALE));
}

void Hv500CanNode::packVoltageInfo(const VoltageInfo& msg, uint8_t (&buffer)[8]) {
    memset(buffer, 0, sizeof(buffer));
    writeInt16BE(&buffer[0], floatToInt16(msg.dc_bus_voltage, VOLTAGE_SCALE));
    writeInt16BE(&buffer[2], floatToInt16(msg.output_voltage, VOLTAGE_SCALE));
    writeInt16BE(&buffer[4], floatToInt16(msg.vab_vd_voltage, VOLTAGE_SCALE));
    writeInt16BE(&buffer[6], floatToInt16(msg.vbc_vq_voltage, VOLTAGE_SCALE));
}

void Hv500CanNode::packFaultCodes(const FaultCodes& msg, uint8_t (&buffer)[8]) {
    memset(buffer, 0, sizeof(buffer));
    writeUint16BE(&buffer[0], msg.post_fault_lo);
    writeUint16BE(&buffer[2], msg.post_fault_hi);
    writeUint16BE(&buffer[4], msg.run_fault_lo);
    writeUint16BE(&buffer[6], msg.run_fault_hi);
}

void Hv500CanNode::packMotorPositionInfo(const MotorPositionInfo& msg, uint8_t (&buffer)[8]) {
    memset(buffer, 0, sizeof(buffer));
    writeUint16BE(&buffer[0], floatToUint16(msg.motor_angle_electrical, ANGLE_SCALE));
    writeInt16BE(&buffer[2], msg.motor_speed);
    writeInt16BE(&buffer[4], floatToInt16(msg.electrical_output_frequency, FREQUENCY_SCALE));
    writeInt16BE(&buffer[6], floatToInt16(msg.delta_resolver_filtered, FREQUENCY_SCALE));
}

bool Hv500CanNode::unpackDriveEnable(const can_frame& frame, Command& out) {
    const uint16_t id = static_cast<uint16_t>(frame.can_id & STANDARD_ID_MASK);
    uint8_t node = 0;
    if (!decodeNodeFromId(id, DRIVE_ENABLE_BASE_ID, node))
    {
        return false;
    }
    if (frame.can_dlc < 1)
    {
        return false;
    }

    out.type = CommandType::DriveEnable;
    out.node = node;
    out.drive_enable = (frame.data[0] != 0);
    return true;    
}


bool Hv500CanNode::unpackRelativeCurrent(const can_frame& frame, Command& out) {
    const uint16_t id = static_cast<uint16_t>(frame.can_id & STANDARD_ID_MASK);
    uint8_t node = 0;
    if (!decodeNodeFromId(id, RELATIVE_CURRENT_BASE_ID, node)) {
        return false;
    }
    if (frame.can_dlc < 2) {
        return false;
    }

    const int16_t raw = readInt16BE(frame.data);
    out.type = CommandType::RelativeCurrent;
    out.node = node;
    out.relative_current = static_cast<float>(raw) * RELCURRENT_SCALE;
    return true;
}


bool Hv500CanNode::unpackTargetErpm(const can_frame& frame, Command& out) {
    const uint16_t id = static_cast<uint16_t>(frame.can_id & STANDARD_ID_MASK);
    uint8_t node = 0;
    if (!decodeNodeFromId(id, TARGET_ERPM_BASE_ID, node)) {
        return false;
    }
    if (frame.can_dlc < 4) {
        return false;
    }

    const int32_t raw = readInt32BE(frame.data);
    out.type = CommandType::TargetErpm;
    out.node = node;
    out.target_erpm = raw;
    return true;
}

bool Hv500CanNode::unpackReadWriteParam(const can_frame& frame, Command& out) {
    const uint16_t id = static_cast<uint16_t>(frame.can_id & STANDARD_ID_MASK);
    if (id != READ_WRITE_PARAM_ID) {
        return false;
    }
    if (frame.can_dlc < 6) {
        return false;
    }

    out.type = CommandType::ReadWriteParam;
    out.node = 0;
    out.parameter_address = readUint16LE(&frame.data[0]);
    out.write = (frame.data[2] & 0x01U) != 0;
    out.parameter_data = readInt16LE(&frame.data[4]);
    return true;
}