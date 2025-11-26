#pragma once

#include <Arduino.h>
#include <mcp2515.h>

class Hv500CanNode
{

public:
    static constexpr uint8_t NODE_FL  = 10;
    static constexpr uint8_t NODE_FR  = 11;
    static constexpr uint8_t NODE_RL  = 13;
    static constexpr uint8_t NODE_RR  = 13;
    static constexpr uint8_t NODE_MIN = NODE_FL;
    static constexpr uint8_t NODE_MAX = NODE_RR;

    struct InternalStates
    {
        uint16_t vsm_state      = 0;
        uint8_t inverter_state  = 0;
        bool relay_1_status     = false;
        bool relay_2_status     = false;
        bool relay_3_status     = false;
        bool relay_4_status     = false;
        bool relay_5_status     = false;
        bool relay_6_status     = false;
        bool inverter_run_mode  = false;
        uint8_t inverter_discharge_state = 0;
        bool inverter_command_mode   = false;
        bool inverter_enable_state   = false;
        bool inverter_enable_lockout = false;
        bool direction_command       = false;
        bool bms_active              = false;
        bool bms_torque_limiting     = false;

    };

    struct CurrentInfo
    {
        float phase_a_current = 0.0f;
        float phase_b_current = 0.0f;
        float phase_c_current = 0.0f;
        float dc_bus_current  = 0.0f;
    };

    struct VoltageInfo 
    {
        float dc_bus_voltage = 0.0f;
        float output_voltage = 0.0f;
        float vab_vd_voltage = 0.0f;
        float vbc_vq_voltage = 0.0f;
    };

    struct FaultCodes 
    {
        uint16_t post_fault_lo = 0;
        uint16_t post_fault_hi = 0;
        uint16_t run_fault_lo = 0;
        uint16_t run_fault_hi = 0;
    };

    struct MotorPositionInfo 
    {
        float motor_angle_electrical = 0.0f;
        int16_t motor_speed = 0;
        float electrical_output_frequency = 0.0f;
        float delta_resolver_filtered = 0.0f;
    };

    enum class CommandType 
    {
        DriveEnable,
        RelativeCurrent,
        TargetErpm,
        ReadWriteParam
    };

    struct Command 
    {
        CommandType type;
        uint8_t node = 0;
        bool drive_enable = false;
        float relative_current = 0.0f;
        int32_t target_erpm = 0;
        uint16_t parameter_address = 0;
        bool write = false;
        int16_t parameter_data = 0;
    };

    explicit Hv500CanNode(uint8_t cs_pin);

    bool begin(CAN_SPEED speed = CAN_500KBPS, CAN_CLOCK clock = MCP_16MHZ);

    bool sendInternalStates(const InternalStates& msg);
    bool sendCurrentInfo(const CurrentInfo& msg);
    bool sendVoltageInfo(const VoltageInfo& msg);
    bool sendFaultCodes(const FaultCodes& msg);
    bool sendMotorPositionInfo(const MotorPositionInfo& msg);

    bool pollCommand(Command& out);    

private:
    MCP2515 mcp2515_;
    bool initialized_ = false;

    static constexpr uint16_t DRIVE_ENABLE_BASE_ID = 0x18A;
    static constexpr uint16_t RELATIVE_CURRENT_BASE_ID = 0x0AA;
    static constexpr uint16_t TARGET_ERPM_BASE_ID = 0x06A;
    static constexpr uint16_t READ_WRITE_PARAM_ID = 0x0C1;
    static constexpr uint16_t INTERNAL_STATES_ID = 0x0AA;
    static constexpr uint16_t CURRENT_INFO_ID = 0x0A6;
    static constexpr uint16_t VOLTAGE_INFO_ID = 0x0A7;
    static constexpr uint16_t FAULT_CODES_ID = 0x0AB;
    static constexpr uint16_t MOTOR_POSITION_ID = 0x0A5;
    static constexpr uint8_t MAX_NODE_COUNT = NODE_MAX - NODE_MIN + 1;
    static constexpr uint16_t STANDARD_ID_MASK = 0x7FF;

    bool sendFrame(uint16_t id, uint8_t dlc, const uint8_t* data);
    static uint8_t nodeFromOffset(uint8_t offset);
    static bool decodeNodeFromId(uint16_t id, uint16_t base, uint8_t& node);

    static void packInternalStates(const InternalStates& msg, uint8_t (&buffer)[8]);
    static void packCurrentInfo(const CurrentInfo& msg, uint8_t (&buffer)[8]);
    static void packVoltageInfo(const VoltageInfo& msg, uint8_t (&buffer)[8]);
    static void packFaultCodes(const FaultCodes& msg, uint8_t (&buffer)[8]);
    static void packMotorPositionInfo(const MotorPositionInfo& msg, uint8_t (&buffer)[8]);

    static bool unpackDriveEnable(const can_frame& frame, Command& out);
    static bool unpackRelativeCurrent(const can_frame& frame, Command& out);
    static bool unpackTargetErpm(const can_frame& frame, Command& out);
    static bool unpackReadWriteParam(const can_frame& frame, Command& out);
    
};
