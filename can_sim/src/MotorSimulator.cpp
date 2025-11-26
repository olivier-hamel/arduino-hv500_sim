#include <MotorSimulator.h>
#include <math.h>

namespace {
    constexpr float MAX_ERPM       = 10000.0f;
    constexpr float ERPM_RAMP_RATE = 2000.0f;
    constexpr float POLE_PAIRS     = 4.0f;
}   // namespace

MotorSimulator::MotorSimulator(uint8_t node) : node_(node) {}

void MotorSimulator::applyCommand(const Hv500CanNode::Command& cmd) {
    if (cmd.node != node_) {
        return;
    }

    switch (cmd.type)
    {
        case Hv500CanNode::CommandType::DriveEnable :
            driveEnabled_ = cmd.drive_enable;
            if (!driveEnabled_)
            {
                relativeCurrent_ = 0.0f;
            }
            break;
        
        case Hv500CanNode::CommandType::RelativeCurrent:
            relativeCurrent_ = cmd.relative_current;
            break;
        
        case Hv500CanNode::CommandType::TargetErpm : 
            targetErpm_ = cmd.target_erpm;
            break;
        
        case Hv500CanNode::CommandType::ReadWriteParam :
            // Param 20 with write==true and data==0 is a fault reset 
            if (cmd.parameter_address == 20 && cmd.write && cmd.parameter_data == 0)
            {
                clearFaults();
            }
            break;       
    }
}

void MotorSimulator::update(uint32_t dt_ms) {
    const float dt_s = static_cast<float>(dt_ms) / 1000.0f;

    // Slow down to a stop if disabled or faulted
    if (!driveEnabled_ || hasFault()) 
    {
        if (currentSpeed_ > 0)
        {
            currentSpeed_ -= static_cast<int16_t>(ERPM_RAMP_RATE * dt_s);
            if (currentSpeed_ < 0)
            {
                currentSpeed_ = 0;
            }
        } else if (currentSpeed_ < 0) {
            currentSpeed_ += static_cast<int16_t>(ERPM_RAMP_RATE * dt_s);
            if (currentSpeed_ > 0)
            {
                currentSpeed_ = 0;
            }
            
        }
        
    }
    
}