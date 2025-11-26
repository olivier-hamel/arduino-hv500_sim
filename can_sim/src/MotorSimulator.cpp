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
    } else {
        // Slowly ramp to target ERPM
        const float commandedErpm = static_cast<float>(targetErpm_) * relativeCurrent_;
        const float delta = commandedErpm - static_cast<float>(currentSpeed_);
        const float maxStep = ERPM_RAMP_RATE * dt_s;

        if (fabsf(delta) < maxStep)
        {
            currentSpeed_ = static_cast<int16_t>(commandedErpm);
        } else if (delta > 0) {
            currentSpeed_ += static_cast<int16_t>(maxStep);
        } else {
            currentSpeed_ -= static_cast<int16_t>(maxStep);
        }

        dcBusCurrent_ = fabsf(relativeCurrent_) * 50.0f;

        const float electricalFreqHz = (fabsf(static_cast<float>(currentSpeed_)) / 60.0f) * POLE_PAIRS;
        electricalAngle_ += electricalFreqHz * 360.0f * dt_s;
        while (electricalAngle_ >= 360.0f)
        {
            electricalAngle_ -= 360.0f;
        } 
    }
}

void MotorSimulator::fillInternalStates(Hv500CanNode::InternalStates& out) const {
    out = {};

    out.vsm_state = driveEnabled_ ? 6 : 0;
    out.inverter_state = driveEnabled_ ? 4 : 0;
    out.inverter_enable_state = driveEnabled_;
    out.inverter_run_mode = driveEnabled_;
    out.inverter_enable_lockout = hasFault();
    out.bms_active = true;
}

void MotorSimulator::fillCurrentInfo(Hv500CanNode::CurrentInfo& out) const {
    out = {};

    out.phase_a_current = dcBusCurrent_ * 0.577f; // +/- 1/sqrt(3)
    out.phase_b_current = -dcBusCurrent_ * 0.289f;
    out.phase_c_current = -dcBusCurrent_ * 0.289f;
    out.dc_bus_current = dcBusCurrent_;
}

void MotorSimulator::fillVoltageInfo(Hv500CanNode::VoltageInfo& out) const {
    out = {};

    out.dc_bus_voltage = dcBusVoltage_;
    out.output_voltage = dcBusVoltage_ * 0.5f;
    out.vab_vd_voltage = 0.0f;
    out.vbc_vq_voltage = 0.0f;
}

void MotorSimulator::fillFaultCodes(Hv500CanNode::FaultCodes& out) const {
    out.post_fault_lo = postFaultLo_;
    out.post_fault_hi = postFaultHi_;
    out.run_fault_lo = runFaultLo_;
    out.run_fault_hi = runFaultHi_;
}

void MotorSimulator::fillMotorPositionInfo(Hv500CanNode::MotorPositionInfo& out) const {
  out = {};
  out.motor_angle_electrical = electricalAngle_;
  out.motor_speed = currentSpeed_;
  out.electrical_output_frequency = (fabsf(static_cast<float>(currentSpeed_)) / 60.0f) * POLE_PAIRS;
  out.delta_resolver_filtered = 0.0f;
}

void MotorSimulator::setFault(uint16_t postLo, uint16_t postHi, uint16_t runLo, uint16_t runHi) {
  postFaultLo_ = postLo;
  postFaultHi_ = postHi;
  runFaultLo_ = runLo;
  runFaultHi_ = runHi;
}

void MotorSimulator::clearFaults() {
  postFaultLo_ = 0;
  postFaultHi_ = 0;
  runFaultLo_ = 0;
  runFaultHi_ = 0;
}

bool MotorSimulator::hasFault() const {
  return (postFaultLo_ | postFaultHi_ | runFaultLo_ | runFaultHi_) != 0;
}