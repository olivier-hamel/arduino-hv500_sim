#pragma once

#include <Arduino.h>
#include <Hv500CanNode.h>

/*
 * Simule un Hv500.
 * Produit les données de télémetry selon les commandes reçues. 
 */

class MotorSimulator {
    public :
        explicit MotorSimulator(uint8_t node);

        uint8_t node() const {return node_; }

        void applyCommand(const Hv500CanNode::Command& cmd);

        void update(uint32_t dt_ms);

        void fillInternalStates(Hv500CanNode::InternalStates& out) const;
        void fillCurrentInfo(Hv500CanNode::CurrentInfo& out) const;
        void fillVoltageInfo(Hv500CanNode::VoltageInfo& out) const;
        void fillFaultCodes(Hv500CanNode::FaultCodes& out) const;
        void fillMotorPositionInfo(Hv500CanNode::MotorPositionInfo& out) const;

        bool getDriveEnabled() const {return driveEnabled_; }
        float getRelativeCurrent() const {return relativeCurrent_; }
        int32_t getTargetErpm() const { return targetErpm_; }

        int16_t getCurrentSpeed() const { return currentSpeed_; }
        float getElectricalAngle() const { return electricalAngle_; }

        void setFault(uint16_t postLo, uint16_t postHi, uint16_t runLo, uint16_t runHi);
        void clearFaults();
        bool hasFault() const;

    private :
        uint8_t node_;

        bool driveEnabled_     = false;
        float relativeCurrent_ = 0.0f;
        int32_t targetErpm_    = 0;

        int16_t currentSpeed_  = 0;
        float electricalAngle_ = 0.0f;

        float dcBusVoltage_ = 400.0f;
        float dcBusCurrent_ = 0.0f;

        uint16_t postFaultLo_ = 0;
        uint16_t postFaultHi_ = 0;
        uint16_t runFaultLo_  = 0;
        uint16_t runFaultHi_  = 0;

};