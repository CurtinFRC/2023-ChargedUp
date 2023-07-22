#pragma once 

#include "Gearbox.h"

#include "behaviour/HasBehaviour.h"
#include <string>
#include <units/math.h>
#include <units/velocity.h>
#include <units/charge.h>

enum class IntakeStates {
    kIntake,
    kOuttake,
    kIdle
};

struct IntakeConfig {
    wom::MotorVoltageController *intakeMotor;
    wom::MotorVoltageController *intakeMotor2;
};

class Intake : public behaviour::HasBehaviour {
    public:
        // Constructor
        Intake(IntakeConfig config);
        // Methods
        void OnUpdate(units::second_t dt);
        void SetIntake();
        void SetOuttake();
        void SetIdle();
        std::string GetState();

    
    private:
        // Variables
        IntakeConfig _config;
        IntakeStates _state = IntakeStates::kIdle;
};

