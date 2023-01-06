#pragma once

#include "behaviour/HasBehaviour.h"

#include "Gearbox.h"

struct ClimberConfig {
    wom::Gearbox gearbox;

    units::volt_t ascendingVoltage = 12_V;
    units::volt_t descendingVoltage = -5_V;
    units::volt_t idleVoltage = 0_V;
    units::volt_t fixedVoltage = 6_V;
    units::volt_t climbedVoltage = 12_V;
};

enum class ClimberState {
    kIdle,
    kAscending,
    kDescending,
    kFixed,
    kClimbed
};

class Climber : public behaviour::HasBehaviour {
    public :
        Climber(ClimberConfig config);

        void OnUpdate(units::second_t dt);

        void SetAscending();

        void SetDescending();

        void SetFixed();

        void SetClimbed();

        ClimberState GetState() const;

    private :
        ClimberConfig _config;
        ClimberState _state = ClimberState::kIdle;

};