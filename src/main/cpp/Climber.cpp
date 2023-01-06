#include "Climber.h"

using namespace wom;

Climber::Climber(ClimberConfig config) : _config(config) {}

void Climber::OnUpdate(units::second_t dt) {
    units::volt_t voltage = 0_V;

    switch (_state) {
    case ClimberState::kIdle:
        voltage = 0_V;
        break;
    case ClimberState::kAscending:
        voltage = _config.ascendingVoltage;
        break;
    case ClimberState::kClimbed:
        voltage = _config.climbedVoltage;
        break;
    case ClimberState::kDescending:
        voltage = _config.descendingVoltage;
        break;
    }

    _config.gearbox.transmission->SetVoltage(voltage);
}

void Climber::SetAscending() {
    _state = ClimberState::kAscending;
}

void Climber::SetDescending() {
    _state = ClimberState::kDescending;
}

ClimberState Climber::GetState() const {
    return _state;
}