#include "Climber.h"

using namespace frc;
using namespace wom;

Climber::Climber(ClimberConfig config) : _config(config) { }

void Climber::OnUpdate(units::second_t dt) {
    units::volt_t voltage = 0_V;

    switch (_state) {
    case ClimberState::kIdle:
        voltage = 0_V;
        break;
    case ClimberState::kWindUp:
        voltage = _config.windupVoltage;
        break;
    case ClimberState::kWindDown:
        voltage = _config.winddownVoltage;
        break;
    case ClimberState::kLocked:
        voltage = 1_V;
        break;
    }

    _config.gearbox.transmission->SetVoltage(voltage);
}

void Climber::SetWindUp() {
    _state = ClimberState::kWindUp;
}

void Climber::SetWindDown() {
    _state = ClimberState::kWindDown;
}
void Climber::SetLocked() {
    _state = ClimberState::kLocked;
}
void Climber::SetIdle() {
    _state = ClimberState::kIdle;
}

ClimberState Climber::GetState() const {
    return _state;
}