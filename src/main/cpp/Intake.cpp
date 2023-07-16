#include "Intake.h" 

Intake::Intake(IntakeConfig config) : _config(config) {}

void Intake::OnUpdate(units::second_t dt) {
    units::volt_t voltage;

    switch (_state)
    {
        case IntakeStates::kIdle:
            voltage = 0_V;
            break;
        case IntakeStates::kIntake:
            voltage = 8_V;
            break;
        case IntakeStates::kOuttake:
            voltage = -7_V;
            break;
    }

    _config.intakeMotor->SetVoltage(voltage);
    _config.intakeMotor2->SetVoltage(voltage);


}

void Intake::SetIntake() {
    _state = IntakeStates::kIntake;
}

void Intake::SetOuttake() {
    _state = IntakeStates::kOuttake;
}

void Intake::SetIdle() {
    _state = IntakeStates::kIdle;
}

std::string Intake::GetState() {
    switch (_state) {
        case IntakeStates::kIdle:
            return "Idle";
            break;
        case IntakeStates::kIntake:
            return "Intake";
            break;
        case IntakeStates::kOuttake:
            return "Outtake";
            break;
    }
}