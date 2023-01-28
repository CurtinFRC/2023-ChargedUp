#pragma once

#include "Gearbox.h"
#include "behaviour/HasBehaviour.h"
#include <frc/DoubleSolenoid.h>
#include <ctre/Phoenix.h>


struct SideIntakeConfig {
  frc::DoubleSolenoid *claspSolenoid;
  frc::DoubleSolenoid *deploySolenoid;
  wom::MotorVoltageController *rightIntakeMotor;
  wom::MotorVoltageController *leftIntakeMotor;
};

enum class SideIntakeState {
  kIdle,
  kIntaking,
  kMovePiston,
  kClaspPiston,
  kOuttaking
};

class SideIntake : public behaviour::HasBehaviour {
 public:
  SideIntake(SideIntakeConfig config);

  void OnUpdate(units::second_t dt);

  void SetIdle();
  void SetIntaking();
  void SetMovePiston();
  void SetClaspPiston();

  void SetOuttaking();

  SideIntakeState GetState() const;

 private:
  SideIntakeConfig _config;
  SideIntakeState _state = SideIntakeState::kIdle;

  units::volt_t intakeVoltage = 10_V;
  units::volt_t outtakeVoltage = -7_V;
  units::volt_t holdVoltage = 2_V;
};