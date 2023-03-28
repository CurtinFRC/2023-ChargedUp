#pragma once

#include "Gearbox.h"
#include "behaviour/HasBehaviour.h"
#include <frc/DoubleSolenoid.h>
#include <ctre/Phoenix.h>
#include <string>


struct SideIntakeConfig {
  frc::DoubleSolenoid *claspSolenoid;
  frc::DoubleSolenoid *deploySolenoid;
  wom::Gearbox *rightGearbox;
  wom::Gearbox *leftGearbox;
};

enum class SideIntakeState {
  kIdle,
  kIntakingWide,
  kOuttakingWide,
  kIntakingClosed,
  kOutakingClosed,
  kWide,
  kClosed,
  kStowed
};

enum class IntakeActuationState {
  kStowed,
  kDeployed
};

enum class IntakeCloseState {
  kClosed,
  kOpen
};

class SideIntake : public behaviour::HasBehaviour {
 public:
  SideIntake(SideIntakeConfig config);

  void OnUpdate(units::second_t dt);

  void SetIdle();
  void SetIntakingWide();
  void SetIntakingClosed();
  void SetOutakingWide();
  void SetOutakingClosed();
  void SetStowed();
  void SetClosed();
  void SetWide();

  void SetClose();
  void SetStow();
  void SetDeploy();
  void SetOpen();

  void SetVoltage(units::volt_t voltage);

  std::string GetState() const;

 private:
  SideIntakeConfig _config;
  SideIntakeState _state = SideIntakeState::kIdle;
  IntakeActuationState _actuationState;
  IntakeCloseState _closedState = IntakeCloseState::kOpen;

  units::volt_t intakeVoltage = 10_V;
  units::volt_t outtakeVoltage = -7_V;
  units::volt_t holdVoltage = 2_V;

  units::volt_t _voltage;
};