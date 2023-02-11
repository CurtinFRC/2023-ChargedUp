#pragma once

#include "RobotMap.h"
#include <string>

#include "behaviour/Behaviour.h"
#include "SideIntake.h"

class SideIntakeBehaviour : public behaviour::Behaviour {
 public:
  SideIntakeBehaviour(SideIntake *sideIntake, frc::XboxController &codriver);

  void OnStart() override;
  void OnTick(units::second_t dt) override;

 private:
  SideIntake *sideIntake;
  frc::XboxController &_codriver;

  bool _intakeReleased = false;
  bool _intakeGrapper = false;
  double _intakeSpeed;

  // nt::NetworkTableInstance _table = nt::NetworkTableInstance::GetDefault();
  // std::shared_ptr<nt::NetworkTable> _intakeTable = _defaultTable.GetTable("intake");

  // std::shared_ptr<nt::NetworkTable> _intakeTable = nt::NetworkTableInstance::GetDefault().GetTable("intake");
};