#include "Armavator.h"
#include <units/math.h>
#include "drivetrain/SwerveDrive.h"
#include "Robot.cpp"

using namespace frc;

//Armavator configeration
Armavator::Armavator(ArmavatorConfig config, wom::SwerveDrive swervedrive)
: config(config), arm(config.arm), elevator(config.elevator), swervedrive(swervedrive) {}

//Instructions for when the program updates (seconds delta time)
void Armavator::OnUpdate(units::second_t dt) {
  units::volt_t voltage{0};

  switch(_state) {
    case ArmavatorState::kIdle:
      break;
    case ArmavatorState::kPosition:
      //frc::Pose2d SwerveDrive::GetPose()
      frc::Pose2d pose = swervedrive.GetPose();
      pose.X();
      pose.Y();
      pose.Rotation();

      if ((pose.X() <= 2.51_m && pose.Y() >= 8.53_m) || (pose.X() >= 2.52_m && pose.Y() >= 10.33_m)){
        ArmavatorPosition{0_m, 90_deg};

      } else{
        arm.SetAngle(_setpoint.angle);
        elevator.SetPID(_setpoint.height);
      }
      break;

  arm.OnUpdate(dt);
  elevator.OnUpdate(dt);
}
}

//Sets the states names
void Armavator::SetIdle() {
  _state = ArmavatorState::kIdle;
}

void Armavator::SetPosition(ArmavatorPosition pos) {
  _state = ArmavatorState::kPosition;
  _setpoint = pos;
}

ArmavatorPosition Armavator::GetCurrentPosition() const {
  return ArmavatorPosition {
    elevator.GetHeight(),
    arm.GetAngle()
  };
}

bool Armavator::IsStable() const {
  return elevator.IsStable() && arm.IsStable();
}

/* SIMULATION */

// ::sim::ArmavatorSim::ArmavatorSim(ArmavatorConfig config)
//   : config(config), armSim(config.arm), elevatorSim(config.elevator) {}

// void ::sim::ArmavatorSim::OnUpdate(units::second_t dt) {
//   armSim.Update(dt);
//   elevatorSim.Update(dt);
// }

// units::ampere_t sim::ArmavatorSim::GetCurrent() const {
//   return armSim.GetCurrent() + elevatorSim.GetCurrent();
// }