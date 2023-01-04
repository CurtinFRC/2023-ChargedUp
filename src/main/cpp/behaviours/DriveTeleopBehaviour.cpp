#include "behaviours/DriveTeleopBehaviour.h"

#include "ControlUtil.h"

DriveTeleopBehaviour::DriveTeleopBehaviour(wom::Drivetrain *d, frc::XboxController *controller)
: _drivetrain(d), _controller(controller) {
  Controls(d);
}

void DriveTeleopBehaviour::OnTick(units::second_t dt) {
  double fwdPower = wom::spow2(wom::deadzone(-_controller->GetLeftY(), 0.05));
  double angPower = wom::spow2(wom::deadzone(_controller->GetLeftTriggerAxis() - _controller->GetRightTriggerAxis(), 0.05));
 
  units::meters_per_second_t fwdSpeed = fwdPower * 2_mps;
  units::radians_per_second_t angSpeed = angPower * 360_deg / 1_s;

  _drivetrain->SetVelocity(frc::ChassisSpeeds {
    fwdSpeed,
    0_mps, 
    angSpeed
  });

  
}