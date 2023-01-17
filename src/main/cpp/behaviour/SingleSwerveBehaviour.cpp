#include "behaviour/SingleSwerveBehaviour.h"
#include "ControlUtil.h"

using namespace wom;

SwerveSingleModManual::SwerveSingleModManual(SwerveModuleTest *swerveModuleTest, frc::XboxController *driverController):  _swerveTestModule(swerveModuleTest), _driverController(driverController){
    Controls(swerveModuleTest);
}

void SwerveSingleModManual::OnUpdate(units::second_t deltaTime){
  double driveSpeed = wom::deadzone(_driverController->GetLeftY(), driverDeadzone);
  double turnSpeed = wom::deadzone(_driverController->GetRightX(), turningDeadzone);
  

  _swerveTestModule->SetRaw(driveSpeed, turnSpeed);
}
