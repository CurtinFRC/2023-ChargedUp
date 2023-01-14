#include "behaviour/SwerveBaseBehaviour.h"
#include "ControlUtil.h"

using namespace wom;

ManualDrivebase::ManualDrivebase(wom::SwerveDrive *swerveDrivebase, frc::XboxController *driverController):  _swerveDrivebase(swerveDrivebase), _driverController(driverController){
    Controls(swerveDrivebase);
}

void ManualDrivebase::OnTick(units::second_t deltaTime){
    
    double l_x = wom::deadzone(_driverController->GetLeftX(), driverDeadzone);
    double l_y = wom::deadzone(-_driverController->GetLeftY(), driverDeadzone);
    double r_x = wom::deadzone(_driverController->GetRightX(), turningDeadzone);

    _swerveDrivebase->SetVelocity(frc::ChassisSpeeds {
        l_y * maxMovementMagnitude,
        l_x * maxMovementMagnitude,
        r_x * 180_deg / 0.25_s
    });
}