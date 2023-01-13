#include "behaviours/DrivebaseBehaviour.h"

using namespace wom;

ManualDrivebase::ManualDrivebase(MecanumDrivebase *mecanumDrivebase, frc::XboxController *driverController):  _mecanumDrivebase(mecanumDrivebase), _driverController(driverController){
    Controls(mecanumDrivebase);
}

void ManualDrivebase::OnTick(units::second_t deltaTime){

    double l_x = _driverController->GetLeftX();
    double l_y = _driverController->GetRightX();
    double r_x = _driverController->GetLeftY();

    // deals with controller deadzones
    if (-driverDeadzone <= l_x && l_x <= driverDeadzone) {l_x = 0;}
    if (-driverDeadzone <= l_y && l_y <= driverDeadzone) {l_y = 0;}
    if (-turningDeadzone <= r_x && r_x <= turningDeadzone) {r_x = 0;}

    // DriveCartesian(l_x, l_y, r_x);

    _mecanumDrivebase->SetVelocity(frc::ChassisSpeeds {
        l_x * maxMovementMagnitude,
        l_y * maxMovementMagnitude,
        r_x * 180_deg / 1_s
    });
}