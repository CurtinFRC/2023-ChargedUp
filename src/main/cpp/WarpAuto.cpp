#include "WarpAuto.h"

using namespace behaviour;

Auto::Auto(Drivebase drivebase, Armavator *armavator, Gripper *gripper): _drivebase(drivebase), _armavator(armavator), _gripper(gripper) {}

std::shared_ptr<Behaviour> Auto::Balance() {
  return make<DrivebasePoseBehaviour>(_drivebase.swerve, frc::Pose2d{4_m, 0_m,  0_deg}, 4_V)->Until(make<WaitFor>([](Drivebase _drivebase) {
      return fabs(_drivebase.gyro->GetAngle()) > 10;
    }))
  <<make<DrivebasePoseBehaviour>(_drivebase.swerve, frc::Pose2d{4_m, 0_m,  0_deg}, 2_V)->Until(make<WaitFor>([](Drivebase _drivebase) {
      return fabs(_drivebase.gyro->GetAngle()) > -10;
    }))
  << make<DrivebasePoseBehaviour>(_drivebase.swerve, frc::Pose2d{2_m, 0_m, 0_deg})->WithTimeout(3_s)
  << make<DrivebasePoseBehaviour>(_drivebase.swerve, frc::Pose2d{4_m, 0_m,  0_deg}, 4_V)->Until(make<WaitFor>([](Drivebase _drivebase) {
      return fabs(_drivebase.gyro->GetAngle()) > 10;
    }))
  <<make<DrivebaseBalance>(_drivebase.swerve, _drivebase.gyro);
}

std::shared_ptr<Behaviour> Auto::HighPlace() {
  return make<ArmavatorGoToAutoSetpoint>(_armavator, 0.5_m, 30_deg, 0.5, 0.3)->WithTimeout(3_s)
  << make<GripperAutoBehaviour>(_gripper, 1);
}
