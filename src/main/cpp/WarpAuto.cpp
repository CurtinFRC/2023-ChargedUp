#include "WarpAuto.h"

using namespace behaviour;

//std::shared_ptr<Behaviour> Balance(Drivebase *_drivebase, Armavator *_armavator) {
  //return make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{4_m, 0_m,  0_deg}, 4_V)->Until(make<WaitFor>([_drivebase]() {
 // return make<DrivebasePoseBehaviour>(_drivebase->swerve, frc::Pose2d{4_m, 0_m,  0_deg}, 4_V)->Until(make<WaitFor>([](Drivebase _drivebase) {
    //  return fabs(_drivebase.gyro->GetAngle()) > 10;
    //}))
  //<<make<DrivebasePoseBehaviour>(_drivebase->swerve, frc::Pose2d{4_m, 0_m,  0_deg}, 2_V)->Until(make<WaitFor>([](Drivebase _drivebase) {
  //    return fabs(_drivebase.gyro->GetAngle()) > -10;
  //  }))
  //<< make<DrivebasePoseBehaviour>(_drivebase->swerve, frc::Pose2d{2_m, 0_m, 0_deg})->WithTimeout(3_s)
  //<< make<DrivebasePoseBehaviour>(_drivebase->swerve, frc::Pose2d{4_m, 0_m,  0_deg}, 4_V)->Until(make<WaitFor>([]() {
  //    return fabs(_drivebase.gyro->GetAngle()) > 10;
  //  }))
  //<<make<DrivebaseBalance>(_drivebase->swerve, _drivebase->gyro);
//}

// std::shared_ptr<Behaviour> HighPlace() {
//   return make<ArmavatorGoToAutoSetpoint>(_armavator, 0.5_m, 30_deg, 0.5, 0.3)->WithTimeout(3_s)
//   << make<GripperAutoBehaviour>(_gripper, 1);
// }

std::shared_ptr<Behaviour> Balance(Drivebase _drivebase, Armavator *armavator) {
  return 
    make<ArmavatorGoToAutoSetpoint>(armavator, 0.9_m, -50_deg)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.8_m, 0_deg)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.2_m, 40_deg)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.1_m, 90_deg)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.1_m, 90_deg)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.1_m, 120_deg) //arm must start back in order to get up the charge station 
    << make<WaitTime>(3_s)
    //moves forwards until a tilt of more than 10 degrees is detected, then moves onto the next behaviour 
    << make<DrivebasePoseBehaviour>(_drivebase.swerve, frc::Pose2d{4_m, 0_m,  0_deg}, 4_V)->Until(make<WaitFor>([_drivebase]() {
      return units::math::abs(_drivebase.gyro->GetPitch()) > 10_deg ||  units::math::abs(_drivebase.gyro->GetRoll()) > 10_deg;
    }))
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.1_m, 70_deg) //arm must swing over to get up the charge station 
    << make<DrivebasePoseBehaviour>(_drivebase.swerve, frc::Pose2d{2_m, 0_m, 0_deg}, 4_V)->Until(make<WaitFor>([_drivebase]() {
      return units::math::abs(_drivebase.gyro->GetPitch()) == 0_deg || units::math::abs(_drivebase.gyro->GetRoll()) == 0_deg;
    }))
    << make<DrivebasePoseBehaviour>(_drivebase.swerve, frc::Pose2d{-4_m, 0_m,  0_deg}, 4_V)->Until(make<WaitFor>([_drivebase]() {
      return units::math::abs(_drivebase.gyro->GetPitch()) > 10_deg ||  units::math::abs(_drivebase.gyro->GetRoll()) > 10_deg;
    }))
    << make<DrivebaseBalance>(_drivebase.swerve, _drivebase.gyro); //automatically balances the robot 
}
