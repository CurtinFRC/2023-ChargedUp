#include "WarpAuto.hpp"

using namespace behaviour;

Auto::Auto() {}

std::shared_ptr<Behaviour> Balance(Drivebase *drivebase) {
  return make<DrivebasePoseBehaviour>(drivebase->swerve, frc::Pose2d{4_m, 0_m,  0_deg}, 4_V)->Until(make<WaitFor>([drivebase]() {
      return units::math::abs(drivebase->gyro->GetPitch()) > 10_deg ||  units::math::abs(drivebase->gyro->GetRoll()) > 10_deg;
    }))
  <<make<DrivebasePoseBehaviour>(drivebase->swerve, frc::Pose2d{4_m, 0_m,  0_deg}, 2_V)->Until(make<WaitFor>([drivebase]() {
      return units::math::abs(drivebase->gyro->GetPitch()) > -10_deg ||  units::math::abs(drivebase->gyro->GetRoll()) > -10_deg;
    }))
  << make<DrivebasePoseBehaviour>(drivebase->swerve, frc::Pose2d{2_m, 0_m, 0_deg})->WithTimeout(3_s)
  << make<DrivebasePoseBehaviour>(drivebase->swerve, frc::Pose2d{4_m, 0_m,  0_deg}, 4_V)->Until(make<WaitFor>([drivebase]() {
      return units::math::abs(drivebase->gyro->GetPitch()) > 10_deg ||  units::math::abs(drivebase->gyro->GetRoll()) > 10_deg;
    }))
  <<make<DrivebaseBalance>(drivebase->swerve, drivebase->gyro);
}
