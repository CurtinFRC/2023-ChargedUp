#include "WarpAuto.h"

using namespace behaviour;

// test
std::shared_ptr<Behaviour> Taxi(Drivebase _drivebase, Armavator *_armavator) {
    return make<DrivebasePoseBehaviour>(_drivebase.swerve, frc::Pose2d{0.5_m, 0_m, 0_deg})->Until(make<WaitTime>(5_s)) // increase distance for taxi
            <<make<ArmavatorGoToAutoSetpoint>(_armavator, 0_m, 0_deg)->Until(make<WaitTime>(5_s));
}

std::shared_ptr<Behaviour> highPlace(Armavator *_armavator, Gripper *_gripper, Drivebase _drivebase) {
    return make<GripperAutoBehaviour>(_gripper, 4)->Until(make<WaitTime>(0.5_s))
            <<make<ArmavatorGoToAutoSetpoint>(_armavator, 0.7_m, -40_deg)->Until(make<WaitTime>(1_s))
            <<make<ArmavatorGoToAutoSetpoint>(_armavator, 0.1_m, 110_deg)->Until(make<WaitTime>(2_s))
            <<make<ArmavatorGoToAutoSetpoint>(_armavator, 0.7_m, 172_deg)->Until(make<WaitTime>(2_s))
            <<make<GripperAutoBehaviour>(_gripper, 1)->Until(make<WaitTime>(0.6_s))
            <<make<GripperAutoBehaviour>(_gripper, 3)->Until(make<WaitTime>(0.6_s))
            <<make<DrivebasePoseBehaviour>(_drivebase.swerve, frc::Pose2d{4.75_m, 0_m, 0_deg})->Until(make<WaitTime>(4.3_s))
            <<make<ArmavatorGoToAutoSetpoint>(_armavator, 0_m, 0_deg)->Until(make<WaitTime>(1_s))
            <<make<ArmavatorGoToAutoSetpoint>(_armavator, 0.7_m, 0_deg)->Until(make<WaitTime>(1_s))
            <<make<ArmavatorGoToAutoSetpoint>(_armavator, 0.7_m, -60_deg)->Until(make<WaitTime>(1_s));
}

std::shared_ptr<Behaviour> MidPlace(Armavator *_armavator, Gripper *_gripper, Drivebase _drivebase) {
    return make<GripperAutoBehaviour>(_gripper, 2)->Until(make<WaitTime>(0.5_s))
            <<make<DrivebasePoseBehaviour>(_drivebase.swerve, frc::Pose2d{0.5_m, 0_m, 0_deg})->Until(make<WaitTime>(1_s))
            <<make<ArmavatorGoToAutoSetpoint>(_armavator, 0.7_m, -40_deg)->Until(make<WaitTime>(1_s))
            <<make<ArmavatorGoToAutoSetpoint>(_armavator, 0_m, 90_deg)->Until(make<WaitTime>(1.5_s))
            <<make<ArmavatorGoToAutoSetpoint>(_armavator, 0.6_m, 180_deg, 0.5, 0.1)->Until(make<WaitTime>(1_s))
            <<make<GripperAutoBehaviour>(_gripper, 1)->Until(make<WaitTime>(0.6_s))
            <<make<GripperAutoBehaviour>(_gripper, 3)->Until(make<WaitTime>(0.6_s))
            <<make<DrivebasePoseBehaviour>(_drivebase.swerve, frc::Pose2d{0.5_m, 0_m, 0_deg})->Until(make<WaitTime>(0.5_s)) // increase distance for taxi
            <<make<ArmavatorGoToAutoSetpoint>(_armavator, 0_m, 0_deg)->Until(make<WaitTime>(2_s))
            <<make<ArmavatorGoToAutoSetpoint>(_armavator, 0.7_m, 0_deg)->Until(make<WaitTime>(2_s))
            <<make<ArmavatorGoToAutoSetpoint>(_armavator, 0.7_m, -60_deg)->Until(make<WaitTime>(2_s));
}


std::shared_ptr<Behaviour> SmallDrivebaseMovement(Drivebase _drivebase) {
    return make<DrivebasePoseBehaviour>(_drivebase.swerve, frc::Pose2d{0.5_m, 0_m, 0_deg})->Until(make<WaitTime>(5_s));
}

std::shared_ptr<Behaviour> SmallArmMovement(Armavator *_armavator) {
    return make<ArmavatorGoToAutoSetpoint>(_armavator, 0.7_m, -40_deg)->Until(make<WaitTime>(1_s));

}
