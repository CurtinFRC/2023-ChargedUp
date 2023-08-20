#include "WarpAuto.h"

using namespace behaviour;

// not tested!! 
 std::shared_ptr<Behaviour> Balance(Drivebase _drivebase, Armavator *_armavator) {
   return 
    make<ArmavatorGoToAutoSetpoint>(_armavator, 0.9_m, -50_deg)
    << make<ArmavatorGoToAutoSetpoint>(_armavator, 0.8_m, 0_deg)
    << make<ArmavatorGoToAutoSetpoint>(_armavator, 0.2_m, 40_deg)
    << make<ArmavatorGoToAutoSetpoint>(_armavator, 0.1_m, 90_deg)
    << make<ArmavatorGoToAutoSetpoint>(_armavator, 0.1_m, 90_deg)
    << make<ArmavatorGoToAutoSetpoint>(_armavator, 0.1_m, 120_deg) //arm must start back in order to get up the charge station 
    << make<DrivebasePoseBehaviour>(_drivebase.swerve, frc::Pose2d{4_m, 0_m,  0_deg}, 4_V)->Until(make<WaitFor>([_drivebase]() {
       return fabs(_drivebase.gyro->GetPitch()) > 10 ||  fabs(_drivebase.gyro->GetRoll()) > 10;
     }))
    << make<DrivebasePoseBehaviour>(_drivebase.swerve, frc::Pose2d{2_m, 0_m, 0_deg}, 4_V)
    << make<DrivebasePoseBehaviour>(_drivebase.swerve, frc::Pose2d{-4_m, 0_m,  0_deg}, 4_V)->Until(make<WaitFor>([_drivebase]() {
       return fabs(_drivebase.gyro->GetPitch()) < 10 ||  fabs(_drivebase.gyro->GetRoll()) < 10;
     }))
     << make<ArmavatorGoToAutoSetpoint>(_armavator, 0.1_m, 70_deg) //arm must swing over to get up the charge station 
     << make<DrivebaseBalance>(_drivebase.swerve, _drivebase.gyro); //automatically balences the robot 
}



std::shared_ptr<Behaviour> HighPlace(Armavator *_armavator, Gripper *_gripper, Drivebase _drivebase) {
	return make<GripperAutoBehaviour>(_gripper, 2)->Until(make<WaitTime>(0.5_s))
		<<make<ArmavatorGoToAutoSetpoint>(_armavator, 0.7_m, -40_deg)->Until(make<WaitTime>(1_s))
		<<make<ArmavatorGoToAutoSetpoint>(_armavator, 0.1_m, 110_deg)->Until(make<WaitTime>(2_s))
		<<make<ArmavatorGoToAutoSetpoint>(_armavator, 0.7_m, 172_deg)->Until(make<WaitTime>(2_s))
		<<make<GripperAutoBehaviour>(_gripper, 1)->Until(make<WaitTime>(0.6_s))
		<<make<GripperAutoBehaviour>(_gripper, 3)->Until(make<WaitTime>(0.6_s))
		<<make<DrivebasePoseBehaviour>(_drivebase.swerve, frc::Pose2d{0.5_m, 0_m, 0_deg})->Until(make<WaitTime>(0.5_s)) // increase distance for taxi
		<<make<ArmavatorGoToAutoSetpoint>(_armavator, 0_m, 0_deg)->Until(make<WaitTime>(2_s))
		<<make<ArmavatorGoToAutoSetpoint>(_armavator, 0.7_m, 0_deg)->Until(make<WaitTime>(2_s))
		<<make<ArmavatorGoToAutoSetpoint>(_armavator, 0.7_m, -60_deg)->Until(make<WaitTime>(2_s));
}

std::shared_ptr<Behaviour> MidPlace(Armavator *_armavator, Gripper *_gripper, Drivebase _drivebase) {
	return make<GripperAutoBehaviour>(_gripper, 2)->Until(make<WaitTime>(0.5_s))
		<<make<DrivebasePoseBehaviour>(_drivebase.swerve, frc::Pose2d{0.5_m, 0_m, 0_deg})->Until(make<WaitTime>(1_s))
		<<make<ArmavatorGoToAutoSetpoint>(_armavator, 0.7_m, -40_deg)->Until(make<WaitTime>(1_s))
		<<make<ArmavatorGoToAutoSetpoint>(_armavator, 0_m, 90_deg)->Until(make<WaitTime>(1_s))
		<<make<ArmavatorGoToAutoSetpoint>(_armavator, 0.6_m, 180_deg, 0.5, 0.1)->Until(make<WaitTime>(1_s))
		<<make<GripperAutoBehaviour>(_gripper, 1)->Until(make<WaitTime>(0.6_s))
		<<make<GripperAutoBehaviour>(_gripper, 3)->Until(make<WaitTime>(0.6_s))
		<<make<DrivebasePoseBehaviour>(_drivebase.swerve, frc::Pose2d{0.5_m, 0_m, 0_deg})->Until(make<WaitTime>(0.5_s)) // increase distance for taxi
		<<make<ArmavatorGoToAutoSetpoint>(_armavator, 0_m, 0_deg)->Until(make<WaitTime>(2_s))
		<<make<ArmavatorGoToAutoSetpoint>(_armavator, 0.7_m, 0_deg)->Until(make<WaitTime>(2_s))
		<<make<ArmavatorGoToAutoSetpoint>(_armavator, 0.7_m, -60_deg)->Until(make<WaitTime>(2_s));
}

