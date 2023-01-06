#pragma once

#include "gearbox.h"
#include "behaviour/HasBehaviour.h"
#include <frc/kinematics/MecanumDriveKinematics.h>
#include <ctre/Phoenix.h>


struct MecanumDrivebaseConfig {
  wom::Gearbox frontLeftGearBox, frontRightGearBox;
  wom::Gearbox rearLeftGearBox, rearRightGearBox;

  units::meter_t wheelRadius = 2_in;

  frc::Translation2d frontLeftPos{1_m, 1_m};
  frc::Translation2d frontRightPos{1_m, -1_m};
  frc::Translation2d rearLeftPos{-1_m, 1_m};
  frc::Translation2d rearRightPos{-1_m, -1_m};
};

enum class MecanumDrivebaseState{
  kIdle,
  kManual,
  kVelocity
};

class MecanumDrivebase : public behaviour::HasBehaviour{
 public:
  MecanumDrivebase(MecanumDrivebaseConfig config);

  void OnUpdate(units::second_t deltaTime);

  void SetManual(/* take in 4 voltages */);
  void SetVelocity(frc::ChassisSpeeds speeds);
  void SetIdle();
 
 private:
  MecanumDrivebaseConfig _config;
  MecanumDrivebaseState _activeState = MecanumDrivebaseState::kIdle;
  frc::ChassisSpeeds _targetSpeeds;

  frc::MecanumDriveKinematics _kinematics;

};