#pragma once

#include "drivetrain/SwerveDrive.h"
#include "behaviour/Behaviour.h"
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>
#include <frc/PS4Controller.h>
#include <networktables/NetworkTableInstance.h>
#include "PID.h"

#include <vector>

/**
 * @brief Behaviour class to handle manual drivebase controlling with the controller
 */ 
class ManualDrivebase : public behaviour::Behaviour{
 public:
   /**
   * @param swerveDrivebase
   * A pointer to the swerve drivebase
   * @param driverController
   * A pointer to the controller that the driver has been allocated
  */
  ManualDrivebase(wom::SwerveDrive *swerveDrivebase, wom::Controller *driverController);

  void OnTick(units::second_t deltaTime) override;
  void OnStart();
  
 private:
  wom::SwerveDrive *_swerveDrivebase;
  wom::Controller *_driverController;

  const double driverDeadzone = 0.08;
  const double turningDeadzone = 0.1;
  const units::meters_per_second_t maxMovementMagnitude = 6.5_ft / 1_s;

  std::shared_ptr<nt::NetworkTable> _swerveDriveTable = nt::NetworkTableInstance::GetDefault().GetTable("swerve");

};




/**
 * @brief Behaviour Class to hangle the swerve drivebase going to and potentially maintaining the position
 */
class DrivebasePoseBehaviour : public behaviour::Behaviour{
 public:
   /**
   * @param swerveDrivebase
   * A pointer to the swerve drivebase
   * @param pose
   * A variable containing an X coordinate, a Y coordinate, and a rotation, for the drivebase to go to
   * @param hold
   * An optional variable (defaulting false), to say whether this position should be maintained
  */
  DrivebasePoseBehaviour(wom::SwerveDrive *swerveDrivebase, frc::Pose2d pose, bool hold = false);
  
  /**
   * @brief 
   * 
   * @param deltaTime change in time since the last iteration
   */
  void OnTick(units::second_t deltaTime) override;
 
 private:
  wom::SwerveDrive *_swerveDrivebase;
  frc::Pose2d _pose;
  bool _hold;
  
  std::shared_ptr<nt::NetworkTable> _swerveDriveTable = nt::NetworkTableInstance::GetDefault().GetTable("swerve");
};




/**
 * @brief Behaviour Class to handle the swerve drivebase balancing on the chargestation
 */
class DrivebaseBalance : public behaviour::Behaviour{
 public:
   /**
   * @param swerveDrivebase
   * A pointer to the swerve drivebase
   * @param gyro
   * A pointer to the type of gyro being used on the swerve drivebase
  */
  DrivebaseBalance(wom::SwerveDrive *swerveDrivebase, wom::NavX *gyro);

  void OnTick(units::second_t deltaTime) override;


 private:
  wom::SwerveDrive *_swerveDrivebase;
  wom::NavX *_gyro;

  wom::SwerveDriveConfig::balance_conf_t balancePIDConfig{
    "swerve/balancePID/",
    0.7_mps / 10_deg,
    wom::SwerveDriveConfig::balance_conf_t::ki_t{0.00},
    wom::SwerveDriveConfig::balance_conf_t::kd_t{0}
  };
  wom::PIDController<units::degree, units::meters_per_second> lateralBalancePID{
    "swerve/balancePID",
    balancePIDConfig,
    0_deg
  };
  wom::PIDController<units::degree, units::meters_per_second> sidwaysBalancePID{
    "swerve/balancePID",
    balancePIDConfig,
    0_deg
  };
  std::shared_ptr<nt::NetworkTable> _swerveDriveTable = nt::NetworkTableInstance::GetDefault().GetTable("swerve");
};




/**
 * @brief Behaviour Class to handle locking wheels
 */
class XDrivebase : public behaviour::Behaviour{
 public:
   /**
   * @param swerveDrivebase
   * A pointer to the swerve drivebase
  */
  XDrivebase(wom::SwerveDrive *swerveDrivebase);

  void OnTick(units::second_t deltaTime) override;

 private:
  wom::SwerveDrive *_swerveDrivebase;
};



/**
 * @brief Behaviour Class to handle the swerve drivebase driving to the nearest team grid position if it is within range
 */
class AlignDrivebaseToNearestGrid : public behaviour::Behaviour{
  struct SwerveGridPoses{
    frc::Pose2d innerGrid1;
    frc::Pose2d innerGrid2;
    frc::Pose2d innerGrid3;
    frc::Pose2d centreGrid1;
    frc::Pose2d centreGrid2;
    frc::Pose2d centreGrid3;
    frc::Pose2d outerGrid1;
    frc::Pose2d outerGrid2;
    frc::Pose2d outerGrid3;
  };
 public:
  /**
   * @param wom::SwerveDrive
   * A pointer to the swerve drivebase
   * @param std::vector<frc::Pose2d*>
   * A vector of frc::Pose2d's storing all 9 current alliance grid positions
  */
  AlignDrivebaseToNearestGrid(wom::SwerveDrive *swerveDrivebase, std::vector<frc::Pose2d*> gridPoses);

  void OnTick(units::second_t deltaTime) override;
  void OnStart() override;

 private:
  wom::SwerveDrive *_swerveDrivebase;
  std::vector<frc::Pose2d*> _gridPoses;
};