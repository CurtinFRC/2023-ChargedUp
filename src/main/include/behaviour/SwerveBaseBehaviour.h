#pragma once

#include "drivetrain/SwerveDrive.h"
#include "behaviour/Behaviour.h"
#include <ctre/Phoenix.h>
#include "XInputController.h"
#include <networktables/NetworkTableInstance.h>
#include "PID.h"
#include <vector>

#include "Vision.h"
#include "Auto.h"

/**
 * @brief Behaviour class to handle manual drivebase controlling with the controller
 */ 
class ManualDrivebase : public behaviour::Behaviour{
 public:
   /**
   * @param swerveDrivebase
   * A pointer to the swerve drivebase (the allocated memory address that stores the "swerve drivebase" object)
   * @param driverController
   * A pointer to the controller that the driver has been allocated (the allocated memory address that stores the "driver controller" object)
  */
  ManualDrivebase(wom::SwerveDrive *swerveDrivebase, frc::XboxController *driverController);

  void OnTick(units::second_t deltaTime) override;
  /**
   * @brief This function handles all of the logic behind the tangent function, to be able to calculate an angle between 0 andd 360 degrees, inclusively
  */
  void CalculateRequestedAngle(double joystickX, double joystickY, units::degree_t defaultAngle);
  void OnStart(units::second_t dt);
  
 private:
  std::shared_ptr<nt::NetworkTable> _swerveDriveTable = nt::NetworkTableInstance::GetDefault().GetTable("swerve");
  wom::SwerveDrive *_swerveDrivebase;
  frc::XboxController *_driverController;
  
  // State-handler Boolean : Is the robot in field orientated control, or robot relative?
  bool isFieldOrientated = true;
  // State-handler Boolean : Do we currently want the angles of the wheels to be 0?
  bool isZero = false;
  

  units::degree_t _requestedAngle;


  // Deadzones
  const double driverDeadzone = 0.08;
  const double turningDeadzone = 0.2;


  // Variables for solution to Anti-tip
  double prevJoystickX, prevJoystickY, prevPrevJoystickX, prevPrevJoystickY, usingJoystickXPos, usingJoystickYPos;
  // The speed that the joystick must travel to activate averaging over previous 3 joystick positions
  const double smoothingThreshold = 1;

  typedef units::meters_per_second_t translationSpeed_;
  typedef units::radians_per_second_t rotationSpeed_;

  // The translation speeds for when "slow speed", "normal speed", "fast speed" modes are active
  const translationSpeed_ lowSensitivityDriveSpeed = 3.25_ft / 1_s;
  const translationSpeed_ defaultDriveSpeed = 13_ft / 1_s;
  const translationSpeed_ highSensitivityDriveSpeed = 18_ft / 1_s;
  // The rotation speeds for when "slow speed", "normal speed", "fast speed" modes are active
  const rotationSpeed_ lowSensitivityRotateSpeed = 90_deg / 1_s;
  const rotationSpeed_ defaultRotateSpeed = 360_deg / 1_s;
  const rotationSpeed_ highSensitivityRotateSpeed = 720_deg / 1_s;

  translationSpeed_ maxMovementMagnitude = defaultDriveSpeed;
  rotationSpeed_ maxRotationMagnitude = defaultRotateSpeed;
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
  DrivebasePoseBehaviour(wom::SwerveDrive *swerveDrivebase, frc::Pose2d pose, units::volt_t voltageLimit =10_V ,bool hold = false);
  
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
  units::volt_t _voltageLimit;
  
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
    70_mps / 1000_deg,
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
  public:
  /**
   * @param wom::SwerveDrive
   * A pointer to the swerve drivebase
   * @param std::vector<frc::Pose2d*>
   * A vector of frc::Pose2d's storing all 9 current alliance grid positions
  */
  AlignDrivebaseToNearestGrid(wom::SwerveDrive *swerveDrivebase);
  AlignDrivebaseToNearestGrid(wom::SwerveDrive *swerveDrivebase, Vision *vision, int alignType);
  

  void OnTick(units::second_t deltaTime) override;
  void OnStart() override;

 private:

  units::meter_t alignmentAllowDistance = 5_m;
  int _alignType = 0;
  wom::SwerveDrive *_swerveDrivebase;
  Vision *_vision = nullptr;
  std::vector<frc::Pose2d> _gridPoses = {
      frc::Pose2d{72.061_in, 20.208_in, 0_deg},
      frc::Pose2d{72.061_in, 42.2_in, 0_deg},
      frc::Pose2d{72.061_in, 64.185_in, 0_deg},
      frc::Pose2d{72.061_in, 86.078_in, 0_deg},
      frc::Pose2d{72.061_in, 108.131_in, 0_deg},
      frc::Pose2d{72.061_in, 130.185_in, 0_deg},
      frc::Pose2d{72.061_in, 152.185_in, 0_deg},
      frc::Pose2d{72.061_in, 174.170_in, 0_deg},
      frc::Pose2d{72.061_in, 196.185_in, 0_deg},
      frc::Pose2d{579.351_in, 20.185_in, 0_deg},
      frc::Pose2d{579.351_in, 42.185_in, 0_deg},
      frc::Pose2d{579.351_in, 64.185_in, 0_deg},
      frc::Pose2d{579.351_in, 86.185_in, 0_deg},
      frc::Pose2d{579.351_in, 108.185_in, 0_deg},
      frc::Pose2d{579.351_in, 130.293_in, 0_deg},
      frc::Pose2d{579.351_in, 152.185_in, 0_deg},
      frc::Pose2d{579.351_in, 174.185_in, 0_deg},
      frc::Pose2d{579.351_in, 196.185_in, 0_deg}
  };
};
