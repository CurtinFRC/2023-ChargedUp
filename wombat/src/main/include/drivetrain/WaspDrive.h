// #pragma once 

// #include "Gearbox.h"
// #include <frc/SpeedController.h>
// #include "behaviour/HasBehaviour.h"
// #include "behaviour/Behaviour.h"
// #include <frc/interfaces/Gyro.h>
// #include "WaspDriveKinematics.h"
// #include "PID.h"

// #include <string>

// namespace wom {

//   enum class WaspDriveState {
//     kIdle, 
//     kVelocity,
//     kRaw
//   };

//   struct WaspDriveConfig {
//     Gearbox &leftDrive;
//     Gearbox &rightDrive;
//     Gearbox &dropDrive;

//     frc::Gyro *gyro;

//     units::meter_t wheelRadius;
//     units::meter_t trackWidth;

//     units::ampere_t currentLimit;

//     PIDConfig<units::meters_per_second, units::volt> velocityPID;
//     // PIDConfig<units::meter, units::meters_per_second> distancePID;
//     // PIDConfig<units::degree, units::degree_per_second> anglePID;
//   };

//   class WaspDrive : public behaviour::HasBehaviour {
//    public: 
//     WaspDrive(std::string path, WaspDriveConfig config);

//     void OnUpdate(units::second_t dt);

//     void SetRawVoltage(units::volt_t left, units::volt_t right, units::volt_t drop);
//     void SetIdle();
//     void SetVelocity(frc::ChassisSpeeds speeds);

//     WaspDriveConfig &GetConfig() { return _config; }

//     units::meter_t GetLeftDistance() const;
//     units::meter_t GetRightDistance() const;

//     units::meters_per_second_t GetLeftSpeed() const;
//     units::meters_per_second_t GetRightSpeed() const;
//     units::meters_per_second_t GetDropSpeed() const;

//    private: 
//     WaspDriveConfig _config;
//     WaspDriveState _state;

//     units::volt_t _leftRawSetpoint;
//     units::volt_t _rightRawSetpoint;
//     units::volt_t _dropRawSetpoint;

//     units::volt_t _leftManualSetpoint;
//     units::volt_t _rightManualSetpoint;
//     units::volt_t _dropManualSetpoint;

//     frc::ChassisSpeeds _speed;
//     WaspDriveKinematics _kinematics;

//     PIDController<units::meters_per_second, units::volt> _leftVelocityController;
//     PIDController<units::meters_per_second, units::volt> _rightVelocityController;
//     PIDController<units::meters_per_second, units::volt> _dropVelocityController;
//   };
// }