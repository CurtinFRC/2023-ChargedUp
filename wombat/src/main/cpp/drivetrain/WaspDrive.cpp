// #include "drivetrain/WaspDrive.h"

// using namespace wom;

// WaspDrive::WaspDrive(std::string path, WaspDriveConfig config) 
//   : _config(config), _kinematics(config.trackWidth, 1_m),
//   _leftVelocityController(path + "/pid/left", config.velocityPID),
//   _rightVelocityController(path + "/pid/right", config.velocityPID), 
//   _dropVelocityController(path + "/pid/drop", config.velocityPID) {}

// void WaspDrive::OnUpdate(units::second_t dt) {
//   units::volt_t leftVoltage{0};
//   units::volt_t rightVoltage{0};
//   units::volt_t dropVoltage{0};

//   auto wheelSpeeds = _kinematics.ToWheelSpeeds(_speed);

//   switch (_state) {
//     case WaspDriveState::kIdle:
//       leftVoltage = 0_V;
//       rightVoltage = 0_V;
//       dropVoltage = 0_V;
//       break;
//     case WaspDriveState::kRaw:
//       leftVoltage = _leftRawSetpoint;
//       rightVoltage = _rightRawSetpoint;
//       dropVoltage = _dropRawSetpoint;
//       break;
//     case WaspDriveState::kVelocity:
//       {
//         _leftVelocityController.SetSetpoint(wheelSpeeds.left);
//         _rightVelocityController.SetSetpoint(wheelSpeeds.right);
//         _dropVelocityController.SetSetpoint(wheelSpeeds.rear);

//         auto feedforwardLeft = _config.leftDrive.motor.Voltage(0_Nm, units::radians_per_second_t{(_leftVelocityController.GetSetpoint() / _config.wheelRadius).value()});
//         auto feedforwardRight = _config.rightDrive.motor.Voltage(0_Nm, units::radians_per_second_t{(_rightVelocityController.GetSetpoint() / _config.wheelRadius).value()});
//         auto feedforwardDrop = _config.dropDrive.motor.Voltage(0_Nm, units::radians_per_second_t{(_dropVelocityController.GetSetpoint() / _config.wheelRadius).value()});

//         // leftVoltage = _leftVelocityController.Calculate(GetLeftSpeed(), dt, feedforwardLeft);
//         // rightVoltage = _rightVelocityController.Calculate(GetRightSpeed(), dt, feedforwardRight);
//         // dropVoltage = _dropVelocityController.Calculate(GetDropSpeed(), dt, feedforwardDrop);
      
//         leftVoltage = feedforwardLeft;
//         rightVoltage = feedforwardRight;
//         dropVoltage = feedforwardDrop;
//       }
//       break;
//   }
// }

// void WaspDrive::SetIdle() {
//   _state = WaspDriveState::kIdle;
// }

// void WaspDrive::SetRawVoltage(units::volt_t left, units::volt_t right, units::volt_t drop) {
//   _state = WaspDriveState::kRaw;
//   _leftRawSetpoint = left;
//   _rightRawSetpoint = right;
//   _dropRawSetpoint = drop;
// }

// void WaspDrive::SetVelocity(frc::ChassisSpeeds speeds) {
//   _state = WaspDriveState::kVelocity;
//   _speed = speeds;
// }

// // units::meter_t WaspDrive::GetLeftDistance() const 

// units::meters_per_second_t WaspDrive::GetLeftSpeed() const {
//   return units::meters_per_second_t{_config.leftDrive.encoder->GetEncoderAngularVelocity().value() * _config.wheelRadius.value()};
// }

// units::meters_per_second_t WaspDrive::GetRightSpeed() const {
//   return units::meters_per_second_t{_config.rightDrive.encoder->GetEncoderAngularVelocity().value() * _config.wheelRadius.value()};
// }

// units::meters_per_second_t WaspDrive::GetDropSpeed() const {
//   return units::meters_per_second_t{_config.dropDrive.encoder->GetEncoderAngularVelocity().value() * _config.wheelRadius.value()};
// }

