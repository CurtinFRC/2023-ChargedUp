#include "behaviour/ArmavatorBehaviour.h"
#include <frc/smartdashboard/SmartDashboard.h>


ArmavatorGoToAutoSetpoint::ArmavatorGoToAutoSetpoint(Armavator *armavator, units::meter_t height, units::degree_t angle, double elevatorSpeed, double armSpeed) 
  : _armavator(armavator), _height(height), _angle(angle), _elevatorSpeed(elevatorSpeed), _armSpeed(armSpeed) {
    Controls(armavator);
}

void ArmavatorGoToAutoSetpoint::OnStart() {
  std::cout << "intake + armavator auto setpoint routines start" << std::endl;
}

void ArmavatorGoToAutoSetpoint::OnTick(units::second_t dt) {
  ArmavatorPosition pos = {_height, _angle};
  _armavator->SetSpeedValues(_elevatorSpeed, _armSpeed);

  _armavator->SetPosition(pos);
  if (_armavator->IsStable()) {
    SetDone();
  }
}

//Constructs class
ArmavatorGoToPositionBehaviour::ArmavatorGoToPositionBehaviour(Armavator *armavator, ArmavatorPosition setpoint)
: _armavator(armavator), _setpoint(setpoint) {
  //tells code that the points are controlled (one point at a time) 
  Controls(armavator);
};

// Function for OnStart
void ArmavatorGoToPositionBehaviour::OnStart() {
  std::cout << "On Start" << std::endl;
}

//Function for OnTick
void ArmavatorGoToPositionBehaviour::OnTick(units::second_t dt) {

  if(_setpoint.height < 1_m) {
    if (_setpoint.angle >  0_rad){ // _setpoint.angle.value() < 0
      _setpoint.angle = 0_rad;
    }
    if (_setpoint.angle > 90_rad){
      _setpoint.angle = 90_rad;
    }
  };
  _armavator->SetPosition(_setpoint);

  if (_armavator->IsStable())
    SetDone();
}

ArmavatorRawBehaviour::ArmavatorRawBehaviour(Armavator *armavator, frc::XboxController &codriver)
: _armavator(armavator), _codriver(codriver) {
  //tells code that the points are controlled (one point at a time) 
  Controls(armavator);
};

void ArmavatorRawBehaviour::OnStart() {
}

void ArmavatorRawBehaviour::OnTick(units::second_t dt) {
  //Raw Positioning
  _armavator->SetManual(
    -_codriver.GetLeftY() * 9_V,
    -_codriver.GetRightY() * 9_V
  );
}


ArmavatorManualBehaviour::ArmavatorManualBehaviour(Armavator *armavator, frc::XboxController &codriver) 
  : _armavator(armavator), _codriver(codriver) {
    Controls(armavator);
}

void ArmavatorManualBehaviour::OnStart() {
  // startHeight = _armavator->GetCurrentPosition().height;
  // _manualSetpoint = _armavator->GetCurrentPosition();
  // _config.elevator.leftGearbox.encoder->ZeroEncoder();
  // _config.elevator.rightGearbox.encoder->ZeroEncoder();

  // _config.arm.leftGearbox.encoder->SetEncoderPosition(90_deg);
  _armavator->OnStart();
  // _config.arm.rightGearbox.encoder->SetEncoderPosition(90_deg);
  // startHeight = 0_m;
  // std::cout << "AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH"<< std::endl;
  // _manualSetpoint = {_armavator->elevator->GetConfig().leftGearbox.encoder->GetEncoderPosition() * _armavator->elevator->GetConfig().radius, _armavator->arm->GetConfig().leftGearbox.encoder->GetEncoderPosition()};
}

void ArmavatorManualBehaviour::OnTick(units::second_t dt) {
  if (_armavator->GetCurrentPosition().height > 5_m) {
    _armavator->OnStart();
  }

  frc::SmartDashboard::PutNumber("manual setpoint height", _manualSetpoint.height.value());
  frc::SmartDashboard::PutNumber("manual setpoint angle", _manualSetpoint.angle.value() * (180 / 3.14159)); 

  if (_codriver.GetAButtonPressed()) {
    if (rawControl) {
      // _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
      rawControl = false;
    } else {
      rawControl = true;
    }
  }

  if (rawControl) {
    double armPower = -wom::deadzone(_codriver.GetLeftY());
    double elePower = -wom::deadzone(_codriver.GetRightY());
    _armavator->SetManual(armPower * 11_V, elePower * 8_V);
    _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
    _armavator->SetSpeedValues(0.5, 0.3);
    frc::SmartDashboard::PutBoolean("PID mode", false); 

  } else {
    
    frc::SmartDashboard::PutBoolean("PID mode", true); 
    /**SETPOINTS
     * hold -> 
     * place front mid -> 
     * place back mid -> 
     * place back high -> 0.2615_m, 157_deg
     * grab cone -> 
     * grab cone lying down -> 0.896, -3.4608
     * grab cone on floor -> 
     * 
    */



    if (_codriver.GetPOV() == 0) {
      //picking up cone down 
      // _setpointValue.height = 0.1_m;
      // _setpointValue.angle = 30_deg;

      _setpointValue.height = 0.86_m;
      _setpointValue.angle = -7.8_deg;

      _armavator->SetPosition(_setpointValue);
      _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
      std::cout << "GO TO armavator POS 1 " << std::endl;
      _armavator->SetSpeedValues(0.5, 0.2);

    } else if (_codriver.GetPOV() == 90) {
      //picking up cone up 
      // _setpointValue.height = 0.5_m;
      // _setpointValue.angle = 0_deg;

      _setpointValue.height = 0.01_m;
      _setpointValue.angle = 34.1_deg;

      _armavator->SetPosition(_setpointValue);
      _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
      std::cout << "GO TO armavator POS 2 " << std::endl;
      _armavator->SetSpeedValues(0.5, 0.2);

    } else if (_codriver.GetPOV() == 180) {
      //picking up cone down to collect 
      // _setpointValue.height = 0.5_m;
      // _setpointValue.angle = 180_deg;

      _setpointValue.height = 0.86_m;
      _setpointValue.angle = -6_deg;

      _armavator->SetPosition(_setpointValue);
      _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
      std::cout << "GO TO armavator POS 3 " << std::endl;
      _armavator->SetSpeedValues(0.3, 0.1);

    } else if (_codriver.GetPOV() == 270) {
      //holding
      //TODO potench change max speed here
      _setpointValue.height = 0.1_m;
      _setpointValue.angle = 60_deg;
      // 0.896, -3.4608
      _armavator->SetPosition(_setpointValue);
      _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
      std::cout << "GO TO armavator POS 4 " << std::endl;
      _armavator->SetSpeedValues(0.35, 0.07);
    } else if (_codriver.GetXButton()) {
      //front mid place 
      _setpointValue.height = 0.612_m;
      _setpointValue.angle = 6.17_deg;
      _armavator->SetPosition(_setpointValue);
      _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
      std::cout << "GO TO armavator POS 5 " << std::endl;
      _armavator->SetSpeedValues(0.35, 0.07);
    } else if (_codriver.GetYButton()) {
      // 152_deg 0.1814_m back high place 
      _setpointValue.height = 0.1918_m;
      _setpointValue.angle = 159.43_deg;
      _armavator->SetPosition(_setpointValue);
      _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
      std::cout << "GO TO armavator POS 6 " << std::endl;
      _armavator->SetSpeedValues(0.35, 0.07);

    } else if (_codriver.GetBButton()) {
      // ground collect 
      _setpointValue.height = 0.128_m;
      _setpointValue.angle = 16.7_deg;
      _armavator->SetPosition(_setpointValue);
      _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
      std::cout << "GO TO armavator POS 7 " << std::endl;
      _armavator->SetSpeedValues(0.35, 0.07);
    } else {
      units::meter_t height = _armavator->GetCurrentPosition().height;
      units::degree_t angle = _armavator->GetCurrentPosition().angle;
      _armavator->SetSpeedValues(0.5, 0.3);

  
      
      if (_manualSetpoint.height > 0.95_m) {
        _manualSetpoint.height = 0.95_m;
      } else if (_manualSetpoint.height < 0.01_m) {
        _manualSetpoint.height = 0.01_m;
      } else {
        //if not outside deadzone set to current value
        _manualSetpoint.height -= (wom::deadzone(_codriver.GetRightY(), 0.15) * 1_m * 0.05);
      }

      if (_manualSetpoint.angle > 265_deg) {
        _manualSetpoint.angle = 265_deg;
      } else if (_manualSetpoint.angle < -60_deg) {
        _manualSetpoint.angle = -60_deg;
      } else {
         
        _manualSetpoint.angle -= (wom::deadzone(_codriver.GetLeftY(), 0.15) * 1_deg * 3);
        // _manualSetpoint.angle = angle - (wom::spow2(wom::deadzone(_codriver.GetLeftY(), 0.3)) * 1_deg * 30);
      }
      // units::degree_t trimmed_angle;
      // units::degree_t max_angle = units::math::asin((height - (1.9_m - 0.51_m)) / (-_armavator->arm->GetConfig().armLength));
      units::meter_t max_height = (1.9_m - 0.51_m) - _armavator->arm->GetConfig().armLength * units::math::sin(_manualSetpoint.angle);
      // if (_manualSetpoint.angle < 90_deg) {
      //   trimmed_angle = units::math::min(max_angle, _manualSetpoint.angle);
      // } else if (_manualSetpoint.angle >= 90_deg) {
      //   trimmed_angle = units::math::max(3.1415_rad - max_angle,  _manualSetpoint.angle);
      // }
      
      ArmavatorPosition sp{
        units::math::min(_manualSetpoint.height, max_height),
        // trimmed_angle
        _manualSetpoint.angle
      };

      // _manualSetpoint.angle = trimmed_angle;

      _armavator->SetPosition(sp);
    }
  }
  // std::cout << "arm angle setpoint: " <<_manualSetpoint.angle.convert<units::degree>().value() << std::endl;
  // std::cout << "elevator height setpoint: " << _manualSetpoint.height.convert<units::meter>().value() << std::endl;
  // units::radian_t armPos = _armavator->arm->GetConfig().leftGearbox.encoder->GetEncoderPosition(); 
  // std::cout << "arm pos: " << armPos.value() << std::endl;
  // std::cout << "elevator pos: " << _armavator->elevator->GetConfig().leftGearbox.encoder->GetEncoderPosition().value() << std::endl;
}

/**
 * @Anna Todo list:
 * - swerve offset tuning (make sure it drives straight)
 * - tune swerve PID 
 * - tune elevator PID 
 * - controller doing PID setpoint not raw 
 * - arm encoder read out, should be 0 - 360 
 * - arm encoder zeroing 
*/