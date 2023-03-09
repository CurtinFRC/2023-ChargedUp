#include "behaviour/ArmavatorBehaviour.h"
#include <frc/smartdashboard/SmartDashboard.h>


ArmavatorGoToAutoSetpoint::ArmavatorGoToAutoSetpoint(Armavator *armavator, units::meter_t height, units::degree_t angle) 
  : _armavator(armavator), _height(height), _angle(angle) {
    Controls(armavator);
}

void ArmavatorGoToAutoSetpoint::OnStart() {
  std::cout << "intake + armavator auto setpoint routines start" << std::endl;
}

void ArmavatorGoToAutoSetpoint::OnTick(units::second_t dt) {
  ArmavatorPosition pos = {_height, _angle};
  _armavator->SetSpeedValues(0.35, 0.2);

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
  } else {
    if (_codriver.GetPOV() == 0) {
      //carrying 
      _setpointValue.height = 0.1_m;
      _setpointValue.angle = 30_deg;
      _armavator->SetPosition(_setpointValue);
      _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
      std::cout << "GO TO armavator POS 1 " << std::endl;
      _armavator->SetSpeedValues(0.5, 0.2);

    } else if (_codriver.GetPOV() == 90) {
      //placing front 
      _setpointValue.height = 0.5_m;
      _setpointValue.angle = 0_deg;
      _armavator->SetPosition(_setpointValue);
      _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
      std::cout << "GO TO armavator POS 2 " << std::endl;
      _armavator->SetSpeedValues(0.5, 0.2);

    } else if (_codriver.GetPOV() == 180) {
      //placing back
      _setpointValue.height = 0.5_m;
      _setpointValue.angle = 180_deg;
      _armavator->SetPosition(_setpointValue);
      _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
      std::cout << "GO TO armavator POS 3 " << std::endl;
      _armavator->SetSpeedValues(0.3, 0.1);

    } else if (_codriver.GetPOV() == 270) {
      //gripper
      //TODO potench change max speed here
      _setpointValue.height = 0.9_m;
      _setpointValue.angle = -50_deg;
      _armavator->SetPosition(_setpointValue);
      _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
      std::cout << "GO TO armavator POS 4 " << std::endl;
      _armavator->SetSpeedValues(0.35, 0.07);
    } else{
      units::meter_t height = _armavator->GetCurrentPosition().height;
      units::degree_t angle = _armavator->GetCurrentPosition().angle;
      _armavator->SetSpeedValues(0.5, 0.3);

      // if (_manualSetpoint.height > 1_m) {
      //   //to high 
      // } else if (_manualSetpoint.height < 0.01_m){
      //   //to low
      // } else {
      //   //just right 
      //   _manualSetpoint.height -= (wom::deadzone(_codriver.GetRightY(), 0.15) * 1_m * 0.05);
      // }

      // if (_manualSetpoint.angle < -60_deg) {
      //   //too far 
      // } else if (_manualSetpoint.angle > 265_deg) {
      //   //too far
      // } else {
      //   //just right 
      //   if (_manualSetpoint.height >= (42.343 * pow((angle / 1_deg), -1.053) * 1_m)) {
      //     _manualSetpoint.angle = (35.067 / pow((height / 1_m), 0.9496676)) * 1_deg;
      //     if (_codriver.GetLeftY() > 0) {
      //       _manualSetpoint.angle -= (wom::deadzone(_codriver.GetLeftY(), 0.15) * 1_deg * 3);
      //     }
      //   } else {
      //     _manualSetpoint.angle -= (wom::deadzone(_codriver.GetLeftY(), 0.15) * 1_deg * 3);
      //   }
      // }


      // if (height > (42.343 * pow((angle / 1_deg), -1.053) * 1_m) || height > 1_m) {
      //   if ((42.343 * pow((angle / 1_deg), -1.053) * 1_m) < 1_m) {
      //     _manualSetpoint.height = 42.343 * pow((angle / 1_deg), -1.053) * 1_m;
      //   } else {
      //     _manualSetpoint.height = 1_m;
      //   }
      // } else if (height < 0.01_m){
      //   _manualSetpoint.height = 0.01_m;
      // } else {
      //   _manualSetpoint.height -= (wom::deadzone(_codriver.GetRightY(), 0.15) * 1_m * 0.05);
      // }

      // if (angle <= 90_deg) {
      //   if (angle > (35.067 / pow((height / 1_m), 0.9496676)) * 1_deg) {
      //     _manualSetpoint.angle = (35.067 / pow((height / 1_m), 0.9496676)) * 1_deg;
      //   } else if (angle < -60_deg) {
      //     _manualSetpoint.angle = -60_deg;
      //   } else {
      //     _manualSetpoint.angle -= (wom::deadzone(_codriver.GetLeftY(), 0.15) * 1_deg * 3);
      //   }
      // } else if (angle > 90_deg) {
      //   // if (angle > 265_deg) {
      //   //   angle = 265_deg;
      //   // } else {
      //   //   _manualSetpoint.angle -= (wom::deadzone(_codriver.GetLeftY(), 0.15) * 1_deg * 3);
      //   // }
      //   if (angle > 265_deg) {
      //     _manualSetpoint.angle = 265_deg;
      //   } else if (angle > ((35.067 / pow((height / 1_m), 0.9496676))) * 1_deg) {
      //     _manualSetpoint.angle = (35.067 / pow((height / 1_m), 0.9496676)) * 1_deg;
      //   } else {
      //     _manualSetpoint.angle -= (wom::deadzone(_codriver.GetLeftY(), 0.15) * 1_deg * 3);
      //   }
      // }



      // if (_manualSetpoint.height > 0.95_m) {
      //   _manualSetpoint.height = 0.95_m;
      // } else if (_manualSetpoint.height < 0.01_m) {
      //   _manualSetpoint.height = 0.01_m;
      // } else {
      //   if (_manualSetpoint.height > (-0.0137 * (_manualSetpoint.angle/1_deg) + 1.2301) * 1_m) {
      //     if (_codriver.GetRightY() < -0.15) {
      //       _manualSetpoint.height -= (wom::deadzone(_codriver.GetRightY(), 0.15) * 1_m * 0.05);
      //     } else {
      //       _manualSetpoint.height = ((-0.0137 * (_manualSetpoint.angle / 1_deg)) + 1.2301) * 1_m;
      //     }
      //   } else {
      //     _manualSetpoint.height -= (wom::deadzone(_codriver.GetRightY(), 0.15) * 1_m * 0.05);
      //   }
      // }
    
      // if (_manualSetpoint.angle > 265_deg) {
      //   _manualSetpoint.angle = 265_deg;
      // } else if (_manualSetpoint.angle < -60_deg) {
      //   _manualSetpoint.angle = -60_deg;
      // } else {
      //   if (_manualSetpoint.angle > 90) {
      //     //2nd quad 

      //     if (_manualSetpoint.angle > 180 - (1/137 * (12301 - 10000 * _manualSetpoint.height))) {
      //       if (_codriver.GetLeftY() > 0.15) {
      //         _manualSetpoint.angle = 1/137 * (12301 - 10000 * _manualSetpoint.height);
      //       } else {
      //         _manualSetpoint.angle -= (wom::deadzone(_codriver.GetLeftY(), 0.15) * 1_deg * 3);
      //       }
      //     }
          
      //   } else { 
      //     //1st quad 
      //     if (_manualSetpoint.angle > 1/137 * (12301 - 10000 * _manualSetpoint.height)) {
      //       if (_codriver.GetLeftY() < -0.15) {
      //         _manualSetpoint.angle = 1/137 * (12301 - 10000 * _manualSetpoint.height);
      //       } else {
      //         _manualSetpoint.angle -= (wom::deadzone(_codriver.GetLeftY(), 0.15) * 1_deg * 3);
      //       }
      //     }
      //   }
      //   _manualSetpoint.angle -= (wom::deadzone(_codriver.GetLeftY(), 0.15) * 1_deg * 3);
      // }

      //  if (_manualSetpoint.angle > 265_deg) {
      //   _manualSetpoint.angle = 265_deg;
      // } else if (_manualSetpoint.angle < -60_deg) {
      //   _manualSetpoint.angle = -60_deg;
      // } else {
      //   _manualSetpoint.angle -= (wom::deadzone(_codriver.GetLeftY(), 0.15) * 1_deg * 3);
      // }



      
      if (_manualSetpoint.height > 0.95_m) {
        _manualSetpoint.height = 0.95_m;
      } else if (_manualSetpoint.height < 0.01_m) {
        _manualSetpoint.height = 0.01_m;
      } else {
        _manualSetpoint.height -= (wom::deadzone(_codriver.GetRightY(), 0.15) * 1_m * 0.05);
      }

      if (_manualSetpoint.angle > 265_deg) {
        _manualSetpoint.angle = 265_deg;
      } else if (_manualSetpoint.angle < -60_deg) {
        _manualSetpoint.angle = -60_deg;
      } else {
        _manualSetpoint.angle -= (wom::deadzone(_codriver.GetLeftY(), 0.15) * 1_deg * 3);
      }

      units::meter_t max_height = 1.6_m - _armavator->arm->GetConfig().armLength * units::math::sin(_manualSetpoint.angle);
      _manualSetpoint.height = units::math::min(_manualSetpoint.height, max_height);

      _armavator->SetPosition(_manualSetpoint);
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