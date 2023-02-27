#include "behaviour/ArmavatorBehaviour.h"

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
  _setpoint.height = 0.0_m;
  _setpoint.angle = 0.0_deg;
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

  if (_codriver.GetAButtonPressed()) {
    if (rawControl) {
      _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
      rawControl = false;
    } else {
      rawControl = true;
    }
  }

  if (rawControl) {
    double armPower = wom::deadzone(_codriver.GetLeftY());
    double elePower = wom::deadzone(_codriver.GetRightY());
    _armavator->SetManual(armPower * 11_V, elePower * 8_V);
  } else {
    if (wom::deadzone(_codriver.GetLeftY())) {
      _manualSetpoint.angle -= (_codriver.GetLeftY() * 1_deg * 0.4);
    }
    if (wom::deadzone(_codriver.GetRightY())) {
      _manualSetpoint.height -= (_codriver.GetRightY() * 1_m * 0.4);
    }
    _manualSetpoint = {0.5_m, 400_deg};
    _armavator->SetPosition(_manualSetpoint);
  }
  // std::cout << "arm angle setpoint: " <<_manualSetpoint.angle.convert<units::degree>().value() << std::endl;
  // std::cout << "elevator height setpoint: " << _manualSetpoint.height.convert<units::meter>().value() << std::endl;
  // units::radian_t armPos = _armavator->arm->GetConfig().leftGearbox.encoder->GetEncoderPosition(); 
  // std::cout << "arm pos: " << armPos.value() << std::endl;
  // std::cout << "elevator pos: " << _armavator->elevator->GetConfig().leftGearbox.encoder->GetEncoderPosition().value() << std::endl;
}