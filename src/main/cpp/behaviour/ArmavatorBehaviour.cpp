#include "behaviour/ArmavatorBehaviour.h"
#include <frc/smartdashboard/SmartDashboard.h>

//used to make the armavator move in autonomous, inputs a height for the elevator, angle for the arm (0 is wherever the arm is started when it's turned on), the elevator and armspeed which is a value from 0-1.
ArmavatorGoToAutoSetpoint::ArmavatorGoToAutoSetpoint(Armavator *armavator, units::meter_t height, units::degree_t angle, double elevatorSpeed, double armSpeed) 
  : _armavator(armavator), _height(height), _angle(angle), _elevatorSpeed(elevatorSpeed), _armSpeed(armSpeed) {
    Controls(armavator);
}

//run once when the behaviour starts
void ArmavatorGoToAutoSetpoint::OnStart() {
  std::cout << "armavator auto setpoint routines start" << std::endl;
}

void ArmavatorGoToAutoSetpoint::OnTick(units::second_t dt) {
  ArmavatorPosition pos = {_height, _angle};
  _armavator->SetSpeedValues(_elevatorSpeed, _armSpeed);

  //sets the position of the armavator, once the system is stable it sets the behaviour to finished
  _armavator->SetPosition(pos);
  if (_armavator->IsStable()) {
    SetDone();
  }
}

//used in teleop to make the armavator move, takes in the armavator system and the setpoint
ArmavatorGoToPositionBehaviour::ArmavatorGoToPositionBehaviour(Armavator *armavator, ArmavatorPosition setpoint)
: _armavator(armavator), _setpoint(setpoint) {
  Controls(armavator);
};

//runs once the behaviour starts 
void ArmavatorGoToPositionBehaviour::OnStart() {
  std::cout << "On Start" << std::endl;
}

//sets the position of the armavator
void ArmavatorGoToPositionBehaviour::OnTick(units::second_t dt) {

  //sets hard limits the armavator can be set to
  if(_setpoint.height < 1_m) {
    if (_setpoint.angle >  0_rad){
      _setpoint.angle = 0_rad;
    }
    if (_setpoint.angle > 90_rad){
      _setpoint.angle = 90_rad;
    }
  };
  _armavator->SetPosition(_setpoint);

  //if the armavator is finished moving then the behaviour is maked done
  if (_armavator->IsStable())
    SetDone();
}

//Used for complete raw control of the system. 
ArmavatorRawBehaviour::ArmavatorRawBehaviour(Armavator *armavator, frc::XboxController &codriver)
: _armavator(armavator), _codriver(codriver) {
  Controls(armavator);
};

void ArmavatorRawBehaviour::OnTick(units::second_t dt) {
  //Raw Positioning
  _armavator->SetManual(
    -_codriver.GetLeftY() * 9_V, //this is essentially the max voltage
    -_codriver.GetRightY() * 9_V
  );
}


//Kind of a bad way to do this, but this is the main behaviour called in teleop 
ArmavatorManualBehaviour::ArmavatorManualBehaviour(Armavator *armavator, frc::XboxController &codriver) 
  : _armavator(armavator), _codriver(codriver) {
    Controls(armavator);
}

//Runs when the behaviour is started 
void ArmavatorManualBehaviour::OnStart() {
  _armavator->OnStart();
}

void ArmavatorManualBehaviour::OnTick(units::second_t dt) {
  //pushes the armavator mode to smart dashboard 
  if (!rawControl) {
    frc::SmartDashboard::PutString("Armavator Mode", "Position Control");
  } else if (velocityControl) {
    frc::SmartDashboard::PutString("Armavator Mode", "Velocity Control");
  } else {
    frc::SmartDashboard::PutString("Armavator Mode", "Raw Control");
  }

  //there is a weird sensor glitch where occationally the encoder starts with a really big number, so we just re-zero it if this happens
  if (_armavator->GetCurrentPosition().height > 5_m) {
    _armavator->OnStart();
  }

  //By default the armavator is on raw control, when codriver presses A it switches to PID mode. 
  if (_codriver.GetAButtonPressed()) {
    if (rawControl) {
      rawControl = false;
    } else {
      rawControl = true;
    }
  }

  if (rawControl) {
    //raw control, no limits, no bounds, used only when something is going wrong. 
    double armPower = -wom::deadzone(_codriver.GetLeftY());
    double elePower = -wom::deadzone(_codriver.GetRightY());
    _armavator->SetManual(armPower * 11_V, elePower * 8_V);
    _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
    _armavator->SetSpeedValues(0.5, 0.3);

  } else {
    //setpoints, use to get to an exact position when you are in the same quadrant, is too violent if you are not already close. 
    if (_codriver.GetPOV() == 0) {
      //picking up cone down 
      _setpointValue.height = 0.896_m;
      _setpointValue.angle = 0_deg;
      _armavator->SetPosition(_setpointValue);
      _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
      std::cout << "GO TO armavator POS 1 " << std::endl;
      _armavator->SetSpeedValues(0.5, 0.2);
    } else if (_codriver.GetPOV() == 90) {
      //picking up cone up 
      _setpointValue.height = 0.01_m;
      _setpointValue.angle = 37.4_deg;    
      _armavator->SetPosition(_setpointValue);
      _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
      std::cout << "GO TO armavator POS 2 " << std::endl;
      _armavator->SetSpeedValues(0.5, 0.2);
    } else if (_codriver.GetPOV() == 180) {
      //picking up cone down to collect 
      _setpointValue.height = 0.896_m;
      _setpointValue.angle = -6_deg;
      _armavator->SetPosition(_setpointValue);
      _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
      std::cout << "GO TO armavator POS 3 " << std::endl;
      _armavator->SetSpeedValues(0.3, 0.1);
    } else if (_codriver.GetPOV() == 270) {
      //holding
      _setpointValue.height = 0.1_m;
      _setpointValue.angle = 60_deg;
      _armavator->SetPosition(_setpointValue);
      _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
      std::cout << "GO TO armavator POS 4 " << std::endl;
      _armavator->SetSpeedValues(0.35, 0.07);
    } else if (_codriver.GetXButton()) {
      //front mid place 
      _setpointValue.height = 0.15_m;
      _setpointValue.angle = 30_deg;
      _armavator->SetPosition(_setpointValue);
      _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
      std::cout << "GO TO armavator POS 5 " << std::endl;
      _armavator->SetSpeedValues(0.35, 0.07);
    } else if (_codriver.GetYButton()) {
      // 152_deg 0.1814_m back high place 
      _setpointValue.height = 0.1814_m;
      _setpointValue.angle = 152_deg;
      _armavator->SetPosition(_setpointValue);
      _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
      std::cout << "GO TO armavator POS 6 " << std::endl;
      _armavator->SetSpeedValues(0.35, 0.07);
    } else if (_codriver.GetBButton()) {
      _setpointValue.height = 0.0_m;
      _setpointValue.angle = 161_deg;
      _armavator->SetPosition(_setpointValue);
      _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
      std::cout << "GO TO armavator POS 7 " << std::endl;
      _armavator->SetSpeedValues(0.35, 0.07);
    } else {
      //change to velocity control 
      if (_codriver.GetLeftBumperPressed()) {
        velocityControl = true;
      } else {
        velocityControl = false;
      }
    }

    if (velocityControl) {
      //velocity control is a better way of controlling this system, makes it more responsive, making it easier to drive
      ArmavatorVelocity av;
      av.angleSpeed = wom::spow2(wom::deadzone(_codriver.GetLeftY(), 0.1)) * (180_deg / 1_s);
      av.elevatorSpeed = -wom::spow2(wom::deadzone(_codriver.GetRightY(), 0.1)) * (2_m / 1_s);
      _armavator->SetVelocity(av);
      frc::SmartDashboard::PutNumber("ArmVelocitySetpoint", av.angleSpeed.value());
    } else {
      units::meter_t height = _armavator->GetCurrentPosition().height;
      units::degree_t angle = _armavator->GetCurrentPosition().angle;
      _armavator->SetSpeedValues(0.5, 0.3);

      //sets hard limits in place, necessary to make the system not break itself (you will burn the neos out if they try to go past what they can)
      if (_manualSetpoint.height > 0.95_m) {
        _manualSetpoint.height = 0.95_m;
      } else if (_manualSetpoint.height < 0.01_m) {
        _manualSetpoint.height = 0.01_m;
      } else {
        _manualSetpoint.height -= (wom::deadzone(_codriver.GetRightY(), 0.2) * 1_m * 0.05); //slows the system down, otherwise it's wayyy too fast 
      }

      if (_manualSetpoint.angle > 265_deg) {
        _manualSetpoint.angle = 265_deg;
      } else if (_manualSetpoint.angle < -60_deg) {
        _manualSetpoint.angle = -60_deg;
      } else {
        _manualSetpoint.angle -= (wom::deadzone(_codriver.GetLeftY(), 0.2) * 1_deg * 1);
      }

      //some funky math which makes the armavator stay within extension limits 
      units::meter_t max_height = (1.9_m - 0.51_m) - _armavator->arm->GetConfig().armLength * units::math::sin(_manualSetpoint.angle);
      ArmavatorPosition sp{
        units::math::min(_manualSetpoint.height, max_height),
        _manualSetpoint.angle
      };
      _armavator->SetPosition(sp);

      //print the values, REMOVE THESE BEFORE WARP, too many print out clog the system and can make the robot loose connection to the field
      std::cout << "set position height: " << _manualSetpoint.height.value() << std::endl;
      std::cout << "set position angle: " << _manualSetpoint.angle.value() << std::endl;
    }
  }
}
