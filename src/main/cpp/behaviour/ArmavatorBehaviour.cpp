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
    if (_armavator->GetCurrentPosition().height > 5_m) {
        _armavator->OnStart();
    }


    switch (_armManualModes) {
        case ArmavatorManualModeEnum::kRaw:
            // raw control
        {
            double armPower = -wom::deadzone(_codriver.GetLeftY());
            double elePower = -wom::deadzone(_codriver.GetRightY());
            _armavator->SetManual(armPower * 11_V, elePower * 8_V);
            _manualSetpoint = {_armavator->GetCurrentPosition().height, _armavator->GetCurrentPosition().angle};
            _armavator->SetSpeedValues(0.5, 0.3);

            std::cout << "Mode Raw" << std::endl;
        }
            break;

        case ArmavatorManualModeEnum::kVelocity:
            // velocity control
        {
            ArmavatorVelocity av;
            av.angleSpeed = wom::spow2(wom::deadzone(_codriver.GetLeftY(), 0.1)) * (180_deg / 1_s);
            av.elevatorSpeed = -wom::spow2(wom::deadzone(_codriver.GetRightY(), 0.1)) * (2_m / 1_s);

            units::meter_t height = _armavator->GetCurrentPosition().height;
            units::degree_t angle = _armavator->GetCurrentPosition().angle;
            //bool above_height = true;
            //sets hard limits in place, necessary to make the system not break itself (you will burn the neos out if they try to go past what they can)
//        if (av.elevatorSpeed > _max_height) {
//          _manualSetpoint.height = _max_height;
//        } else if (_manualSetpoint.height < _min_height) {
//          _manualSetpoint.height = _min_height;
//        }
//        else {
//            av.elevatorSpeed = -wom::spow2(wom::deadzone(_codriver.GetRightY(), 0.1)) * (2_m / 1_s);
//        }

            units::meter_t max_height = (1.7_m - 0.51_m) - _armavator->arm->GetConfig().armLength * units::math::sin(_manualSetpoint.angle);
            units::degree_t max_angle_front = units::math::acos(_armavator->GetCurrentPosition().height/_armavator->arm->GetConfig().armLength) + 10_deg - 120_deg;
            units::degree_t max_angle_back = 360_deg - units::math::acos(_armavator->GetCurrentPosition().height/_armavator->arm->GetConfig().armLength) - 5_deg - 50_deg;


            //units::meter_t height = _armavator->GetCurrentPosition().height;
            //units::degree_t angle = _armavator->GetCurrentPosition().angle;

            frc::SmartDashboard::PutNumber("Max Angle Front: ", max_angle_front.value());
            frc::SmartDashboard::PutNumber("Max Angle Back: ", max_angle_back.value());
            frc::SmartDashboard::PutNumber("Current Angle", angle.value());

            frc::SmartDashboard::PutNumber("Height", height.value());
            frc::SmartDashboard::PutNumber("Max Height", max_height.value());


            if (height >= max_height) {
                _armavator->SetElevatorPosition(max_height);
                av.elevatorSpeed = 0_mps;
            }

            if (height > 0.675_m) {
                _armavator->SetVelocity(av);
            }
            else if (angle < max_angle_front) {
                //_armavator->SetArmPosition(max_angle_front * (std::numbers::pi / 180));
                if (av.angleSpeed > (0_deg / 1_s)) {
                    _armavator->SetVelocity(av);
                }
                else {
                    av.angleSpeed = 5_deg / 1_s;
                    _armavator->SetVelocity(av);
                }
            }
            else if (angle > max_angle_back) {
                if (av.angleSpeed < (0_deg / 1_s)) {
                    _armavator->SetVelocity(av);
                }
                else {
                    av.angleSpeed = -5_deg / 1_s;
                    _armavator->SetVelocity(av);
                }

            }
            else {
                std::cout << "max_height" << std::endl;
                std::cout << max_height.value() << std::endl;

                _armavator->SetVelocity(av);
            }





            frc::SmartDashboard::PutNumber("ArmVelocitySetpoint", av.angleSpeed.value());

            CheckSetpoints();
            std::cout << "Mode Velocity" << std::endl;

        }
            break;
        case ArmavatorManualModeEnum::kPosition:
            // position control
        {

            units::meter_t height = _armavator->GetCurrentPosition().height;
            units::degree_t angle = _armavator->GetCurrentPosition().angle;
//          _armavator->SetSpeedValues(0.5, 0.3);

            CheckSetpoints();

            if (goingToSetpoint) {
                if (angle >= 90_deg) {
                    // side = back
                    if (_setpointValue.angle <= 90_deg) {
                        if (height > 0.12_m) {
                            _manualSetpoint.height = _min_height;
                        } else {
                            _manualSetpoint.angle = _setpointValue.angle;
                        }

                        if (angle < _setpointValue.angle + 2_deg && angle > _setpointValue.angle - 2_deg) {
                            _manualSetpoint.height = _setpointValue.height;
                            _manualSetpoint.angle = _setpointValue.angle;
                            goingToSetpoint = false;
                        }
                    } else {
                        _manualSetpoint.height = _setpointValue.height;
                        _manualSetpoint.angle = _setpointValue.angle;
                        goingToSetpoint = false;
                    }

                } else {
                    // side = front
                    if (_setpointValue.angle > 90_deg) {
                        if (height > 0.12_m) {
                            _manualSetpoint.height = _min_height;
                        } else {
                            _manualSetpoint.angle = _setpointValue.angle;
                        }
                        if (angle < _setpointValue.angle + 2_deg && angle > _setpointValue.angle - 2_deg) {
                            _manualSetpoint.height = _setpointValue.height;
                            _manualSetpoint.angle = _setpointValue.angle;
                            goingToSetpoint = false;
                        }
                    } else {
                        _manualSetpoint.height = _setpointValue.height;
                        _manualSetpoint.angle = _setpointValue.angle;
                        goingToSetpoint = false;
                    }
                }
            }

            //bool above_height = true;
            //sets hard limits in place, necessary to make the system not break itself (you will burn the neos out if they try to go past what they can)
            if (_manualSetpoint.height > _max_height) {
                _manualSetpoint.height = _max_height;
            } else if (_manualSetpoint.height < _min_height) {
                _manualSetpoint.height = _min_height;
            } else {
                _manualSetpoint.height -= (wom::deadzone(_codriver.GetRightY(), 0.2) * 1_m *
                                           0.1); //slows the system down, otherwise it's wayyy too fast
            }

            if (_manualSetpoint.angle > _max_angle) {
                _manualSetpoint.angle = _max_angle;
            } else if (_manualSetpoint.angle < _min_angle) {
                _manualSetpoint.angle = _min_angle;
            } else {
                _manualSetpoint.angle += (wom::deadzone(_codriver.GetLeftY(), 0.2) * 1_deg * 1.5);
            }

//        if (wom::deadzone(_codriver.GetLeftY(), 0.2) || wom::deadzone(_codriver.GetRightY(), 0.2)) {
//            _armavator->SetSpeedValues(0.5, 0.3);
//        } else {
//            _armavator->SetSpeedValues(0.3, 0.1);
//        }
            _armavator->SetSpeedValues(0.5, 0.3);


            //
//          _manualSetpoint.height -= (wom::deadzone(_codriver.GetRightY(), 0.2) * 1_m * 0.05); //slows the system down, otherwise it's wayyy too fast
//          _manualSetpoint.angle -= (wom::deadzone(_codriver.GetLeftY(), 0.2) * 1_deg * 1);

            //some funky math which makes the armavator stay within extension limits
            units::meter_t max_height =
                    (1.6_m - 0.51_m) - _armavator->arm->GetConfig().armLength * units::math::sin(_manualSetpoint.angle);
            frc::SmartDashboard::PutNumber("Max Height: ", max_height.value());
            ArmavatorPosition sp{
                    units::math::min(_manualSetpoint.height, max_height),
                    _manualSetpoint.angle
            };


            //units::meter_t max_height = (1.7_m - 0.51_m) - _armavator->arm->GetConfig().armLength * units::math::sin(_manualSetpoint.angle);
            units::degree_t max_angle_front = units::math::acos(
                    _armavator->GetCurrentPosition().height / _armavator->arm->GetConfig().armLength) + 5_deg - 120_deg;
            units::degree_t max_angle_back = 360_deg - units::math::acos(
                    _armavator->GetCurrentPosition().height / _armavator->arm->GetConfig().armLength) - 5_deg - 50_deg;


            //units::meter_t height = _armavator->GetCurrentPosition().height;
            //units::degree_t angle = _armavator->GetCurrentPosition().angle;

            frc::SmartDashboard::PutNumber("Max Angle Front: ", max_angle_front.value());
            frc::SmartDashboard::PutNumber("Max Angle Back: ", max_angle_back.value());
            frc::SmartDashboard::PutNumber("Current Angle", angle.value());

            frc::SmartDashboard::PutNumber("Current Height", height.value());
            frc::SmartDashboard::PutNumber("Max Height", max_height.value());

            frc::SmartDashboard::PutNumber("Manual Setpoint Height", _manualSetpoint.height.value());
            frc::SmartDashboard::PutNumber("Manual Setpoint Angle", _manualSetpoint.angle.value());


            if (height > 0.675_m) {
                _armavator->SetPosition(sp);
            } else if (angle < max_angle_front) {
                //_armavator->SetArmPosition(max_angle_front * (std::numbers::pi / 180));
                if (sp.angle > max_angle_front) {
                    _armavator->SetPosition(sp);
                } else {
                    //sp.angle = max_angle_front / 60;
                    _manualSetpoint.angle = max_angle_front;
                    _armavator->SetPosition(sp);
                }
            } else if (angle > max_angle_back) {
                if (sp.angle < max_angle_back) {
                    _armavator->SetPosition(sp);
                } else {
                    _manualSetpoint.angle = max_angle_back;
                    _armavator->SetPosition(sp);
                }

            } else {
                //if (wom::deadzone(_codriver.GetLeftY(), 0.2)) {
                //_manualSetpoint.angle -= (wom::deadzone(_codriver.GetLeftY(), 0.2) * 1_deg * 1);
                //}
                //if (wom::deadzone(_codriver.GetRightY(), 0.2)) {
                //_manualSetpoint.height -= (wom::deadzone(_codriver.GetRightY(), 0.2) * 1_m * 0.05); //slows the system down, otherwise it's wayyy too fast
                //}
                _armavator->SetPosition(sp);

                frc::SmartDashboard::PutNumber("Target Angle", _manualSetpoint.angle.value());
                frc::SmartDashboard::PutNumber("Target Height", _manualSetpoint.height.value());


            }


            //print the values, REMOVE THESE BEFORE WARP, too many print out clog the system and can make the robot loose connection to the field
            std::cout << "set position height: " << _manualSetpoint.height.value() << std::endl;
            std::cout << "set position angle: " << _manualSetpoint.angle.value() << std::endl;


            break;
        }


    }

    if (_codriver.GetAButtonPressed()) {
        _armManualModes = ArmavatorManualModeEnum::kRaw;
    }
    else if (_codriver.GetLeftBumperPressed()) {
        _armManualModes = ArmavatorManualModeEnum::kVelocity;
    }
    else if (_codriver.GetRightBumperPressed()) {
        _armManualModes = ArmavatorManualModeEnum::kPosition;
        units::meter_t height = _armavator->GetCurrentPosition().height;
        units::degree_t angle = _armavator->GetCurrentPosition().angle;

        _manualSetpoint.angle = angle;
        _manualSetpoint.height = height;
    } else if (_codriver.GetStartButton()) {
        ZeroElevatorEncoder();
    }
}

void ArmavatorManualBehaviour::CheckSetpoints() {

    if (_codriver.GetPOV() == 0) {
        //BACK SINGLE

        SetPosition(160_deg, 0.01_m, "1", 0.5, 0.2);
    } else if (_codriver.GetPOV() == 90) {
        //BACK DOUBLE

        SetPosition(183_deg, 1.3_m, "2", 0.5, 0.2);
    } else if (_codriver.GetPOV() == 180) {
        //FRONT DOUBLE

        SetPosition(-8_deg, 1.3_m, "3", 0.3, 0.1);
    } else if (_codriver.GetPOV() == 270) {
        // FRONT SINGLE bottom elevator mid angle
        // FRONT DOUBLE almost top elevator 180 angle

        // FRONT SINGLE

        SetPosition(20_deg, 0.01_m, "4", 0.35, 0.07);
    } else if (_codriver.GetXButton()) {
        //FRONT MID

        SetPosition(-7_deg, 0.7_m, "5", 0.35, 0.07);
    } else if (_codriver.GetYButton()) {
        // BACK HIGH

        SetPosition(172_deg, 1.7_m, "6", 0.35, 0.07);
    } else if (_codriver.GetBButton()) {
        // BACK MID

        SetPosition(187_deg, 0.7_m, "7", 0.35, 0.07);

        std::cout << "Mode Position" << std::endl;

    }

    return;
}

void ArmavatorManualBehaviour::ZeroElevatorEncoder() {
    _armavator->ZeroElevatorEncoder();
}


void ArmavatorManualBehaviour::SetPosition(units::degree_t setpoint_angle, units::meter_t setpoint_height, std::string name, double elevatorSpeed, double armSpeed) {
    if (_armManualModes == ArmavatorManualModeEnum::kPosition) {
        _setpointValue.height = setpoint_height;
        _setpointValue.angle = setpoint_angle;
        goingToSetpoint = true;



    }
    else if (_armManualModes == ArmavatorManualModeEnum::kVelocity) {
        units::meter_t height = _armavator->GetCurrentPosition().height;
        units::degree_t angle = _armavator->GetCurrentPosition().angle;
        _armavator->SetSpeedValues(0.5, 0.3);
        //bool above_height = true;
        //sets hard limits in place, necessary to make the system not break itself (you will burn the neos out if they try to go past what they can)
        if (_manualSetpoint.height > _max_height) {
            _manualSetpoint.height = _max_height;
        } else if (_manualSetpoint.height < _min_height) {
            _manualSetpoint.height = _min_height;
        }
        else {
            _manualSetpoint.height = setpoint_height;
        }

        if (_manualSetpoint.angle > _max_angle) {
            _manualSetpoint.angle = _max_angle;
        } else if (_manualSetpoint.angle < _min_angle) {
            _manualSetpoint.angle = _min_angle;
        }
        else {
            _manualSetpoint.angle = setpoint_angle;
        }

        //
        //          _manualSetpoint.height -= (wom::deadzone(_codriver.GetRightY(), 0.2) * 1_m * 0.05); //slows the system down, otherwise it's wayyy too fast
        //          _manualSetpoint.angle -= (wom::deadzone(_codriver.GetLeftY(), 0.2) * 1_deg * 1);

        //some funky math which makes the armavator stay within extension limits
        units::meter_t max_height = (1.6_m - 0.51_m) - _armavator->arm->GetConfig().armLength * units::math::sin(_manualSetpoint.angle);
        ArmavatorPosition sp{
                units::math::min(_manualSetpoint.height, max_height),
                _manualSetpoint.angle
        };


        //units::meter_t max_height = (1.7_m - 0.51_m) - _armavator->arm->GetConfig().armLength * units::math::sin(_manualSetpoint.angle);
        units::degree_t max_angle_front = units::math::acos(_armavator->GetCurrentPosition().height/_armavator->arm->GetConfig().armLength) + 10_deg - 120_deg;
        units::degree_t max_angle_back = 360_deg - units::math::acos(_armavator->GetCurrentPosition().height/_armavator->arm->GetConfig().armLength) - 5_deg - 50_deg;


        //units::meter_t height = _armavator->GetCurrentPosition().height;
        //units::degree_t angle = _armavator->GetCurrentPosition().angle;

        frc::SmartDashboard::PutNumber("Max Angle Front: ", max_angle_front.value());
        frc::SmartDashboard::PutNumber("Max Angle Back: ", max_angle_back.value());
        frc::SmartDashboard::PutNumber("Current Angle", angle.value());

        frc::SmartDashboard::PutNumber("Height", height.value());
        frc::SmartDashboard::PutNumber("Max Height", max_height.value());



        if (height > 0.675_m) {
            _armavator->SetPosition(sp);
        }
        else if (angle < max_angle_front) {
            //_armavator->SetArmPosition(max_angle_front * (std::numbers::pi / 180));
            if (sp.angle > max_angle_front) {
                _armavator->SetPosition(sp);
            }
            else {
                //sp.angle = max_angle_front / 60;
                _manualSetpoint.angle = max_angle_front;
                _armavator->SetPosition(sp);
            }
        }
        else if (angle > max_angle_back) {
            if (sp.angle < max_angle_back) {
                _armavator->SetPosition(sp);
            }
            else {
                _manualSetpoint.angle = max_angle_back;
                _armavator->SetPosition(sp);
            }

        }
        else {
            //if (wom::deadzone(_codriver.GetLeftY(), 0.2)) {
            //_manualSetpoint.angle -= (wom::deadzone(_codriver.GetLeftY(), 0.2) * 1_deg * 1);
            //}
            //if (wom::deadzone(_codriver.GetRightY(), 0.2)) {
            //_manualSetpoint.height -= (wom::deadzone(_codriver.GetRightY(), 0.2) * 1_m * 0.05); //slows the system down, otherwise it's wayyy too fast
            //}
            _armavator->SetPosition(sp);

            frc::SmartDashboard::PutNumber("Current Angle", _manualSetpoint.angle.value());
            frc::SmartDashboard::PutNumber("Current Height", _manualSetpoint.height.value());


        }


    }
}