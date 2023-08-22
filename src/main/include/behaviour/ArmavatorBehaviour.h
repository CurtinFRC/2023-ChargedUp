#pragma once

#include <frc/XboxController.h>
#include "behaviour/Behaviour.h"
#include "Armavator.h"
#include "Grid.h"
#include <units/angle.h>
#include <units/length.h>
#include <iostream>
#include <frc/event/EventLoop.h>
#include "ControlUtil.h"

//Armavator states
enum class ArmavatorAutoSetpointEnum {
    kInIntake,
    kTravel,
    kFrontMidPlace,
    kFrontLowPlace,
    kBackHighPlace,
    kBackMidPlace,
    kBackLowPlace,
    kWaitToCollect
};

enum class ArmavatorManualModeEnum {
    kRaw,
    kVelocity,
    kPosition
};
//armavator go to auto setpoint 
class ArmavatorGoToAutoSetpoint : public behaviour::Behaviour {
public:
    ArmavatorGoToAutoSetpoint(Armavator *armavator, units::meter_t height, units::degree_t angle, double elevatorSpeed = 0.5, double armSpeed = 0.3);

    void OnStart();
    void OnTick(units::second_t dt) override;
private:
    Armavator *_armavator;

    units::degree_t _angle;
    units::meter_t _height;

    double _elevatorSpeed;
    double _armSpeed;
};

class ArmavatorGoToPositionBehaviour : public behaviour::Behaviour {
public:
    using grid_t = ArmavatorConfig::grid_t;

    ArmavatorGoToPositionBehaviour(Armavator *armavator, ArmavatorPosition setpoint);

    void OnStart() override;
    void OnTick(units::second_t dt) override;

private:
    Armavator *_armavator;

    ArmavatorPosition _setpoint;
    std::deque<grid_t::GridPathNode<units::second>> _waypoints;
};

class ArmavatorRawBehaviour : public behaviour::Behaviour {
public:
    using grid_t = ArmavatorConfig::grid_t;

    ArmavatorRawBehaviour(Armavator *armavator, frc::XboxController &codriver);

    void OnTick(units::second_t dt) override;
private:
    Armavator *_armavator;

    ArmavatorPosition _setpoint;
    std::deque<grid_t::GridPathNode<units::second>> _waypoints;
    frc::XboxController &_codriver;
};


class ArmavatorManualBehaviour : public behaviour::Behaviour {
public:
    ArmavatorManualBehaviour(Armavator *armavator, frc::XboxController &codriver);

    void OnStart() override;
    void OnTick(units::second_t dt) override;


private:
    Armavator *_armavator;
    ArmavatorPosition _manualSetpoint{0_m,0_rad};
    ArmavatorPosition _setpointValue;

    bool goingToSetpoint = false;

    units::meter_t _max_height = 0.7_m;
    units::meter_t _min_height = 0.01_m;

    units::degree_t _max_angle = 265_deg;
    units::degree_t _min_angle = -60_deg;

    frc::XboxController &_codriver;
    units::degree_t max_diff = 10_deg;

    units::meter_t startHeight;
    frc::EventLoop *loop;

    bool velocityControl = false;
    bool rawControl = true;



    ArmavatorManualModeEnum _armManualModes = ArmavatorManualModeEnum::kRaw;

    void SetPosition(units::degree_t angle, units::meter_t height, std::string name, double elevatorSpeed, double armSpeed);
    void CheckSetpoints();
    void ZeroElevatorEncoder();
};