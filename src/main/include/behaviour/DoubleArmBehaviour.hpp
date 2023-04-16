#pragma once

#include "DoubleArm.hpp"
#include "behaviour/Behaviour.h"

#include <frc/XboxController.h>
#include <math.h>
#include <units/math.h>

enum class DoubleArmSetpointStates {
  
};

class DoubleArmGoToAutoSetPoint : public behaviour::Behaviour {
  public :
    DoubleArmGoToAutoSetPoint(DoubleArm *doubleArm, units::radian_t baseAngle, units::radian_t extensionAngle, double extensionSpeed = 0.3, double baseSpeed = 0.3);

    void OnStart();
    void OnTick(units::second_t dt) override;

  private :
  DoubleArm *_doubleArm;

  units::radian_t _baseAngle;
  units::radian_t _extensionAngle;

  double _extensionSpeed;
  double _baseSpeed;

  DoubleArmSetpointStates _setpoint;
  DoubleArmPos _setpointVal;
};

class RawControlBehaviour : public behaviour::Behaviour {
  public :
    RawControlBehaviour(DoubleArm *doubleArm, frc::XboxController &codriver);

    void OnStart() override;
    void OnTick(units::second_t dt) override;
  private :
    DoubleArm *_doubleArm;

    DoubleArmPos _setpoint;
    frc::XboxController &_codriver;
};

class ManualControlBehaviour : public behaviour::Behaviour {
  public :
    ManualControlBehaviour(DoubleArm *doubleArm, frc::XboxController &codriver);

    void OnStart() override;
    void OnTick(units::second_t dt) override;
  private :
  DoubleArm *_doubleArm;
  DoubleArmPos _manualSetpoint;
  DoubleArmPos _setpointVal;

  frc::XboxController &_codriver;
  
  units::radian_t startAngle;
  frc::EventLoop *loop;

  bool manualControl = true;
};