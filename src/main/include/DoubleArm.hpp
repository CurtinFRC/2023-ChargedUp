#pragma once

#include <ctre/Phoenix.h>
#include <units/math.h>
#include <units/velocity.h>

#include "Gearbox.h"
#include "Arm.h"


struct DoubleArmConfig {
  wom::ArmConfig baseArm;
  wom::ArmConfig extendedArm;
};

struct DoubleArmPos {
  units::radian_t baseAngle;
  units::radian_t extensionAngle;
};

enum class DoubleArmState {
  kIdle,
  kAngleBase,
  kAngleExtension,
  kAngle,
  kRaw,
  kPos,
  kManual
};

class DoubleArm : public  behaviour::HasBehaviour {
  public :

  DoubleArm(wom::Gearbox _baseControl, wom::Gearbox _connectorControl);

  // set info for states
  void SetIdle();
  void SetPosition(DoubleArmPos pos);
  void SetZeroing();
  void SetManual(units::volt_t BaseArm, units::volt_t extensionArm);
  void SetSpeedValues(double baseArmSpeed, double extensionArmSpeed);

  DoubleArmPos GetCurrentPos();

  void OnStart();
  void OnUpdate(units::second_t dt);

  // creates both arms
  wom::Arm *_baseArm;
  wom::Arm *_extensionArm;
  DoubleArmPos _setpoint;

  private :
    DoubleArmState _state = DoubleArmState::kIdle;

    // creates an instance of double arm
    DoubleArmConfig *_config;
    wom::Gearbox *_baseControl;
    wom::Gearbox *_connectorControl;

    units::volt_t _extensionArmRaw;
    units::volt_t _baseArmRaw;

  protected :
    DoubleArmPos _zeroed;
};