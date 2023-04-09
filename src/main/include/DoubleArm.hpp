#pragma once

#include <ctre/Phoenix.h>
#include <units/math.h>
#include <units/velocity.h>
#include <frc/XboxController.h>


#include "Gearbox.h"
#include "Arm.h"


struct DoubleArmConfig {
  wom::Arm baseArm;
  wom::Arm extensionArm;

// not sure whether or not to define here or in class
// will do here for now
  wom::Gearbox baseControl;
  wom::Gearbox connectorControl;
};

struct DoubleArmPos {
  units::radian_t baseAngle;
  units::radian_t extensionAngle;
};

enum class DoubleArmState {
  kIdle,
  kAngleBase,
  kAngleExtension,
  kPos,
  kManual
};

class DoubleArm : public  behaviour::HasBehaviour {
  public :
  DoubleArm(DoubleArmConfig _config);
  // DoubleArm(DoubleArmConfig _config, wom::Gearbox _baseArm, wom::Gearbox _connectorControl)

  // set info for states
  void SetIdle();
  void SetPosition(DoubleArmPos pos);
  void SetZeroing();
  void SetManual(units::volt_t BaseArm, units::volt_t extensionArm);
  void SetSpeedValues(double baseArmSpeed, double extensionArmSpeed);
  bool IsStable();
  DoubleArmConfig *GetConfig();
  DoubleArmPos GetCurrentPos();

  void OnStart();
  void OnUpdate(units::second_t dt);

  // creates both arms
  wom::Arm *_baseArm;
  wom::Arm *_extensionArm;
  DoubleArmPos _setpoint;
  units::radian_t _inputAngle;

  private :
    DoubleArmState _state = DoubleArmState::kIdle;

    // creates an instance of double arm
    DoubleArmConfig *_config;
    // wom::Gearbox *_baseControl;
    // wom::Gearbox *_connectorControl;

    units::volt_t _extensionArmRaw;
    units::volt_t _baseArmRaw;

  protected :
    DoubleArmPos _zeroed;
};