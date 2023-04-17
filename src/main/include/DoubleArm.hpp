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
  kRaw,
  kPos,
  kManual
};

struct ConfigSpacePos {
      units::meter_t X;
      units::meter_t Y;
    };

class DoubleArm : public  behaviour::HasBehaviour {
  public :
  DoubleArm(DoubleArmConfig _config, frc::XboxController codriver);
  // DoubleArm(DoubleArmConfig _config, wom::Gearbox _baseArm, wom::Gearbox _connectorControl)

  // set info for states
  void SetIdle();
  void SetPosition(DoubleArmPos pos);
  void SetZeroing();
  void SetManual(units::volt_t BaseArm, units::volt_t extensionArm);
  void SetSpeedValues(double baseArmSpeed, double extensionArmSpeed);
  bool IsStable();
  DoubleArmConfig *GetConfig() { return _config; };
  DoubleArmPos GetCurrentPos();
  
  ConfigSpacePos GetEndEffectorPos(units::meter_t r1, units::meter_t r2, units::radian_t baseAngle, units::radian_t extensionAngle) {
      return ConfigSpacePos{((units::math::sin(baseAngle) * r1) + (units::math::sin(extensionAngle) * r2)).value() * 1_m, ((units::math::cos(baseAngle) * r1)  + (units::math::cos(extensionAngle) * r2)).value() * 1_m};
  };
  void OnStart();
  void OnUpdate(units::second_t dt);

  // creates both arms
  wom::Arm *_baseArm;
  wom::Arm *_extensionArm;
  DoubleArmPos _setpoint;
  units::radian_t _inputAngleBase;
  units::radian_t _inputAngleExtension;
  private :
    DoubleArmState _state = DoubleArmState::kIdle;

    // creates an instance of double arm
    DoubleArmConfig *_config;
    // wom::Gearbox *_baseControl;
    // wom::Gearbox *_connectorControl;

    units::volt_t _extensionArmRaw;
    units::volt_t _baseArmRaw;

    frc::XboxController &_codriver;

  protected :
    DoubleArmPos _zeroed;
};