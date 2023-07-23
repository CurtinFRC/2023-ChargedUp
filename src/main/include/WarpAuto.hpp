#pragma once

#include "behaviour/Behaviour.h"
#include "behaviour/SwerveBaseBehaviour.h"
#include "behaviour/ArmavatorBehaviour.h"
#include "behaviour/LimelightBehaviours.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/event/BooleanEvent.h>


class Auto {
 public:
  Auto();

  struct Drivebase {
    wom::SwerveDrive *swerve;
    wom::NavX *gyro;
  };

  std::shared_ptr<behaviour::Behaviour> Balance();
  std::shared_ptr<behaviour::Behaviour> BalanceAndHighPlace();
  std::shared_ptr<behaviour::Behaviour> BalanceGrabAndHighPlace();
  std::shared_ptr<behaviour::Behaviour> HighPlace();
  std::shared_ptr<behaviour::Behaviour> Quad();
  std::shared_ptr<behaviour::Behaviour> Steal();

  frc::SendableChooser<std::string> m_chooser;
  std::string m_autoSelected;

 private:

  const std::string kBalance = "kBalance";
  const std::string kBalanceAndHighPlace = "kBalanceAndHighPlace";
  const std::string kBalanceGrabAndHighPlace = "kBalanceGrabAndHighPlace";
  const std::string kHighPlace = "kHighPlace";
  const std::string kQuad = "kQuad";
  const std::string kSteal = "kSteal";
};
