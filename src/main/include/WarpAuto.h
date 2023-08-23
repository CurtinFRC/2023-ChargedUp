#pragma once

#include "behaviour/Behaviour.h"
#include "behaviour/SwerveBaseBehaviour.h"
#include "behaviour/ArmavatorBehaviour.h"
#include "behaviour/LimelightBehaviours.h"
#include "behaviour/GripperBehaviour.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/event/BooleanEvent.h>

using namespace behaviour;

struct Drivebase {
    wom::SwerveDrive *swerve;
    Pigeon2 *gyro;
};

std::shared_ptr<Behaviour> Taxi(Drivebase _drivebase, Armavator *_armavator);
std::shared_ptr<Behaviour> HighPlace(Armavator *_armavator, Gripper *_gripper, Drivebase _drivebase); // first
std::shared_ptr<Behaviour> Balance(Drivebase _drivebase, Armavator *_armavator); // second
std::shared_ptr<Behaviour> MidPlace(Armavator *_armavator, Gripper *_gripper, Drivebase _drivebase);
std::shared_ptr<Behaviour> BalanceAndHighPlace(Drivebase _drivebase, Armavator *_armavator, Gripper *_gripper); // third
std::shared_ptr<Behaviour> BalanceAndMidPlace(Drivebase _drivebase, Armavator *_armavator, Gripper *_gripper);

std::shared_ptr<Behaviour> SmallDrivebaseMovement(Drivebase _drivebase);
std::shared_ptr<Behaviour> SmallArmMovement(Armavator *_armavator);
// std::shared_ptr<behaviour::Behaviour> BalanceGrabAndHighPlace(); // fourth
// std::shared_ptr<behaviour::Behaviour> Steal(); // fifth
// std::shared_ptr<behaviour::Behaviour> Quad(); // last

//frc::SendableChooser<std::string> m_chooser;
//std::string m_autoSelected;

// const std::string kBalance = "kBalance";
// const std::string kBalanceAndHighPlace = "kBalanceAndHighPlace";
// const std::string kBalanceGrabAndHighPlace = "kBalanceGrabAndHighPlace";
// const std::string kHighPlace = "kHighPlace";
// const std::string kQuad = "kQuad";
// const std::string kSteal = "kSteal";