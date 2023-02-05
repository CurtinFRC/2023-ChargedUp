#pragma once

#include "behaviour/Behaviour.h"
#include "drivetrain/SwerveDrive.h"

std::shared_ptr<behaviour::Behaviour> BlueSinglePiece();

std::shared_ptr<behaviour::Behaviour> CircularPathing(wom::SwerveDrive *swerve);