#include "behaviour/VisionBehaviour.h"

// VisionBehaviour::VisionBehaviour(Vision *vision, frc::XboxController *codriver) 
//   : _vision(vision), _codriver(codriver) 
//   {
//     Controls(vision);
//   }

VisionBehaviour::VisionBehaviour(Vision *vision, wom::SwerveDrive *swerveDrivebase, frc::XboxController *codriver) : _vision(vision), _swerveDrivebase(swerveDrivebase), _codriver(codriver)
  {
    Controls(_vision);
  }

void VisionBehaviour::OnTick(units::second_t dt) {
  
}