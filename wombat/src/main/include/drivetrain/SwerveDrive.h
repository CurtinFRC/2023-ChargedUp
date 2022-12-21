#pragma once 

#include "Gearbox.h"
#include <frc/SpeedController.h>
#include "behaviour/HasBehaviour.h"
#include "behaviour/Behaviour.h"
#include <frc/interfaces/Gyro.h>

namespace wom {
  enum class SwerveDriveState {
    
  };

  struct SwerveDriveConfig {

  };

  class SwerveDrive : public behaviour::HasBehaviour {
   public:
    SwerveDrive(SwerveDriveConfig config);
   protected:

   private:
    SwerveDriveConfig _config;
  };
}

