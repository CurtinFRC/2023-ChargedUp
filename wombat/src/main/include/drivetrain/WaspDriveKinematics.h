#include <frc/kinematics/ChassisSpeeds.h>
#include <Eigen/Core>

namespace wom {
  struct WaspWheelSpeeds {
    units::meters_per_second_t left, right, rear;
  };

  class WaspDriveKinematics {
   public:
    WaspDriveKinematics(units::meter_t trackWidth, units::meter_t trackLength) {
      invKinematics <<  1, -1, -(trackWidth / 2 + trackLength / 2),
                        1,  1, (trackWidth / 2 + trackLength / 2),
                        0, -1, trackLength / 2;
    }

    WaspWheelSpeeds ToWheelSpeeds(const frc::ChassisSpeeds &chassis) const {
      auto speeds = invKinematics * Eigen::Vector3d{chassis.vx.value(), chassis.vy.value(), chassis.omega.value()};
      return WaspWheelSpeeds{
        units::meters_per_second_t{speeds(0)},
        units::meters_per_second_t{speeds(1)},
        units::meters_per_second_t{speeds(2)}
      };
    }
   private:
    Eigen::Matrix3d invKinematics;
  };
}