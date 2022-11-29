#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

double currentTimeStamp;
double lastTimeStamp;
double dt;

void Robot::RobotInit() {
  driver = new frc::XboxController(0);

  leftDrive = new TalonFX(6);
  rightDrive = new TalonFX(7);


  dropWheelOne = new TalonSRX(4);
  dropWheelTwo = new TalonSRX(5);

  ahrs = new AHRS(SPI::Port::kMXP);

}
void Robot::RobotPeriodic() {
  currentTimeStamp = (double)frc::Timer::GetFPGATimestamp();
  dt = currentTimeStamp - lastTimeStamp;


  lastTimeStamp = currentTimeStamp;
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {

  if (!(driver->GetBButton())) {
    double leftSpeed = fabs(driver->GetLeftY()) > 0.1 ? driver->GetLeftY() : 0;
    double rightSpeed = fabs(driver->GetRightY()) > 0.1 ? driver->GetRightY() : 0;

    if (driver->GetAButtonReleased()) {
      if (dropToggle) {
        dropToggle = false;
      } else {
        dropToggle = true;
      }
    }

    std::cout << ahrs->GetYaw() << std::endl;

    if (dropToggle) {
      dropSpeed = fabs(driver->GetRightX()) > 0.3 ? driver->GetRightX() : 0;
      std::cout << "Right Side only" << std::endl;
    } else {
      dropSpeed = (fabs(driver->GetRightX()) > 0.3 && fabs(driver->GetLeftX()) > 0.3) ? driver->GetRightX() : 0;
      std::cout << "both" << std::endl;
    }

    leftDrive->Set(ControlMode::PercentOutput, -leftSpeed);
    rightDrive->Set(ControlMode::PercentOutput, rightSpeed);

    dropWheelOne->Set(ControlMode::PercentOutput, dropSpeed);
    dropWheelTwo->Set(ControlMode::PercentOutput, dropSpeed);
  } else {
    double input = ahrs->GetYaw();
    double error = 0 - input; // is the goal
    double derror = (error - _previousError) / dt;
    _sum += error * dt;

    double output = kP * error + kI * _sum + kD * derror;
    dropPIDSpeed = output * 10;
    std::cout << "output speed: " << dropPIDSpeed << std::endl;

    // leftDrive->Set(ControlMode::PercentOutput, 0.3); //set this to be a variable speed later 
    // rightDrive->Set(ControlMode::PercentOutput, -0.3);

    dropWheelOne->Set(ControlMode::PercentOutput, dropPIDSpeed);
    dropWheelTwo->Set(ControlMode::PercentOutput, dropPIDSpeed);

    _previousError = error;
  }
  

}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}