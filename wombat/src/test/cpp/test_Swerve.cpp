#include <gtest/gtest.h>

#include "drivetrain/SwerveDrive.h"

#include "FakeVoltageController.h"
#include "FakeEncoder.h"

#include <frc/simulation/DCMotorSim.h>

#include <fstream>

using namespace wom;

class SwerveModuleTest : public ::testing::Test {
 public:
  FakeVoltageController driveMotor, turnMotor;
  FakeEncoder driveEncoder{1024}, turnEncoder{1024};

  Gearbox drive{ &driveMotor, &driveEncoder, DCMotor::NEO(1).WithReduction(8.14) };
  Gearbox turn{ &turnMotor, &turnEncoder, DCMotor::NEO(1).WithReduction(150 / 7.0) };

  SwerveModuleConfig config{
    frc::Translation2d{0_m, 0_m},
    drive, turn,
    4_in / 2,
  };

  SwerveModule::angle_pid_conf_t anglePID{
    12_V / 90_deg,
  };

  SwerveModule::velocity_pid_conf_t velocityPID{
    12_V / 1_mps
  };

  SwerveModule mod{config, anglePID, velocityPID};

  frc::sim::DCMotorSim turnSim{ turn.motor.ToWPI(), 1.0, 0.5 * 6_lb * 7.5_in * 7.5_in };
  frc::sim::DCMotorSim driveSim{ drive.motor.ToWPI(), 1.0, units::kilogram_square_meter_t{0.1} };
};

TEST_F(SwerveModuleTest, Simple) {
  std::ofstream out{"swerve_module.csv"};
  out << "t,angle,velocity" << std::endl;

  mod.SetIdle();

  for (units::second_t t = 0_s; t < 2_s; t += 20_ms) {
    if (t > 20_ms)
      mod.SetPID(45_deg, 8_ft / 1_s);
    
    mod.OnUpdate(20_ms);
    turnSim.SetInputVoltage(turnMotor.GetVoltage());
    turnSim.Update(20_ms);
    driveSim.SetInputVoltage(driveMotor.GetVoltage());
    driveSim.Update(20_ms);

    turnEncoder.SetTurns(turnSim.GetAngularPosition());
    driveEncoder.SetTurnVelocity(driveSim.GetAngularVelocity(), 20_ms);

    out << t.value() << "," << turnSim.GetAngularPosition().convert<units::degree>().value() << "," 
        << driveSim.GetAngularVelocity().value() * config.wheelRadius.value() << std::endl;
  }
}