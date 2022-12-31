#include <gtest/gtest.h>

#include "drivetrain/SwerveDrive.h"

#include "FakeVoltageController.h"
#include "FakeEncoder.h"
#include "FakeGyro.h"
#include "SwerveSim.h"

#include <frc/simulation/DCMotorSim.h>

#include <fstream>

using namespace wom;

struct SwerveModuleTestVars {
  SwerveModuleTestVars(frc::Translation2d pos) : config{ pos, drive, turn, 4_in / 2 }, mod{ "mod", config, anglePID, velocityPID } {}

  FakeVoltageController driveMotor, turnMotor;
  FakeEncoder driveEncoder{1024}, turnEncoder{1024};

  Gearbox drive{ &driveMotor, &driveEncoder, DCMotor::NEO(1).WithReduction(8.14) };
  Gearbox turn{ &turnMotor, &turnEncoder, DCMotor::NEO(1).WithReduction(150 / 7.0) };

  SwerveModuleConfig config;

  SwerveModule::angle_pid_conf_t anglePID{
    "/mod/pid/angle/config",
    12_V / 90_deg,
  };

  SwerveModule::velocity_pid_conf_t velocityPID{
    "/mod/pid/velocity/config",
    12_V / 1_mps
  };

  SwerveModule mod;
  SwerveModuleSim sim{ turn.motor, drive.motor, 54_kg / 4, 0.5 * 6_lb * 7.5_in * 7.5_in, 4_in / 2 };

  void update(units::second_t dt) {
    turnEncoder.SetTurns(sim.GetAngle());
    driveEncoder.SetTurnVelocity(sim.GetSpeed(), dt);
  }
};

class SwerveModuleTest : public ::testing::Test {
 public:
  SwerveModuleTestVars vars{ frc::Translation2d{0_m, 0_m} };
};

TEST_F(SwerveModuleTest, Simple) {
  std::ofstream out{"swerve_module.csv"};
  out << "t,angle,velocity" << std::endl;

  vars.mod.SetIdle();

  for (units::second_t t = 0_s; t < 2_s; t += 20_ms) {
    if (t > 20_ms)
      vars.mod.SetPID(45_deg, 8_ft / 1_s);
    
    vars.mod.OnUpdate(20_ms);
    vars.sim.Calculate(vars.turnMotor.GetVoltage(), vars.driveMotor.GetVoltage(), 20_ms);

    vars.update(20_ms);

    out << t.value() << "," << vars.sim.GetAngle().convert<units::degree>().value() << "," 
        << vars.sim.GetVelocity().value() << std::endl;
  }
}

class SwerveTest : public ::testing::Test {
 public:
  wpi::array<SwerveModuleTestVars *, 4> modules{ 
    new SwerveModuleTestVars(frc::Translation2d{1_m, 1_m} ),
    new SwerveModuleTestVars(frc::Translation2d{1_m, -1_m} ),
    new SwerveModuleTestVars(frc::Translation2d{-1_m, -1_m} ),
    new SwerveModuleTestVars(frc::Translation2d{-1_m, 1_m} ),
  };

  FakeGyro gyro;

  SwerveDriveConfig cfg{
    modules[0]->anglePID, modules[1]->velocityPID,
    { modules[0]->config, modules[1]->config, modules[2]->config, modules[3]->config },
    &gyro,
    {
      "swerve/pid/heading/config",
      (180_deg / 1_s) / 45_deg
    },
    {
      "swerve/pid/position/config",
      4_mps / 1_m
    },
    { 0.1, 0.1, 0.1 },
    { 0.05 },
    { 0.0, 0.0, 0.0 }
  };

  SwerveDrive swerve{"swerve", cfg, frc::Pose2d{ frc::Translation2d{0_m, 0_m}, frc::Rotation2d{ 0_deg } }};

  SwerveSim sim{
    frc::SwerveDriveKinematics(modules[0]->config.position, modules[1]->config.position, modules[2]->config.position, modules[3]->config.position),
    { &modules[0]->sim, &modules[1]->sim, &modules[2]->sim, &modules[3]->sim }
  };
};

TEST_F(SwerveTest, Simple) {
  std::ofstream out{"swerve.csv"};
  out << "t,x,y,heading,t1,t2,t3,t4,v0,v1,v2,v3,xe,ye,he" << std::endl;

  swerve.SetIdle();

  for (units::second_t t = 0_s; t < 2_s; t += 20_ms) {
    if (t > 20_ms)
      // swerve.SetFieldRelativeVelocity(FieldRelativeSpeeds {
      //   1_mps, 1_mps, 45_deg / 1_s
      // });
      swerve.SetPose(frc::Pose2d {
        frc::Translation2d{1_m, 1.5_m},
        frc::Rotation2d{45_deg}
      });
    
    swerve.OnUpdate(20_ms);
    sim.Calculate({
      modules[0]->turnMotor.GetVoltage(),
      modules[1]->turnMotor.GetVoltage(),
      modules[2]->turnMotor.GetVoltage(),
      modules[3]->turnMotor.GetVoltage(),
    }, {
      modules[0]->driveMotor.GetVoltage(),
      modules[1]->driveMotor.GetVoltage(),
      modules[2]->driveMotor.GetVoltage(),
      modules[3]->driveMotor.GetVoltage(),
    }, 20_ms);

    // vars.turnEncoder.SetTurns(sim.GetAngle());
    // vars.driveEncoder.SetTurnVelocity(sim.GetSpeed(), 20_ms);
    gyro.SetAngle(sim.theta);
    modules[0]->update(20_ms);
    modules[1]->update(20_ms);
    modules[2]->update(20_ms);
    modules[3]->update(20_ms);

    out << t.value() << "," << sim.x.value() << "," 
        << sim.y.value() << "," << sim.theta.convert<units::degree>().value() << ","
        << sim.modules[0]->GetAngle().convert<units::degree>().value() << ","
        << sim.modules[1]->GetAngle().convert<units::degree>().value() << ","
        << sim.modules[2]->GetAngle().convert<units::degree>().value() << ","
        << sim.modules[3]->GetAngle().convert<units::degree>().value() << ","
        << sim.modules[0]->GetSpeed().value() << ","
        << sim.modules[1]->GetSpeed().value() << ","
        << sim.modules[2]->GetSpeed().value() << ","
        << sim.modules[3]->GetSpeed().value() << ","
        << swerve.GetPose().X().value() << ","
        << swerve.GetPose().Y().value() << ","
        << swerve.GetPose().Rotation().Degrees().value() << ","
        << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}