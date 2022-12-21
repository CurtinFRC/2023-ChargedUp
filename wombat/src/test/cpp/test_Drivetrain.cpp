#include <gtest/gtest.h>

#include "drivetrain/Drivetrain.h"

#include "FakeVoltageController.h"
#include "FakeEncoder.h"
#include "FakeGyro.h"

#include <frc/simulation/DifferentialDrivetrainSim.h>

#include <fstream>

using namespace wom;
using namespace behaviour;

class DrivetrainTest : public ::testing::Test {
 public:
  FakeVoltageController leftMotor, rightMotor;
  FakeEncoder leftEncoder{1024}, rightEncoder{1024};
  FakeGyro gyro;

  Gearbox left{ &leftMotor, &leftEncoder, DCMotor::NEO(3).WithReduction(12.75) };
  Gearbox right{ &rightMotor, &rightEncoder, DCMotor::NEO(3).WithReduction(12.75) };

  DrivetrainConfig config{
    left, right, &gyro,
    4_in / 2,
    0.5_m,
    {
      12_V / 2_mps,
    },
    {
      4_mps / 1_m,
      0_mps / 1_s / 1_m,
      0_m / 1_m,
      5_cm,
      0.2_mps
    },
    {
      (180_deg / 0.5_s) / 45_deg,
      0_deg / 1_s / 1_s / 1_deg,
      0_deg / 1_deg,
      2_deg,
      1_deg / 1_s
    }
  };

  Drivetrain dt{config};

  frc::sim::DifferentialDrivetrainSim sim{
    left.motor.ToWPI(), 1.0, units::kilogram_square_meter_t{5}, 50_kg, config.wheelRadius, config.trackWidth
  };
};

TEST_F(DrivetrainTest, Forward) {
  std::ofstream out{"drivetrain_simple.csv"};
  out << "t,x,y,heading,vl,vr,vf" << std::endl;

  for (units::second_t t = 0_s; t < 2_s; t += 20_ms) {
    if (t > 20_ms)
      dt.SetVelocity(frc::ChassisSpeeds { 1.5_mps, 0_mps, 180_deg_per_s });
    dt.OnUpdate(20_ms);
    sim.SetInputs(leftMotor.GetVoltage(), rightMotor.GetVoltage());
    sim.Update(20_ms);

    leftEncoder.SetTurnVelocity(units::radians_per_second_t{(sim.GetLeftVelocity() / config.wheelRadius).value()}, 20_ms);
    rightEncoder.SetTurnVelocity(units::radians_per_second_t{(sim.GetRightVelocity() / config.wheelRadius).value()}, 20_ms);

    out << t.value() << "," << sim.GetPose().X().value() << "," << sim.GetPose().Y().value() << ","
        << sim.GetHeading().Degrees().value() << "," << sim.GetLeftVelocity().value() << "," << sim.GetRightVelocity().value() << ","
        << (sim.GetLeftVelocity() + sim.GetRightVelocity()).value() / 2 << std::endl;
  }
}

TEST_F(DrivetrainTest, Forward1Meter) {
  std::ofstream out{"drivetrain_fwd1m.csv"};
  out << "t,x,y,heading,vl,vr,vf" << std::endl;

  auto bhvr = make<DrivetrainDriveDistance>(&dt, 1_m);

  for (units::second_t t = 0_s; t < 2_s; t += 20_ms) {
    dt.OnUpdate(20_ms);
    sim.SetInputs(leftMotor.GetVoltage(), rightMotor.GetVoltage());
    sim.Update(20_ms);

    bhvr->Tick();

    leftEncoder.SetTurnVelocity(units::radians_per_second_t{(sim.GetLeftVelocity() / config.wheelRadius).value()}, 20_ms);
    rightEncoder.SetTurnVelocity(units::radians_per_second_t{(sim.GetRightVelocity() / config.wheelRadius).value()}, 20_ms);

    out << t.value() << "," << sim.GetPose().X().value() << "," << sim.GetPose().Y().value() << ","
        << sim.GetHeading().Degrees().value() << "," << sim.GetLeftVelocity().value() << "," << sim.GetRightVelocity().value() << ","
        << (sim.GetLeftVelocity() + sim.GetRightVelocity()).value() / 2 << std::endl;
  }
}

TEST_F(DrivetrainTest, Turn90Degrees) {
  std::ofstream out{"drivetrain_t90deg.csv"};
  out << "t,x,t,heading,vl, vr, vf" << std::endl;

  auto bhvr = make<DrivetrainTurnToAngle>(&dt, 90_deg);

  for (units::second_t t = 0_s; t < 2_s; t += 20_ms) {
    dt.OnUpdate(20_ms);
    sim.SetInputs(leftMotor.GetVoltage(), rightMotor.GetVoltage());
    sim.Update(20_ms);

    bhvr->Tick();

    leftEncoder.SetTurnVelocity(units::radians_per_second_t{(sim.GetLeftVelocity() / config.wheelRadius).value()}, 20_ms);
    rightEncoder.SetTurnVelocity(units::radians_per_second_t{(sim.GetRightVelocity() / config.wheelRadius).value()}, 20_ms);
    gyro.SetAngle(sim.GetHeading().Degrees());

    out << t.value() << "," << sim.GetPose().X().value() << "," << sim.GetPose().Y().value() << ","
        << sim.GetHeading().Degrees().value() << "," << sim.GetLeftVelocity().value() << "," << sim.GetRightVelocity().value() << ","
        << (sim.GetLeftVelocity() + sim.GetRightVelocity()).value() / 2 << std::endl;
  }
}

TEST_F(DrivetrainTest, Complex) {
  std::ofstream out{"drivetrain_complex.csv"};
  out << "t,x,y,heading,vl,vr,vf" << std::endl;

  auto bhvr = make<DrivetrainDriveDistance>(&dt, 1_m, 2_m)
              << make<DrivetrainTurnToAngle>(&dt, 45_deg)
              << make<DrivetrainDriveDistance>(&dt, 0.5_m)
              << make<DrivetrainTurnToAngle>(&dt, -100_deg)
              << make<DrivetrainDriveDistance>(&dt, 2_m);

  for (units::second_t t = 0_s; t < 20_s; t += 20_ms) {
    dt.OnUpdate(20_ms);
    sim.SetInputs(leftMotor.GetVoltage(), rightMotor.GetVoltage());
    sim.Update(20_ms);

    bhvr->Tick();

    leftEncoder.SetTurnVelocity(units::radians_per_second_t{(sim.GetLeftVelocity() / config.wheelRadius).value()}, 20_ms);
    rightEncoder.SetTurnVelocity(units::radians_per_second_t{(sim.GetRightVelocity() / config.wheelRadius).value()}, 20_ms);
    gyro.SetAngle(sim.GetHeading().Degrees());

    out << t.value() << "," << sim.GetPose().X().value() << "," << sim.GetPose().Y().value() << ","
        << sim.GetHeading().Degrees().value() << "," << sim.GetLeftVelocity().value() << "," << sim.GetRightVelocity().value() << ","
        << (sim.GetLeftVelocity() + sim.GetRightVelocity()).value() / 2 << std::endl;
  }
}