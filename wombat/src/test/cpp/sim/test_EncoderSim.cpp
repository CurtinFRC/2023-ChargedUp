#include "gtest/gtest.h"

#include "Encoder.h"

using namespace wom;

TEST(EncoderSim, DigitalEncoder) {
  DigitalEncoder encoder{1, 2, 2048};
  auto sim = encoder.MakeSimEncoder();
  sim->SetEncoderTurns(12_rad);
  sim->SetEncoderTurnVelocity(24_rad / 1_s);

  EXPECT_NEAR(encoder.GetEncoderPosition().value(), 12, 0.01);
  EXPECT_NEAR(encoder.GetEncoderAngularVelocity().value(), 24, 0.01);

  encoder.SetEncoderPosition(4_rad);

  EXPECT_NEAR(encoder.GetEncoderPosition().value(), 4, 0.01);
  EXPECT_NEAR(encoder.GetEncoderAngularVelocity().value(), 24, 0.01);
}

TEST(EncoderSim, SparkMax) {
  rev::CANSparkMax sparkMax{99, rev::CANSparkMax::MotorType::kBrushless};
  CANSparkMaxEncoder encoder{&sparkMax};
  auto sim = encoder.MakeSimEncoder();
  sim->SetEncoderTurns(12_rad);
  sim->SetEncoderTurnVelocity(24_rad / 1_s);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_NEAR(encoder.GetEncoderPosition().value(), 12, 0.01);
  EXPECT_NEAR(encoder.GetEncoderAngularVelocity().value(), 24, 0.01);
}

// TEST(EncoderSim, TalonFX) {
//   WPI_TalonFX talonFX{99};
//   TalonFXEncoder encoder{&talonFX};
//   auto sim = encoder.MakeSimEncoder();
//   sim->SetEncoderTurns(12_rad);
//   sim->SetEncoderTurnVelocity(24_rad / 1_s);

//   std::this_thread::sleep_for(std::chrono::milliseconds(100));

//   EXPECT_NEAR(encoder.GetEncoderPosition().value(), 12, 0.01);
//   EXPECT_NEAR(encoder.GetEncoderAngularVelocity().value(), 24, 0.01);
// }