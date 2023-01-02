#include <gtest/gtest.h>

#include "sim/WASPSim.h"

class WASPSimTest : public ::testing::Test {
 public:
  WASPSim sim{
    wom::DCMotor::Falcon500(1).WithReduction(1 / (14.0 * 14 / 32 / 36)), 4_in / 2, 0.8_m,
    wom::DCMotor::NEO(1).WithReduction(1 / (14.0 * 14 / 32 / 36)), 4_in / 2, 0.9_m,
    units::kilogram_square_meter_t{5}, 54_kg
  };
};

TEST_F(WASPSimTest, Forward) {
  sim.Update(12_V, 12_V, 0_V, 20_ms);
  EXPECT_GT(sim.vx.value(), 0);
  EXPECT_NEAR(sim.vy.value(), 0, 1e-6);
  EXPECT_NEAR(sim.omega.value(), 0, 1e-6);
}

TEST_F(WASPSimTest, RearPivot) {
  sim.Update(0_V, 0_V, 12_V, 20_ms);
  EXPECT_GT(sim.omega.value(), 0);
  EXPECT_NEAR(sim.vx.value(), 0, 1e-6);
  EXPECT_LT(sim.vy.value(), 0);
}

TEST_F(WASPSimTest, Left) {
  sim.Update(12_V, 0_V, 0_V, 20_ms);
  EXPECT_GT(sim.vx.value(), 0);
  EXPECT_LT(sim.vy.value(), 0);
  EXPECT_LT(sim.omega.value(), 0);
}

TEST_F(WASPSimTest, Right) {
  sim.Update(0_V, 12_V, 0_V, 20_ms);
  EXPECT_GT(sim.vx.value(), 0);
  EXPECT_GT(sim.vy.value(), 0);
  EXPECT_GT(sim.omega.value(), 0);
}