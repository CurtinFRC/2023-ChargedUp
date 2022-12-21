#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "VoltageController.h"

#include <frc/simulation/RoboRioSim.h>

class MockMotorController : public frc::MotorController {
 public:
  MOCK_METHOD1(Set, void(double));
  MOCK_CONST_METHOD0(Get, double(void));
  MOCK_METHOD1(SetInverted, void(bool));
  MOCK_CONST_METHOD0(GetInverted, bool(void));
  MOCK_METHOD0(Disable, void(void));
  MOCK_METHOD0(StopMotor, void(void));
};

TEST(MotorVoltageController, SetVoltage) {
  MockMotorController c;
  wom::MotorVoltageController vc(&c);

  frc::sim::RoboRioSim ::SetVInVoltage(12.0_V);

  EXPECT_CALL(c, Set(1.0));
  vc.SetVoltage(12_V);
}