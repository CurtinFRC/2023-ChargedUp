#pragma once 

#include "Encoder.h"
#include "DcMotor.h"
#include "VoltageController.h"
#include <frc/system/plant/DCMotor.h>

namespace wom {
/**
 * Struct for motor and encoder pairs.
 *
 * Combines the motor and encoder parts into one data structure,
 * so that both Spark + Encoder and Talon SRX are treated the same.
 */
struct Gearbox {

  /**
   * The VoltageController (Motor Controller). May not be null.
   */
  actuators::VoltageController *transmission;

  /**
   * The Encoder. May be null, depending on the consumer of this structure.
   */
  sensors::Encoder *encoder = nullptr;
  /**
   * The motor being used. By default, this is a dual CIM.
   */
  frc::DCMotor motor = frc::DCMotor::CIM(2);
};
} //ns wom 