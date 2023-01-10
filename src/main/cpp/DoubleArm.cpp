#include "DoubleArm.h"

// Your code here

/* SIMULATION */ 
::sim::DoubleArmSim::DoubleArmSim(DoubleArmConfig config)
  : arm1sim(config.arm1), arm2sim(config.arm2) {}

void ::sim::DoubleArmSim::Update(units::second_t dt) {
  arm1sim.config.loadMass = arm2sim.config.armMass + arm2sim.config.loadMass;
  arm1sim.additionalTorque = arm2sim.torque; 
  arm1sim.Update(dt);

  arm2sim.config.angleOffset = -180_deg + arm1sim.angle;
  arm2sim.Update(dt);
}

units::ampere_t sim::DoubleArmSim::GetCurrent() const {
  return arm1sim.GetCurrent() + arm2sim.GetCurrent();
}