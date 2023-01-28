// #pragma once

// #include "Arm.h"

// struct DoubleArmConfig {
//   wom::ArmConfig arm1;
//   wom::ArmConfig arm2;
// };

// // Your code here

// /* SIMULATION */

// namespace sim {
//   class DoubleArmSim {
//    public:
//     DoubleArmSim(DoubleArmConfig config);

//     void Update(units::second_t dt);

//     units::ampere_t GetCurrent() const;

//     wom::sim::ArmSim arm1sim, arm2sim;
//   };
// }