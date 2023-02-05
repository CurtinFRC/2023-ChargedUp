#include <gtest/gtest.h>

#include "LUT.h"

#include <units/length.h>

using namespace wom;

TEST(LUT, NoPoints) {
    LUT<double, double> lut({});
    EXPECT_NEAR(lut.Estimate(10), 0, 0.001);
}

TEST(LUT, OnePoint) {
    LUT<units::meter_t, units::meter_t> lut({{1_m, 9_m}});
    EXPECT_NEAR(lut.Estimate(3_m).value(), 9, 0.001);
}

TEST(LUT, BeforeFirstPoint) {
    LUT<double, double> lut({
        {1, 9},
        {5, 12.8},
        {16, 7.1},
        {21, 3}
    });
    EXPECT_NEAR(lut.Estimate(0.3), 9, 0.001);
}

TEST(LUT, IntermediatePoint) {
    LUT<double, double> lut({
        {1, 9},
        {5, 12.8},
        {16, 7.1},
        {21, 3}
    });
    EXPECT_NEAR(lut.Estimate(8.3), 11.09, 0.001);
    
}

TEST(LUT, AfterLastPoint) {
    LUT<double, double> lut({
        {1, 9},
        {5, 12.8},
        {16, 7.1},
        {21, 3}
    });
    EXPECT_NEAR(lut.Estimate(314159265359), 3, 0.001);
    
}
TEST(LUT, AtAPoint) {
    LUT<double, double> lut({
        {1, 900},
        {5, 1200.8},
        {16, 7.1},
        {21, 300}
    });
    EXPECT_NEAR(lut.Estimate(16), 7.1, 0.001);
}