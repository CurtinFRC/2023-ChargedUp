#include <gtest/gtest.h>

#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>

#include "Grid.h"

#include <iostream>

// TEST(Grid, AStar) {
//   Eigen::MatrixXi matrix{
//     { 1, 1, 1, 1, 1, 1, 1 },
//     { 1, 0, 1, 1, 0, 0, 0 },
//     { 1, 0, 1, 1, 0, 1, 0 },
//     { 1, 0, 1, 0, 0, 0, 0 },
//     { 1, 0, 0, 0, 1, 0, 1 },
//     { 1, 1, 1, 0, 0, 0, 1 },
//     { 1, 1, 1, 1, 1, 1, 1 },
//   };

//   wom::DiscretisedOccupancyGrid<units::radian, units::meter> grid{ 180_deg, 1_m};
//   grid.Load(matrix);

//   auto q = grid.AStar<units::second>(
//     { 1, 1 }, { 5, 1 },
//     1_s / 180_deg, 1_s / 1_m
//   );

//   ASSERT_EQ(q.size(), 7);
//   ASSERT_EQ(q.front(), (Eigen::Vector2i{1, 1})); q.pop_front();
//   ASSERT_EQ(q.front(), (Eigen::Vector2i{1, 2})); q.pop_front();
//   ASSERT_EQ(q.front(), (Eigen::Vector2i{1, 3})); q.pop_front();
//   ASSERT_EQ(q.front(), (Eigen::Vector2i{2, 4})); q.pop_front();
//   ASSERT_EQ(q.front(), (Eigen::Vector2i{3, 3})); q.pop_front();
//   ASSERT_EQ(q.front(), (Eigen::Vector2i{4, 2})); q.pop_front();
//   ASSERT_EQ(q.front(), (Eigen::Vector2i{5, 1}));
// }