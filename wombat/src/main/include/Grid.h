#pragma once

#include <Eigen/Core>

#include <units/base.h>
#include <units/math.h>
#include <queue>
#include <vector>
#include <memory>
#include <algorithm>

namespace wom {
  template<typename CostT>
  struct AStarNode {
    Eigen::Vector2i position;
    std::shared_ptr<AStarNode> parent;
    units::unit_t<CostT> gScore;
    units::unit_t<CostT> fScore;
  };

  template<typename T_X, typename T_Y>
  class DiscretisedOccupancyGrid {
   public:
    using X_t = units::unit_t<T_X>;
    using Y_t = units::unit_t<T_Y>;

    using Idx_t = Eigen::Vector2i;
    struct ContinuousIdxT {
      X_t x;
      Y_t y;
    };

    DiscretisedOccupancyGrid(X_t xsize, Y_t ysize, size_t ux, size_t uy)
      : _xsize(xsize), _ysize(ysize)
    {
      _grid.resize(uy, ux);
      Reset();
    }

    void Reset() {
      _grid.fill(0);
    }

    void Fill(bool value) {
      _grid.fill(value ? 1 : 0);
    }

    void Load(const Eigen::MatrixXi &matrix) {
      if (matrix.cols() != _grid.cols() || matrix.rows() != _grid.rows()) {
        throw std::invalid_argument("Rows / Cols Mismatch!");
      } else {
        _grid = matrix;
      }
    }

    bool Get(Idx_t idx) {
      if (idx.y() < 0 || idx.x() < 0)
        return true;
      if (idx.y() >= _grid.rows() || idx.x() >= _grid.cols())
        return true;
      
      return _grid(idx.y(), idx.x());
    }

    void Set(Idx_t idx, bool occupied) {
      _grid(idx.y(), idx.x()) = occupied;
    }

    Idx_t Discretise(ContinuousIdxT i) {
      auto x_per_grid = _xsize / (float)_grid.cols();
      auto y_per_grid = _ysize / (float)_grid.rows();

      return Eigen::Vector2i{
        (int)(i.x / x_per_grid),
        (int)(i.y / y_per_grid),
      };
    }

    /* SEARCH */

    template<typename From, typename To>
    using converting_unit = typename units::unit_t<units::compound_unit<To, units::inverse<From>>>;

    template<typename CostT>
    std::deque<Idx_t> AStar(Idx_t start, Idx_t end, converting_unit<T_X, CostT> dxCost, converting_unit<T_Y, CostT> dyCost) {
      using cost_t = units::unit_t<CostT>;
      using node_t = std::shared_ptr<AStarNode<CostT>>;

      std::vector<node_t> allNodes;
      allNodes.push_back(std::make_shared<AStarNode<CostT>>(
        start, nullptr, cost_t{0}, Cost<CostT>(start, end, dxCost, dyCost)
      ));

      std::vector<node_t> openSet;
      openSet.push_back(allNodes[0]);

      while (!openSet.empty()) {
        auto currentIt = std::min_element(openSet.cbegin(), openSet.cend(), [](auto a, auto b) { return a->fScore < b->fScore; });
        node_t current = *currentIt;
        openSet.erase(currentIt);
        if (current->position == end) {
          std::deque<Idx_t> queue;
          queue.push_front(current->position);

          while (current->parent) {
            queue.push_front(current->parent->position);
            current = current->parent;
          }

          return queue;
        } else {
          for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
              Idx_t newPos = current->position + Idx_t{ dx, dy };
              // Find neighbour, or create if not exists.
              auto neighbourIt = std::find_if(allNodes.cbegin(), allNodes.cend(), [newPos](auto node) { return node->position == newPos; });
              if (neighbourIt == allNodes.cend()) {
                allNodes.push_back(std::make_shared<AStarNode<CostT>>(
                  newPos, nullptr, cost_t{1e9}, cost_t{1e9}
                ));
                neighbourIt = allNodes.cend() - 1;
              }
              node_t neighbour = *neighbourIt;

              if (newPos != current->position && !Get(newPos)) {
                cost_t tentative = current->gScore + Cost<CostT>(current->position, newPos, dxCost, dyCost);

                if (tentative < neighbour->gScore) {
                  neighbour->parent = current;
                  neighbour->gScore = tentative;
                  neighbour->fScore = tentative + Cost<CostT>(newPos, end, dxCost, dyCost);
                  if (std::find(openSet.cbegin(), openSet.cend(), neighbour) == openSet.cend()) {
                    openSet.push_back(neighbour);
                  }
                }
              }
            }
          }
        }
      }

      return std::deque<Idx_t>{};
    }

   protected:
    template<typename CostT>
    units::unit_t<CostT> Cost(Idx_t start, Idx_t end, converting_unit<T_X, CostT> dxCost, converting_unit<T_Y, CostT> dyCost) {
      auto x_per_grid = _xsize / (float)_grid.cols();
      auto y_per_grid = _ysize / (float)_grid.rows();

      Idx_t rel = end - start;
      auto xcost = rel.x() * x_per_grid * dxCost;
      auto ycost = rel.y() * y_per_grid * dyCost;

      return units::math::sqrt(xcost * xcost + ycost * ycost);
    }

   private:
    X_t _xsize;
    Y_t _ysize;
    Eigen::MatrixXi _grid;
  };
}