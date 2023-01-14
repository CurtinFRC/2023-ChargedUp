#pragma once

#include <Eigen/Core>

#include <units/base.h>
#include <units/math.h>
#include <queue>
#include <vector>
#include <memory>
#include <algorithm>

namespace wom {
  template<typename I, typename O>
  O remap(I x, I in_min, I in_max, O out_min, O out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

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

    template<typename CostT>
    struct GridPathNode {
      ContinuousIdxT position;
      units::unit_t<CostT> cost;
    };

    DiscretisedOccupancyGrid(X_t xmin, X_t xmax, Y_t ymin, Y_t ymax, size_t ux, size_t uy)
      : _xmin(xmin), _xmax(xmax), _ymin(ymin), _ymax(ymax)
    {
      _grid.resize(uy, ux);
      Reset();
    }

    DiscretisedOccupancyGrid(X_t xmin, X_t xmax, Y_t ymin, Y_t ymax, Eigen::MatrixXi matrix)
      : _xmin(xmin), _xmax(xmax), _ymin(ymin), _ymax(ymax), _grid(matrix) { }

    void Reset() {
      _grid.fill(0);
    }

    void Fill(bool value) {
      _grid.fill(value ? 1 : 0);
    }

    DiscretisedOccupancyGrid FillF(std::function<bool(X_t, Y_t)> f) {
      for (int x = 0; x < _grid.cols(); x++) {
        for (int y = 0; y < _grid.rows(); y++) {
          ContinuousIdxT ci = CenterOf(Eigen::Vector2i{x, y});
          _grid(y, x) = f(ci.x, ci.y);
        }
      }
      return *this;
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
      return Eigen::Vector2i{
        (int)remap(i.x, _xmin, _xmax, 0.0, (double)_grid.cols()),
        (int)remap(i.y, _ymin, _ymax, 0.0, (double)_grid.rows())
      };
    }

    ContinuousIdxT CenterOf(Idx_t idx) {
      return ContinuousIdxT {
        (remap((double)idx.x(), 0.0, (double)_grid.cols(), _xmin, _xmax) + remap((double)idx.x() + 1, 0.0, (double)_grid.cols(), _xmin, _xmax)) / 2.0,
        (remap((double)idx.y(), 0.0, (double)_grid.rows(), _ymin, _ymax) + remap((double)idx.y() + 1, 0.0, (double)_grid.rows(), _ymin, _ymax)) / 2.0
      };
    }

    /* SEARCH */

    template<typename From, typename To>
    using converting_unit = typename units::unit_t<units::compound_unit<To, units::inverse<From>>>;

    template<typename CostT>
    std::deque<GridPathNode<CostT>> AStar(Idx_t start, Idx_t end, converting_unit<T_X, CostT> dxCost, converting_unit<T_Y, CostT> dyCost) {
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
          std::deque<GridPathNode<CostT>> queue;
          queue.push_front(GridPathNode<CostT> {
            CenterOf(current->position),
            current->gScore
          });

          while (current->parent) {
            queue.push_front(GridPathNode<CostT> {
              CenterOf(current->parent->position),
              current->gScore
            });
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

      return std::deque<GridPathNode<CostT>>{};
    }

    template<typename CostT>
    units::unit_t<CostT> Cost(Idx_t start, Idx_t end, converting_unit<T_X, CostT> dxCost, converting_unit<T_Y, CostT> dyCost) {
      auto x_per_grid = (_xmax - _xmin) / (float)_grid.cols();
      auto y_per_grid = (_ymax - _ymin) / (float)_grid.rows();

      Idx_t rel = end - start;
      auto xcost = rel.x() * x_per_grid * dxCost;
      auto ycost = rel.y() * y_per_grid * dyCost;

      return units::math::sqrt(xcost * xcost + ycost * ycost);
    }

    X_t _xmin, _xmax;
    Y_t _ymin, _ymax;
    Eigen::MatrixXi _grid;
  };
}