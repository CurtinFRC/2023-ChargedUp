/*
    Take in a vector of structs (that have both x and y)
    Take in an x value
        draw lines starting and ending at the closest point to the right and the closest point to the left
        find the intereception point and return that y value
*/


#pragma once

#include <vector>

namespace wom {
  template<typename X, typename Y>
  struct LUTPoint{
    X x = 0;
    Y y = 0;  
  };

  template<typename X, typename Y>
  class LUT{
  public:
    LUT(std::vector<LUTPoint<X, Y>> points) : _points(points) {}


    Y Estimate(X x){
      int tableSize = _points.size();

      if (tableSize == 0){  return Y{0};  }  // if no recorded data points
      if (tableSize == 1) {  return _points[0].y;  }  // if only 1 recorded data point
      
      if (x < _points[0].x){  return _points[0].y;  }  // if x < allOthers
      
      LUTPoint<X, Y> finalPoint = _points.back();
      if (x > finalPoint.x){  return finalPoint.y;  }  // if x > allOthers

      LUTPoint<X, Y> previousThing = _points[0];
      for (int pointNum = 1; pointNum < tableSize; pointNum++){
        previousThing = _points[pointNum - 1];
        //if (_points[pointNum].x == x){  return _points[pointNum].y;  }

        if (_points[pointNum].x >= x){
          double m = (_points[pointNum].y - previousThing.y) / (_points[pointNum].x - previousThing.x);
          return m * (x - previousThing.x) + previousThing.y; // y = mx+c
        }
      }
    };

  private:
    std::vector<LUTPoint<X, Y>> _points;
  };
}
