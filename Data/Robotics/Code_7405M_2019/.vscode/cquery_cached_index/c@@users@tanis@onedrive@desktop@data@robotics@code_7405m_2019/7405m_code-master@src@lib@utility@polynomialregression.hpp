//
// Created by alexweiss on 7/7/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_UTILITY_POLYNOMIALREGRESSION_HPP_
#define INC_7405M_CODE_SRC_LIB_UTILITY_POLYNOMIALREGRESSION_HPP_

#include <vector>
#include <array>

namespace util {
  class Point {
   public:
    double x_, y_;
    Point(double x, double y) : x_(x), y_(y) {};
  };
  class PolynomialRegression {
   private:
    std::vector<Point> points_;
    int degree_;
    double sse; // sum of squares due to error
    double sst; // total sum of squares
    void solve();

   public:
    std::vector<double> coeffs_;
    PolynomialRegression(std::vector<Point> points, int degree);
    double R2();
    double beta(int i);
    int degree();
    double predict(double x);
  };
}

#endif //INC_7405M_CODE_SRC_LIB_UTILITY_POLYNOMIALREGRESSION_HPP_
