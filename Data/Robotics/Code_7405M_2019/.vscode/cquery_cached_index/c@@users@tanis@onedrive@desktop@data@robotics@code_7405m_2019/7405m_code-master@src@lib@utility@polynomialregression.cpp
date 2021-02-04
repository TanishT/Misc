//
// Created by alexweiss on 7/7/19.
//


#include "PolynomialRegression.hpp"
#include <stdlib.h>
#include <cmath>
#include <stdio.h>

namespace util {
  PolynomialRegression::PolynomialRegression(std::vector<Point> points, int degree) : points_ (points), degree_(degree) {
    solve();
  };
  void PolynomialRegression::solve() { // https://gist.github.com/chrisengelsma/108f7ab0a746323beaaf7d6634cf4add
    std::vector<double> x;
    std::vector<double> y;
    for(Point i : points_) {
      x.push_back(i.x_);
      y.push_back(i.y_);
    }

    size_t N = x.size();
    int n = degree_;
    int np1 = n + 1;
    int np2 = n + 2;
    int tnp1 = 2 * n + 1;
    double tmp;

    // X = vector that stores values of sigma(xi^2n)
    std::vector<double> X(tnp1);
    for (int i = 0; i < tnp1; ++i) {
      X[i] = 0;
      for (int j = 0; j < N; ++j)
        X[i] += (double) pow(x.at(j), i);
    }

    // a = vector to store final coefficients.
    std::vector<double> a(np1);

    // B = normal augmented matrix that stores the equations.
    std::vector<std::vector<double> > B(np1, std::vector<double> (np2, 0));

    for (int i = 0; i <= n; ++i)
      for (int j = 0; j <= n; ++j)
        B[i][j] = X[i + j];

    // Y = vector to store values of sigma(xi^n * yi)
    std::vector<double> Y(np1);
    for (int i = 0; i < np1; ++i) {
      Y[i] = (double)0;
      for (int j = 0; j < N; ++j) {
        Y[i] += (double)pow(x[j], i)*y[j];
      }
    }

    // Load values of Y as last column of B
    for (int i = 0; i <= n; ++i)
      B[i][np1] = Y[i];

    n += 1;
    int nm1 = n-1;

    // Pivotisation of the B matrix.
    for (int i = 0; i < n; ++i)
      for (int k = i+1; k < n; ++k)
        if (B[i][i] < B[k][i])
          for (int j = 0; j <= n; ++j) {
            tmp = B[i][j];
            B[i][j] = B[k][j];
            B[k][j] = tmp;
          }

    // Performs the Gaussian elimination.
    // (1) Make all elements below the pivot equals to zero
    //     or eliminate the variable.
    for (int i=0; i<nm1; ++i)
      for (int k =i+1; k<n; ++k) {
        double t = B[k][i] / B[i][i];
        for (int j=0; j<=n; ++j)
          B[k][j] -= t*B[i][j];         // (1)
      }

    // Back substitution.
    // (1) Set the variable as the rhs of last equation
    // (2) Subtract all lhs values except the target coefficient.
    // (3) Divide rhs by coefficient of variable being calculated.
    for (int i=nm1; i >= 0; --i) {
      a[i] = B[i][n];                   // (1)
      for (int j = 0; j<n; ++j)
        if (j != i)
          a[i] -= B[i][j] * a[j];       // (2)
      a[i] /= B[i][i];                  // (3)
    }

    for (size_t i = 0; i < a.size(); ++i)
      coeffs_.insert(coeffs_.begin()+i, a[i]);
  }
  double PolynomialRegression::R2() {
    return 0.0;
  }
  double PolynomialRegression::beta(int i) {
    if(coeffs_.size() - 1 < i) {
      return 0.0;
    }
    return coeffs_[i];
  }
  int PolynomialRegression::degree() {
    return degree_;
  }
  double PolynomialRegression::predict(double x) {
    double y = 0;
    for(int i = 0; i <= degree_; i++) {
      y += beta(i) * pow(x, i);
    }
    return y;
  }


}