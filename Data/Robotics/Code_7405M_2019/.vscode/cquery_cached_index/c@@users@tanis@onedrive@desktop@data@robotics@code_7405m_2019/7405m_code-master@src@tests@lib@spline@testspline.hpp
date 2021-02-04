//
// Created by alexweiss on 7/3/19.
//

#ifndef INC_7405M_CODE_TESTSPLINE_HPP
#define INC_7405M_CODE_TESTSPLINE_HPP

#include "../../testBase.hpp"

namespace test {
    class testSpline {
    public:
        static void testCubicSpline();
        static void testQuinticSpline();
        static void testSplineGenerator();
        static void testDCurve();
    };
}

#endif //INC_7405M_CODE_TESTSPLINE_HPP
