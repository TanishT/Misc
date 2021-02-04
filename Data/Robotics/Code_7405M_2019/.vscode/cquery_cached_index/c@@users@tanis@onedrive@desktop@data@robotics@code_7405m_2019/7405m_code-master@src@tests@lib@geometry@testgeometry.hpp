//
// Created by alexweiss on 7/3/19.
//

#ifndef INC_7405M_CODE_TESTGEOMETRY_HPP
#define INC_7405M_CODE_TESTGEOMETRY_HPP

#include "../../testBase.hpp"

namespace test {
    class testGeometry {
      public:
        static void testRotation2d();
        static void testTranslation2d();
        static void testPose2d();
        static void testTwist();
    };
}

#endif //INC_7405M_CODE_TESTGEOMETRY_HPP
