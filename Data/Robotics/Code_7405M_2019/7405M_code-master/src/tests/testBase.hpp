//
// Created by alexweiss on 7/3/19.
//

#ifndef INC_7405M_CODE_TESTBASE_HPP
#define INC_7405M_CODE_TESTBASE_HPP

#include "../lib/utility/Utility.hpp"
#include "../lib/geometry/Translation2d.hpp"
#include "../lib/utility/Units.hpp"

namespace test {
    template <typename T> void assertEquals(T val, T compare) {
      if(val == compare) {
        printf("PASS\n");
      }
      else {
        printf("FAILURE\n");
      }
    }
    void assertEquals(double val, double compare, double eps);

    template <typename A, typename B, typename C, typename D>
    void assertEquals(double val, units::RQuantity<A, B, C, D> compare, double eps) {
      assertEquals(val, compare.getValue(), eps);
    }

    template <typename A, typename B, typename C, typename D>
    void assertEquals(double val, units::RQuantity<A, B, C, D> compare) {
      assertEquals(val, compare.getValue(), 0.0001);
    }

    template <typename A, typename B, typename C, typename D>
    void assertEquals(units::RQuantity<A, B, C, D> val, double compare, double eps) {
      assertEquals(compare, val.getValue(), eps);
    }

    template <typename A, typename B, typename C, typename D>
    void assertEquals(units::RQuantity<A, B, C, D> val, double compare) {
      assertEquals(compare, val.getValue(), 0.0001);
    }

    template <typename A, typename B, typename C, typename D>
    void assertEquals(units::RQuantity<A, B, C, D> val, units::RQuantity<A, B, C, D> compare, double eps) {
      assertEquals(compare.getValue(), val.getValue(), eps);
    }

    template <typename A, typename B, typename C, typename D>
    void assertEquals(units::RQuantity<A, B, C, D> val, units::RQuantity<A, B, C, D> compare) {
      assertEquals(compare.getValue(), val.getValue(), 0.0001);
    }

    void assertTrue(bool val);
    void assertFalse(bool val);
}

#endif