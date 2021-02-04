//
// Created by alexweiss on 7/8/19.
//

#ifndef INC_7405M_CODE_SRC_TESTS_TESTPHYSICS_HPP_
#define INC_7405M_CODE_SRC_TESTS_TESTPHYSICS_HPP_

#include "../../testBase.hpp"
#include "../../../lib/physics/DCMotorTransmission.hpp"
#include "../../../lib/physics/DifferentialDrive.hpp"
#include "../../../lib/physics/DriveCharacterization.hpp"


namespace test {
  class testPhysics {
   private:
    static double rpm_to_rads_per_sec(double rpm);
    static double inches_to_meters(double inches);
    static double feet_to_meters(double feet);
    static double meters_to_feet(double m);
    static double degrees_to_radians(double deg);
   public:
    static void unitTester();
    static void testDriveCharacterization();
    static void testDCMotorTransmission();
    static void testDifferentialDrive();
  };
}

#endif //INC_7405M_CODE_SRC_TESTS_TESTPHYSICS_HPP_
