//
// Created by alexweiss on 7/4/19.
//

#ifndef INC_7405M_CODE_DRIVECHARACTERIZATION_HPP
#define INC_7405M_CODE_DRIVECHARACTERIZATION_HPP

#include "../utility/PolynomialRegression.hpp"
#include <vector>

namespace physics {
    class DriveCharacterization {
    public:
        class CharacterizationConstants {
        public:
            double ks; //voltage needed to break static friction
            double kv;
            double ka;
        };

        class VelocityDataPoint {
        public:
            const double velocity_;
            const double power_;
            VelocityDataPoint(double velocity, double power) : velocity_(velocity), power_(power) {};
        };

        class AccelerationDataPoint {
        public:
            const double velocity_;
            const double power_;
            const double acceleration_;
            AccelerationDataPoint(double velocity, double power, double acceleration) :
              velocity_(velocity), power_(power), acceleration_(acceleration) {};
        };

        class CurvatureDataPoint {
        public:
            const double linear_velocity_;
            const double angular_velocity_;
            const double left_voltage_;
            const double right_voltage_;
            CurvatureDataPoint(double linear_velocity, double angular_velocity, double left_voltage, double right_voltage) :
              linear_velocity_(linear_velocity), angular_velocity_(angular_velocity), left_voltage_(left_voltage), right_voltage_(right_voltage) {};
        };

        static CharacterizationConstants characterizeDrive(std::vector<VelocityDataPoint> velocityData, std::vector<AccelerationDataPoint> accelerationData);
        static CharacterizationConstants getVelocityCharacterization(std::vector<util::Point> points);
        static void getAccelerationCharacterization(std::vector<util::Point> points, CharacterizationConstants * velocityCharacterization);

        // Cleans Data
        static std::vector<util::Point> getVelocityData(std::vector<VelocityDataPoint> input);

        static std::vector<util::Point> getAccelerationData(std::vector<AccelerationDataPoint> input, CharacterizationConstants constants);


    };
}


#endif //INC_7405M_CODE_DRIVECHARACTERIZATION_HPP
