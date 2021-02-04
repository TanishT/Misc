//
// Created by alexweiss on 7/4/19.
//

#include "DriveCharacterization.hpp"
#include "../utility/PolynomialRegression.hpp"
#include "../utility/Utility.hpp"
#include <cmath>

namespace physics {

  DriveCharacterization::CharacterizationConstants DriveCharacterization::characterizeDrive(
      std::vector<DriveCharacterization::VelocityDataPoint> velocityData,
      std::vector<DriveCharacterization::AccelerationDataPoint> accelerationData) {
    CharacterizationConstants rv;
    rv = getVelocityCharacterization(getVelocityData(velocityData));
    getAccelerationCharacterization(getAccelerationData(accelerationData, rv), &rv);
    return rv;
  }

  DriveCharacterization::CharacterizationConstants DriveCharacterization::getVelocityCharacterization(
      std::vector<util::Point> points) {
    CharacterizationConstants constants;
    if (points.size() == 0) {
      return constants;
    }
    util::PolynomialRegression p(points, 1);
    constants.ks = p.beta(0);
    constants.kv = p.beta(1);
    return constants;
  }

  void DriveCharacterization::getAccelerationCharacterization(
      std::vector<util::Point> points,
      DriveCharacterization::CharacterizationConstants * velocityCharacterization) {
    if (points.size() == 0) {
      return;
    }
    util::PolynomialRegression p(points, 1);
    velocityCharacterization->ka = p.beta(1);
  }

  // Cleans Data
  std::vector<util::Point> DriveCharacterization::getVelocityData(std::vector<DriveCharacterization::VelocityDataPoint> input) {
    bool started = false;
    std::vector<util::Point> output;
    for (int i = 0; i < input.size(); ++i) {
      if (fabs(input.at(i).velocity_) > EPSILON)
          started = true;
      if(started)
        output.push_back(util::Point(input.at(i).velocity_, input.at(i).power_));
    }
    return output;
  }
  std::vector<util::Point> DriveCharacterization::getAccelerationData(std::vector<DriveCharacterization::AccelerationDataPoint> input, DriveCharacterization::CharacterizationConstants constants) {
    std::vector<util::Point> output;
    for (int i = 0; i < input.size(); ++i) {
      output.push_back(util::Point(input.at(i).acceleration_, input.at(i).power_ - constants.kv * input.at(i).velocity_ - constants.ks));
    }
    return output;
  }
}