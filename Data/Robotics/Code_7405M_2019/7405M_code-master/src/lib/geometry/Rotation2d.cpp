#define _USE_MATH_DEFINES

#include "Rotation2d.hpp"
#include "Translation2d.hpp"
#include "../utility/Utility.hpp"
#include <cmath>
#include <math.h>
#include <string>
#include <sstream>
#include <stdio.h>

namespace geometry {
    Rotation2d::Rotation2d() {
      sin_angle_ = 0.0;
      cos_angle_ = 1.0;
    }
    Rotation2d::Rotation2d(Translation2d other) {
      Rotation2d(other.x().getValue(), other.y().getValue());
    }

    Rotation2d::Rotation2d(units::Angle a) {
      Rotation2d(units::cos(a), units::sin(a));
    }
    Rotation2d::Rotation2d(units::QLength x, units::QLength y) {
      Rotation2d(x.getValue(), y.getValue());
    }
    Rotation2d::Rotation2d(double x, double y) {
      double magnitude = std::hypot(x,y);
      sin_angle_ = y / magnitude;
      cos_angle_ = x / magnitude;
    }

    double Rotation2d::cos() {
      return cos_angle_;
    }
    double Rotation2d::sin() {
      return sin_angle_;
    }
    double Rotation2d::tan() {
      if(std::fabs(cos_angle_) < EPSILON) {
          if (sin_angle_ > 0)
              return HUGE_VAL;
          return -HUGE_VAL;
      }
      return sin_angle_ / cos_angle_;
    }
    units::Angle Rotation2d::getAngle() {
      return std::atan2(sin_angle_, cos_angle_) * units::radian;
    }
    double Rotation2d::getRadians() {
      return getAngle().Convert(units::radian);
    }
    double Rotation2d::getDegrees() {
      return getAngle().Convert(units::degree);
    }
    Rotation2d Rotation2d::rotateBy(Rotation2d other) {
      return Rotation2d(cos_angle_ * other.cos_angle_ - sin_angle_ * other.sin_angle_,
                            cos_angle_ * other.sin_angle_ + sin_angle_ * other.cos_angle_);
    }
    Rotation2d Rotation2d::normal() {
      return Rotation2d(-sin_angle_, cos_angle_);
    }
    Rotation2d Rotation2d::inverse() {
      return Rotation2d(cos_angle_, -sin_angle_);
    }
    bool Rotation2d::isParallel(Rotation2d other) {
      return FEQUALS(Translation2d::cross(toTranslation(), other.toTranslation()), 0.0*units::metre2);
    }
    Translation2d Rotation2d::toTranslation() {
      return Translation2d(cos_angle_, sin_angle_);
    }
    units::QLength Rotation2d::distance(Rotation2d other) {
      return inverse().rotateBy(other).getRadians();
    }
    bool Rotation2d::operator==(Rotation2d other) {
      return FEQUALS(cos_angle_, other.cos_angle_) && FEQUALS(sin_angle_, other.sin_angle_);
    }
    Rotation2d Rotation2d::interpolate(Rotation2d other, units::Number x) {
      x = LIMIT(x, 0.0*units::num, 1.0*units::num);
      double angle_diff = inverse().rotateBy(other).getRadians();
      return rotateBy(Rotation2d::fromRadians(angle_diff * x));
    }
    std::string Rotation2d::toCSV() {
      std::ostringstream stringStream;
      stringStream << "Rotation2d," << std::to_string(cos_angle_) << "," << std::to_string(sin_angle_);
      return stringStream.str();
    }
    std::string Rotation2d::toString() {
      return toCSV();
    }

    Rotation2d Rotation2d::fromAngle(units::Angle angle) {
      return Rotation2d(units::cos(angle), units::sin(angle));
    }
    Rotation2d Rotation2d::fromRadians(units::Number angle_radians) {
      return fromAngle(angle_radians * units::radian);
    }
    Rotation2d Rotation2d::fromDegrees(units::Number angle_degrees) {
      return fromAngle(angle_degrees * units::degree);
    }
}