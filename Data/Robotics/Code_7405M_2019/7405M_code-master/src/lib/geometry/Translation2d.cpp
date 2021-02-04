#include "Translation2d.hpp"
#include "Rotation2d.hpp"
#include "../utility/Utility.hpp"
#include <cmath>
#include <math.h>
#include <string>
#include <sstream>

namespace geometry {
    Translation2d::Translation2d() {
      x_ = 0;
      y_ = 0;
    }
    Translation2d::Translation2d(Translation2d start, Translation2d end) {
      x_ = end.x_ - start.x_;
      y_ = end.y_ - start.y_;
    }
    Translation2d::Translation2d(units::QLength x, units::QLength y) {
      x_ = x;
      y_ = y;
    }
    Translation2d Translation2d::translation() {
      return Translation2d(x_, y_);
    }
    units::QLength Translation2d::norm() {
      return units::Qsqrt(x_ * x_ + y_ * y_);
    }
    units::QLength Translation2d::x() {
      return x_;
    }
    units::QLength Translation2d::y() {
      return y_;
    }
    Translation2d Translation2d::translateBy(Translation2d other) {
      return Translation2d(x_ + other.x_, y_ + other.y_);
    }
    Translation2d Translation2d::rotateBy(Rotation2d rotation){
      return Translation2d(x_ * rotation.cos() - y_ * rotation.sin(), x_ * rotation.sin() + y_ * rotation.cos());
    }
    Rotation2d Translation2d::direction() {
      return Rotation2d(x_, y_);
    }
    Translation2d Translation2d::inverse() {
      return Translation2d(x_ * -1, y_ * -1);
    }
    Translation2d Translation2d::interpolate(Translation2d other, units::Number x) {
      x = LIMIT(x, 0.0*units::num, 1.0*units::num);
      return extrapolate(other, x);
    }
    Translation2d Translation2d::extrapolate(Translation2d other, units::Number x) {
      return Translation2d(x * (other.x_ - x_) + x_, x * (other.y_ - y_) + y_);
    }
    Translation2d Translation2d::scale(units::Number s) {
      return Translation2d(x_ * s, y_ * s);
    }
    bool Translation2d::operator==(Translation2d other) {
      return FEQUALS(x_, other.x_) && FEQUALS(y_, other.y_);
    }
    units::QLength Translation2d::distance(Translation2d other) {
      return inverse().translateBy(other).norm().getValue();
    }
    std::string Translation2d::toCSV() {
      std::ostringstream stringStream;
      stringStream << "Translation2d," << std::to_string(x_.getValue()) << "," << std::to_string(y_.getValue());
      return stringStream.str();
    }
    std::string Translation2d::toString() {
      return toCSV();
    }
    units::QArea Translation2d::dot(Translation2d a, Translation2d b) {
      return a.x_ * b.x_ + a.y_ * b.y_;
    }
    units::QArea Translation2d::cross(Translation2d a, Translation2d b) {
      return a.x_ * b.y_ - a.y_ * b.x_;
    }
    Rotation2d Translation2d::getAngle(Translation2d a, Translation2d b) {
      double cos_angle = (dot(a, b) / (a.norm() * b.norm())).getValue();
      if(std::isnan(cos_angle))
        return Rotation2d();
      return Rotation2d::fromRadians(std::acos(std::min(1.0, std::max(cos_angle, -1.0))));
    }

}