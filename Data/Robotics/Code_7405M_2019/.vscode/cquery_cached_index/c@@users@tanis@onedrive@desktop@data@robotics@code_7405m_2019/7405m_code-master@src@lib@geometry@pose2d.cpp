//
// Created by alexweiss on 7/2/19.
//

#include "Pose2d.hpp"
#include "../utility/Utility.hpp"
#include <cmath>
#include <stdio.h>
#include <sstream>

namespace geometry {
    Pose2d::Pose2d() {
        translation_ =  Translation2d();
        rotation_ = Rotation2d();
    }
    Pose2d::Pose2d(units::QLength x, units::QLength y, Rotation2d rotation) {
        translation_ =  Translation2d(x, y);
        rotation_ = rotation;
    }
    Pose2d::Pose2d(Translation2d translation, Rotation2d rotation) {
        translation_ =  translation;
        rotation_ = rotation;
    }
    Pose2d::Pose2d(const Pose2d *other) {
        translation_ = Translation2d(other->translation().x(), other->translation().y());
        rotation_ = Rotation2d::fromAngle(other->rotation().getAngle());
    }
    Pose2d Pose2d::pose() {
      return Pose2d(translation_, rotation_);
    }
    Translation2d Pose2d::translation() const{
        return translation_;
    }
    Rotation2d Pose2d::rotation() const{
        return rotation_;
    }
    Pose2d Pose2d::transformBy(Pose2d other) {
        return Pose2d(translation_.translateBy(other.translation_.rotateBy(rotation_)),
                          rotation_.rotateBy(other.rotation_));
    }
    Pose2d Pose2d::inverse() {
        Rotation2d rotation_inverted = rotation_.inverse();
        return Pose2d(translation_.inverse().rotateBy(rotation_inverted), rotation_inverted);
    }
    Pose2d Pose2d::normal() {
        return Pose2d(translation_, rotation_.normal());
    }
    // Find intersection point based on headings
    Translation2d Pose2d::intersection(Pose2d other) {
        Rotation2d other_rotation = other.rotation();
        if (rotation_.isParallel(other_rotation)) {
            return Translation2d(INFINITY, INFINITY);
        }
        if (fabs(rotation_.cos()) < fabs(other_rotation.cos())) {
            return intersectionInternal(*this, other);
        } else {
            return intersectionInternal(other, *this);
        }
    }
    bool Pose2d::IsLinear(Pose2d other) {
        if (!rotation().isParallel(other.rotation()))
            return false;
        Twist2d twist = log(inverse().transformBy(other));
        return FEQUALS(twist.dy_, 0.0 * units::metre) && FEQUALS(twist.dtheta_, 0.0 * units::radian);
    }
    // Interpolation based on CONSTANT CURVATURE
    Pose2d Pose2d::interpolate(Pose2d other, units::Number x) {
        if (x <= 0*units::num) {
            return Pose2d(*this);
        } else if (x >= 1*units::num) {
            return Pose2d(other);
        }
        Twist2d twist = log(inverse().transformBy(other));
        return transformBy(exp(twist.scaled(x)));
    }
    Pose2d Pose2d::mirror() {
        return Pose2d(Translation2d(translation().x(), -1 * translation().y()), rotation().inverse());
    }
    units::QLength Pose2d::distance(Pose2d other) {
        return log(inverse().transformBy(other)).norm();
    }
    std::string Pose2d::toCSV() {
        std::ostringstream stringStream;
        stringStream << "Pose2d," << std::to_string(translation_.x().getValue()) << "," << std::to_string(translation_.y().getValue());
        stringStream << "," << std::to_string(rotation_.getRadians());
        return stringStream.str();
    }
    std::string Pose2d::toString() {
        return toCSV();
    }
    bool Pose2d::operator==(Pose2d other) {
        return (translation_ == other.translation_ && rotation_ == other.rotation_);
    }

    Pose2d Pose2d::fromTranslation(Translation2d translation) {
        return Pose2d(translation, Rotation2d());
    }
    Pose2d Pose2d::fromRotation(Rotation2d rotation) {
        return Pose2d(Translation2d(), rotation);
    }
    // Resulting Pose after a Twist
    // Based on https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
    // Sophus is a MIT Licenced C++ Implementation of Lie Groups
    Pose2d Pose2d::exp(Twist2d delta) {
        double sin_theta = units::sin(delta.dtheta_);
        double cos_theta = units::cos(delta.dtheta_);
        double s, c;
        if (fabs(delta.dtheta_) < EPSILON*units::radian) {
            s = 1.0 - (1.0 / 6.0 * delta.dtheta_ * delta.dtheta_).getValue();
            c = (.5 * delta.dtheta_).getValue();
        } else {
            s = (sin_theta / delta.dtheta_).getValue();
            c = ((1.0 - cos_theta) / delta.dtheta_).getValue();
        }
        return Pose2d(Translation2d(delta.dx_ * s - delta.dy_ * c, delta.dx_ * c + delta.dy_ * s),
                          Rotation2d(cos_theta, sin_theta));
    }
    // Given final Pose, find Twist
    Twist2d Pose2d::log(Pose2d transform) {
        units::Angle dtheta = transform.rotation().getRadians();
        units::Angle half_dtheta = 0.5 * dtheta;
        double cos_minus_one = transform.rotation().cos() - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (fabs(cos_minus_one) < EPSILON) {
            halftheta_by_tan_of_halfdtheta = 1.0 - (1.0 / 12.0 * dtheta * dtheta).getValue();
        } else {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.rotation().sin()).getValue() / cos_minus_one;
        }

        double cos_angle = halftheta_by_tan_of_halfdtheta;
        double sin_angle = -half_dtheta.getValue();
        Translation2d translation_part = transform.translation();
        units::QLength new_x = translation_part.x() * cos_angle - translation_part.y() * sin_angle;
        units::QLength new_y = translation_part.x() * sin_angle + translation_part.y() * cos_angle;


        return Twist2d(new_x, new_y, dtheta);
    }
    Translation2d Pose2d::intersectionInternal(Pose2d a, Pose2d b) {
        Rotation2d a_r = a.rotation();
        Rotation2d b_r = b.rotation();
        Translation2d a_t = a.translation();
        Translation2d b_t = b.translation();

        double tan_b = b_r.tan();
        double t = (((a_t.x() - b_t.x()) * tan_b + b_t.y() - a_t.y())
                         / (a_r.sin() - a_r.cos() * tan_b)).getValue();
        if (std::isnan(t)) {
            return Translation2d(INFINITY, INFINITY);
        }
        return a_t.translateBy(a_r.toTranslation().scale(t));
    }



}