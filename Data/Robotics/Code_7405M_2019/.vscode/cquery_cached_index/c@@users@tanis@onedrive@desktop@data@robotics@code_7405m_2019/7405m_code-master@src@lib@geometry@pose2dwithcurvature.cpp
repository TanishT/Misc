//
// Created by alexweiss on 7/3/19.
//

#include "Pose2dWithCurvature.hpp"
#include "Pose2d.hpp"
#include "Translation2d.hpp"
#include "Rotation2d.hpp"
#include "../utility/Utility.hpp"
#include <cmath>
#include <sstream>

namespace geometry {
    Pose2dWithCurvature::Pose2dWithCurvature() {
        pose_ = Pose2d();
        curvature_ = 0.0;
        dcurvature_ds_ = 0.0;
    }
    Pose2dWithCurvature::Pose2dWithCurvature(Pose2d pose, units::QCurvature curvature) {
        pose_ = pose;
        curvature_ = curvature;
        dcurvature_ds_ = 0.0;
    }
    Pose2dWithCurvature::Pose2dWithCurvature(Pose2d pose, units::QCurvature curvature, units::QDCurvatureDs dcurvature_ds) {
        pose_ = pose;
        curvature_ = curvature;
        dcurvature_ds_ = dcurvature_ds;
    }
    Pose2dWithCurvature::Pose2dWithCurvature(Translation2d translation, Rotation2d rotation, units::QCurvature curvature) {
        pose_ = Pose2d(translation, rotation);
        curvature_ = curvature;
        dcurvature_ds_ = 0.0;
    }
    Pose2dWithCurvature::Pose2dWithCurvature(Translation2d translation, Rotation2d rotation, units::QCurvature curvature, units::QDCurvatureDs dcurvature_ds) {
        pose_ = Pose2d(translation, rotation);
        curvature_ = curvature;
        dcurvature_ds_ = dcurvature_ds;
    }
    Pose2dWithCurvature Pose2dWithCurvature::getPose() {
      return Pose2dWithCurvature(pose_, curvature_, dcurvature_ds_);
    }
    Pose2d Pose2dWithCurvature::pose() {
        return pose_;
    }
    Pose2dWithCurvature Pose2dWithCurvature::transformBy(Pose2d transform) {
        return Pose2dWithCurvature(pose().transformBy(transform), curvature(), dcurvature());
    }
    Pose2dWithCurvature Pose2dWithCurvature::mirror() {
        return Pose2dWithCurvature(pose().mirror(), -1 * curvature(), -1 * dcurvature());
    }
    units::QCurvature Pose2dWithCurvature::curvature() {
        return curvature_;
    }
    units::QDCurvatureDs Pose2dWithCurvature::dcurvature() {
        return dcurvature_ds_;
    }
    Translation2d Pose2dWithCurvature::translation() {
        return pose_.translation();
    }
    Rotation2d Pose2dWithCurvature::rotation() {
        return pose_.rotation();
    }
    Pose2dWithCurvature Pose2dWithCurvature::interpolate(Pose2dWithCurvature other, units::Number x) {
        return Pose2dWithCurvature(pose().interpolate(other.pose(), x),
                INTERPOLATE(curvature_, other.curvature(), x),
                INTERPOLATE(dcurvature_ds_, other.dcurvature(), x));
    }
    units::QLength Pose2dWithCurvature::distance(Pose2dWithCurvature other) {
        return pose().distance(other.pose());
    }
    bool Pose2dWithCurvature::operator==(Pose2dWithCurvature other) {
        return pose_ == other.pose() && curvature_ == other.curvature() && dcurvature_ds_ == other.dcurvature();
    }
    Pose2dWithCurvature& Pose2dWithCurvature::operator=(Pose2dWithCurvature other) {
        pose_ = Pose2d(other.pose());
        curvature_ = other.curvature();
        dcurvature_ds_ = other.dcurvature();
        return *this;
    }
    std::string Pose2dWithCurvature::toCSV() {
        std::ostringstream stringStream;
        stringStream << "Pose2dWithCurvature,(" << pose_.toCSV() << ")," << curvature_.to_string();
        stringStream << "," << dcurvature_ds_.to_string();
        return stringStream.str();
    }
    std::string Pose2dWithCurvature::toString() {
        return toCSV();
    }
}