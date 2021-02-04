#ifndef CODE_V1_POSE2DWITHCURVATURE_HPP
#define CODE_V1_POSE2DWITHCURVATURE_HPP

#include "Pose2d.hpp"
#include "Rotation2d.hpp"
#include "Translation2d.hpp"
#include "interfaces/State.hpp"
#include "interfaces/IPose2d.hpp"
#include "interfaces/ICurvature.hpp"
#include <string>

namespace geometry {
  class Pose2dWithCurvature : IPose2d<Pose2dWithCurvature>, ICurvature<Pose2dWithCurvature> {
    private:
      Pose2d pose_;
      units::QCurvature curvature_;
      units::QDCurvatureDs dcurvature_ds_;
    public:
      Pose2dWithCurvature();
      Pose2dWithCurvature(Pose2d pose, units::QCurvature curvature);
      Pose2dWithCurvature(Pose2d pose, units::QCurvature curvature, units::QDCurvatureDs dcurvature_ds);
      Pose2dWithCurvature(Translation2d translation, Rotation2d rotation, units::QCurvature curvature);
      Pose2dWithCurvature(Translation2d translation, Rotation2d rotation, units::QCurvature curvature, units::QDCurvatureDs dcurvature_ds);
      Pose2d pose();
      Pose2dWithCurvature getPose();
      Pose2dWithCurvature transformBy(Pose2d transform);
      Pose2dWithCurvature mirror();
      units::QCurvature curvature();
      units::QDCurvatureDs dcurvature();
      Translation2d translation();
      Rotation2d rotation();
      Pose2dWithCurvature interpolate(Pose2dWithCurvature other, units::Number x);
      units::QLength distance(Pose2dWithCurvature other);
      bool operator==(Pose2dWithCurvature other);
      Pose2dWithCurvature& operator=(Pose2dWithCurvature other);
      std::string toCSV();
      std::string toString();
  };
}

#endif //CODE_V1_POSE2DWITHCURVATURE_HPP
