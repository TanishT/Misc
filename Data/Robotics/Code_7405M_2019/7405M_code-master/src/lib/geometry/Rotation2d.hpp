#ifndef CODE_V1_ROTATION2D_HPP
#define CODE_V1_ROTATION2D_HPP

#include "interfaces/IRotation2d.hpp"
#include "../utility/Units.hpp"

namespace geometry {

    class Translation2d;

    class Rotation2d : IRotation2d<Rotation2d> {
    private:
        double cos_angle_;
        double sin_angle_;
    public:
        Rotation2d();
        Rotation2d(Translation2d other);
        Rotation2d(units::Angle a);
        Rotation2d(units::QLength x, units::QLength y);
        Rotation2d(double x, double y);
        double cos();
        double sin();
        double tan();
        units::Angle getAngle();
        double getRadians();
        double getDegrees();
        Rotation2d rotateBy(Rotation2d other);
        Rotation2d normal();
        Rotation2d inverse();
        Translation2d toTranslation();
        bool isParallel(Rotation2d other);
        units::QLength distance(Rotation2d other);
        bool operator==(Rotation2d other);
        Rotation2d interpolate(Rotation2d other, units::Number x);
        std::string toCSV();
        std::string toString();

        static Rotation2d fromAngle(units::Angle angle);
        static Rotation2d fromRadians(units::Number angle_radians);
        static Rotation2d fromDegrees(units::Number angle_degrees);
    };
}

#endif //CODE_V1_ROTATION2D_HPP
