#ifndef CODE_V1_IPOSE2D_HPP
#define CODE_V1_IPOSE2D_HPP

#include "IRotation2d.hpp"
#include "ITranslation2d.hpp"

namespace geometry {
  class Pose2d;

  template<typename T> class IPose2d :  IRotation2d<T>, ITranslation2d<T> {
    public:
      virtual Pose2d pose() = 0;
      virtual T transformBy(Pose2d transform) = 0;
      virtual T mirror() = 0;
  };
}


#endif //CODE_V1_IPOSE2D_HPP
