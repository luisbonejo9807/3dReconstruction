#ifndef PINHOLEPOINTCOLORPROJECTOR_H
#define PINHOLEPOINTCOLORPROJECTOR_H
#include "nicp/pinholepointprojector.h"

#pragma once

#include "pointprojector.h"

namespace nicp {

/** \class PinholePointProjector pinholepointprojector.h "pinholepointprojector.h"
   *  \brief Class for point projection/unprojection based on pinhole camera mdel.
   *
   *  This class extends the PointProjector class in order to provide point
   *  projection/unprojection based on pinhole camera model.
   */
class PinholePointColorProjector : virtual public PinholePointProjector {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Virtual method that unprojects to the 3D euclidean space the points contained in a depth image
     *  This method stores the unprojected points in a vector of points and generates
     *  the associated index image. It also compute a vector of gaussians reprensenting the intrinsic error
     *  of the sensor used to acquire the input depth image.
     *  @param points is an output parameter which is a reference to a vector containing the set
     *  of homogeneous points to unproject to the 3D euclidean space.
     *  @param gaussians is an output parameter which is a reference to a vector containing the set
     *  of gaussians representing the intrinsic error of the sensor used to acquire the input depth image.
     *  @param indexImage is an output parameter which is a reference to an image containing indices.
     *  Each element of this matrix contains the index of the corresponding point in the computed vector of points.
     *  @param depthImage is an output parameter which is a constant reference to an image containing the depth values
     *  in meters.
     *  @see project()
     *  @see projectIntervals()
     */
    void unProject(PointVector &points,
                   Gaussian3fVector &gaussians,
                   IntImage &indexImage,
                   const DepthImage &depthImage,
                   const RGBImage &rgbImage) const;
};

}

#endif // PINHOLEPOINTCOLORPROJECTOR_H
