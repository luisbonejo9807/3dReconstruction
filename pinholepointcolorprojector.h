#pragma once
#include "nicp/pinholepointprojector.h"

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
         *  Empty constructor.
         *  This constructor creates a PinholePointProjector object with default values for all its
         *  attributes. The camera matrix has to be set.
         */

        virtual void unProject(PointVector &points,
                               Gaussian3fVector &gaussians,
                               IntImage &indexImage,
                               const DepthImage &depthImage,
                               const RGBImage &rgbImage) const{
            assert(depthImage.rows > 0 && depthImage.cols > 0 && "PinholePointColorProjector: Depth image has zero dimensions");
            points.resize(depthImage.rows * depthImage.cols);
            gaussians.resize(depthImage.rows * depthImage.cols);
            indexImage.create(depthImage.rows, depthImage.cols);

            int count = 0;
            Point *point = &points[0];
            Gaussian3f *gaussian = &gaussians[0];
            float fB = _baseline * _cameraMatrix(0, 0);
            Eigen::Matrix3f J;
            for(int r = 0; r < depthImage.rows; r++) {
                const float *f = &depthImage(r, 0);
                int *i = &indexImage(r, 0);
                for(int c = 0; c < depthImage.cols; c++, f++, i++) {
                    if(!_unProject(*point, c, r, *f) || (
                                rgbImage.at<cv::Vec3b>(r,c).val[0] <= 100
                                && rgbImage.at<cv::Vec3b>(r,c).val[1] <= 100
                                && rgbImage.at<cv::Vec3b>(r,c).val[2] <= 100)) {
                        *i = -1;
                        continue;
                    }
                    float z = *f;
                    float zVariation = (_alpha * z * z) / (fB + z * _alpha);
                    J <<
                         z, 0, (float)c,
                            0, z, (float)r,
                            0, 0, 1;
                    J = _iK * J;
                    Diagonal3f imageCovariance(3.0f, 3.0f, zVariation);
                    Eigen::Matrix3f cov = J * imageCovariance * J.transpose();
                    *gaussian = Gaussian3f(point->head<3>(), cov);
                    gaussian++;
                    point++;
                    *i = count;
                    count++;
                }
            }
        }
    };
}
