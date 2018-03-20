#pragma once

#include "nicp/stats.h"
#include "nicp/informationmatrix.h"
#include "nicp/gaussian3.h"
#include "nicp/cloud.h"
#include <vector>

using namespace std;
using namespace nicp;

/** \class CloudConfidence cloudconfidence.h "cloudconfidence.h"
   *  \brief Class for point cloud representation with extension to store confidence information over each point.
   *
   *  This class has the objective to easy represent a point cloud. In particular it maintains in a vector
   *  structure the homogenous 3D points composing the cloud. Using this class it is possible also to maintain
   *  additional information like the point normals, point properties, point information matrices, normal
   *  information matrices, a vector of gaussians representing the intrinsic error due to the sensor and also
   *  a traversability information about the points.
   */
class CloudConfidence : public Cloud{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    vector<unsigned int> _pointsMatchingCounter; /**< Vector to store how many times each point have been matched with any other point.*/

    CloudConfidence() {}

    void resize(size_t s, bool hasRGB) {
        if (s) {
            _points.resize(s);
            _pointsMatchingCounter.resize(s);
            _normals.resize(s);
            _stats.resize(s);
            _normalInformationMatrix.resize(s);
            _pointInformationMatrix.resize(s);
            _gaussians.resize(s);
            if(hasRGB) { _rgbs.resize(s); }
        } else
            clear();
    }

    void add(Cloud& cloud, const Eigen::Isometry3f &T) {
        cloud.transformInPlace(T);
        size_t k = _points.size();
        _pointsMatchingCounter.resize(k + cloud.points().size());
        _points.resize(k + cloud.points().size());
        _normals.resize(k + cloud.normals().size());
        _stats.resize(k + cloud.stats().size());
        _pointInformationMatrix.resize(k + cloud.pointInformationMatrix().size());
        _normalInformationMatrix.resize(k + cloud.normalInformationMatrix().size());
        _gaussians.resize(k + cloud.gaussians().size());
        if (_rgbs.size() + cloud.rgbs().size())
            _rgbs.resize(k + cloud.rgbs().size());
        for(int i = 0; k < _points.size(); k++, i++) {
            _points[k] = cloud.points()[i];
            _normals[k] = cloud.normals()[i];
            _stats[k] = cloud.stats()[i];
            // if(cloud.pointInformationMatrix().size() != 0) {
            _pointInformationMatrix[k] = cloud.pointInformationMatrix()[i];
            _normalInformationMatrix[k] = cloud.normalInformationMatrix()[i];
            // }
            // if(cloud.gaussians().size() != 0) {
            _gaussians[k] = cloud.gaussians()[i];
            // }
            if(cloud.rgbs().size() != 0) {
                _rgbs[k] = cloud.rgbs()[i];
            }
        }
    }
};
