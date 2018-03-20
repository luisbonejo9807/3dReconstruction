#pragma once

#include "nicp/pinholepointprojector.h"
#include "nicp/depthimageconverter.h"
#include "nicp/cloud.h"
#include "nicp/stats.h"
#include "nicp/informationmatrix.h"
#include "nicp/gaussian3.h"

#include "cloudconfidence.h"

using namespace std;
using namespace nicp;


struct IndexTriplet {
    int x, y, z, index;

    IndexTriplet() {
        x = y = z = 0;
        index = -1;
    }

    IndexTriplet(const Eigen::Vector4f& v, int idx, float ires) {
        x = (int)(ires*v.x());
        y = (int)(ires*v.y());
        z = (int)(ires*v.z());
        index = idx;
    }

    bool operator < (const IndexTriplet& o) const {
        if(z < o.z) { return true; }
        if(z > o.z) { return false; }
        if(x < o.x) { return true; }
        if(x > o.x) { return false; }
        if(y < o.y) { return true; }
        if(y > o.y) { return false; }
        if(index < o.index) { return true; }
        return false;
    }

    bool sameCell(const IndexTriplet& o) const {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct StatsAccumulator {
    StatsAccumulator() {
        acc = 0;
        u = Eigen::Vector3f::Zero();
        omega = Eigen::Matrix3f::Zero();
    }

    int acc;
    Eigen::Vector3f u;
    Eigen::Matrix3f omega;
};


/** \class Merger merger.h "merger.h"
   *  \brief Class for point cloud merging.
   *
   *  This class allows to merge points in a point cloud. This is particularly useful
   *  when multiple point clouds are added togheter in a single one. This creates an excess
   *  of points that are not necessary and slow the computation. The merging process will reduct
   *  the points erasing the ones in surplus.
   */
class MyMerger {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Empty constructor.
     *  This constructor creates a Merger with default values for all its attributes.
     *  All the pointers to objects implementing an algorithm have to be set since
     *  this constructor sets them to zero.
     */
    MyMerger();

    /**
     *  Destructor.
     */
    virtual ~MyMerger() {}

    /**
     *  Method that returns the distance threshold used by the Merger.
     *  @return the distance threshold used by the Merger.
     *  @see setDistanceThreshold()
     */
    inline float distanceThreshold() const { return _distanceThreshold; }

    /**
     *  Method that set the distance threshold used by the Merger to the one given input.
     *  @param distanceThreshold_ is a float value used to update the distance threshold used by the Merger.
     *  @see distanceThreshold()
     */
    inline void setDistanceThreshold(float distanceThreshold_) { _distanceThreshold = distanceThreshold_; }

    /**
     *  Method that returns the normal threshold used by the Merger.
     *  @return the normal threshold used by the Merger.
     *  @see setNormalThreshold()
     */
    inline float normalThreshold() const { return _normalThreshold; }

    /**
     *  Method that set the normal threshold used by the Merger to the one given input.
     *  @param normalThreshold_ is a float value used to update the normal threshold used by the Merger.
     *  @see normalThreshold()
     */
    inline void setNormalThreshold(float normalThreshold_) { _normalThreshold = normalThreshold_; }

    /**
     *  Method that returns the depth threshold used by the Merger.
     *  @return the depth threshold used by the Merger.
     *  @see setMaxPointDepth()
     */
    inline float maxPointDepth() const { return _maxPointDepth; }

    /**
     *  Method that set the depth threshold used by the Merger to the one given input.
     *  @param maxPointDepth_ is a float value used to update the depth threshold used by the Merger.
     *  @see maxPointDepth()
     */
    inline void setMaxPointDepth(float maxPointDepth_) { _maxPointDepth = maxPointDepth_; }

    /**
     *  Method that returns the DepthImageConverter used by the Merger.
     *  @return the DepthImageConverter used by the Merger.
     *  @see setDepthImageConverter()
     */
    inline DepthImageConverter* depthImageConverter() const { return _depthImageConverter; }

    /**
     *  Method that set the DepthImageConverter used by the Merger to the one given input.
     *  @param depthImageConverter_ is a pointer used to update the depthImageConverter used by the Merger.
     *  @see depthImageConverter()
     */
    inline void setDepthImageConverter(DepthImageConverter *depthImageConverter_) { _depthImageConverter = depthImageConverter_; }

    /**
     *  Method that returns the image size set to the Merger.
     *  @return the image size set to the Merger.
     *  @see setImageSize()
     */
    inline Eigen::Vector2i imageSize() const { return Eigen::Vector2i(_indexImage.rows, _indexImage.cols); }

    /**
     *  Method that set the the image size of the Merger to the one given input.
     *  @param r is an int value representing the number of rows of the image.
     *  @param c is an int value representing the number of cols of the image.
     *  @see imageSize()
     */
    inline void setImageSize(int r, int c) {
        _indexImage.create(r, c);
        _depthImage.create(r, c);
    }

    /**
     *  Method that computes the merged point cloud given a transformation pose and the point cloud to merge.
     *  @param cloud is a pointer to the cloud to merge.
     *  @param transform is the pose transofrmation to use in the merging process.
     */
    void merge(CloudConfidence *cloud, Eigen::Isometry3f transform = Eigen::Isometry3f::Identity());

    void voxelize(Cloud* model, float res);

protected:
    float _distanceThreshold; /**< Distance threshold for which points over it are not collapsed. */
    float _normalThreshold; /**< Normal threshold for which points with normals over it are not collapsed. */
    float _maxPointDepth; /**< Depth threshold for which points over it are not collapsed. */

    DepthImageConverter *_depthImageConverter; /**< Pointer to the DepthImageConverter. */

    DepthImage _depthImage; /**< DepthImage for inner computations. */
    IntImage _indexImage; /**< IndexImage for inner computations. */
    std::vector<int> _collapsedIndices; /**< Vector of collapsed point indeces to merge. */
};
