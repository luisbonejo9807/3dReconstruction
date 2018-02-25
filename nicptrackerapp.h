#ifndef NICPTRACKERAPP_H
#define NICPTRACKERAPP_H

#include <iostream>
#include <fstream>
#include <sys/time.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "nicp/bm_se3.h"
#include "nicp/imageutils.h"
#include "nicp/pinholepointprojector.h"
#include "nicp/depthimageconverterintegralimage.h"
#include "nicp/statscalculatorintegralimage.h"
#include "nicp/alignerprojective.h"
#include "nicp/merger.h"

#include <nicp_viewer/nicp_qglviewer.h>
#include <nicp_viewer/drawable_points.h>

#include "nicptrackerappviewer.h"

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace nicp;

class NICPTrackerApp {
public:

    NICPTrackerApp(const std::string& configurationFile, NICPTrackerAppViewer* viewer_ = 0);

    ~NICPTrackerApp();

    void init(const std::string& configurationFile);

    void setInputParameters(map<string, std::vector<float> >& inputParameters);

    bool fillInputParametersMap(map<string, std::vector<float> >& inputParameters,
                                const string& configurationFilename);

    inline double get_time() {
        struct timeval ts;
        gettimeofday(&ts, 0);
        return ts.tv_sec + ts.tv_usec * 1e-6;
    }
    void compareDepths(float& in_distance, int& in_num,
                       float& out_distance, int& out_num,
                       const FloatImage& depths1, const IntImage& indices1,
                       const FloatImage& depths2, const IntImage& indices2,
                       float dist, bool scale_z);

    Eigen::Isometry3f globalT() const { return _globalT; }

    void setStartingPose(const Eigen::Isometry3f startingPose);

    double spinOnce(Eigen::Isometry3f& deltaT, const std::string& depthFilename,
                    const std::string &rgbFileName);

protected:
    double _tBegin, _tEnd;
    double _tInput, _tAlign, _tUpdate;

    bool _demoteToGICP;
    int _seq;
    int _rows, _cols;
    int _imageScaling;
    int _inNum, _outNum;
    float _inDistance, _outDistance;
    float _depthScaling;
    float _breakingAngle, _breakingDistance, _breakingInlierRatio;
    Matrix3f _K;

    Eigen::Isometry3f _deltaT, _globalT, _localT;

    RawDepthImage _rawDepth;
    RGBImage _rgbImage;
    DepthImage _depth, _scaledDepth, _referenceScaledDepth;
    IndexImage _scaledIndeces, _referenceScaledIndeces;
    Cloud* _referenceScene;
    Cloud* _referenceCloud;
    Cloud* _currentCloud;

    PinholePointProjector _projector;
    StatsCalculatorIntegralImage _statsCalculator;
    PointInformationMatrixCalculator _pointInformationMatrixCalculator;
    NormalInformationMatrixCalculator _normalInformationMatrixCalculator;
    DepthImageConverterIntegralImage _converter;
    CorrespondenceFinderProjective _correspondenceFinder;
    Linearizer _linearizer;
    AlignerProjective _aligner;
    Merger _merger;

    NICPTrackerAppViewer* _viewer;
};

#endif // NICPTRACKERAPP_H
