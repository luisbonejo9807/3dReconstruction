#include "nicptrackerapp.h"

NICPTrackerApp::NICPTrackerApp(const std::string& configurationFile, NICPTrackerAppViewer* viewer_ = 0) {
    _viewer = viewer_;
    if(_viewer) { std::cout << "[INFO] g  enabled viewer " << _viewer << std::endl; }
    else { std::cout << "[INFO]: g  disabled viewer " << _viewer << std::endl; }

    _demoteToGICP = false;
    _seq = 0;
    _rows = 480;
    _cols = 640;
    _imageScaling = 1;
    _depthScaling = 0.001f;
    _breakingAngle = M_PI / 2.0f;
    _breakingDistance = 1.0f;
    _breakingInlierRatio = 0.75f;
    _K <<
          525.0f,   0.0f, 319.5f,
            0.0f, 525.0f, 239.5f,
            0.0f,   0.0f,   1.0f;

    _deltaT.setIdentity();
    _deltaT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _globalT.setIdentity();
    _globalT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _localT.setIdentity();
    _localT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

    _currentCloud = new Cloud();
    _finalCloud = new CloudConfidence();
    _referenceScene = new CloudConfidence();

    init(configurationFile);
}

NICPTrackerApp::~NICPTrackerApp() {}

void NICPTrackerApp::init(const std::string& configurationFile) {
    // Fill input parameter map
    map<string, std::vector<float> > inputParameters;
    bool fillInputParameters =  fillInputParametersMap(inputParameters, configurationFile);
    if(!fillInputParameters) {
        std::cerr << "[ERROR]: error occurred reading input parameters... quitting" << std::endl;
        exit(-1);
    }

    setInputParameters(inputParameters);
}

void NICPTrackerApp::setInputParameters(map<string, std::vector<float> >& inputParameters) {
    map<string, std::vector<float> >::iterator it;

    // General
    if((it = inputParameters.find("depthScaling")) != inputParameters.end()) _depthScaling = ((*it).second)[0];
    if((it = inputParameters.find("imageScaling")) != inputParameters.end()) _imageScaling = ((*it).second)[0];
    Vector3f initialTranslation = Vector3f(0.0f, 0.0f, 0.0f);
    Quaternionf initialRotation = Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
    if((it = inputParameters.find("startingPose")) != inputParameters.end()) {
        if((*it).second.size() == 7) {
            initialTranslation.x() = ((*it).second)[0];
            initialTranslation.y() = ((*it).second)[1];
            initialTranslation.z() = ((*it).second)[2];
            initialRotation.x() = ((*it).second)[3];
            initialRotation.y() = ((*it).second)[4];
            initialRotation.z() = ((*it).second)[5];
            initialRotation.w() = ((*it).second)[6];
        }
        else { std::cerr << "[WARNING]: startingPose has bad formatting from file, expecting 7 values but " << (*it).second.size() << " where given... keeping default values" << std::endl; }
    }
    initialRotation.normalize();
    _globalT.translation() = initialTranslation;
    _globalT.linear() = initialRotation.toRotationMatrix();
    _globalT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    if((it = inputParameters.find("breakingAngle")) != inputParameters.end()) _breakingAngle = ((*it).second)[0];
    if((it = inputParameters.find("breakingDistance")) != inputParameters.end()) _breakingDistance = ((*it).second)[0];
    if((it = inputParameters.find("breakingInlierRatio")) != inputParameters.end()) _breakingInlierRatio = ((*it).second)[0];
    if((it = inputParameters.find("demoteToGICP")) != inputParameters.end()) _demoteToGICP = ((*it).second)[0];
    std::cout << "[INFO]: g  depth scaling " << _depthScaling << std::endl;
    std::cout << "[INFO]: g  image scaling " << _imageScaling << std::endl;
    std::cout << "[INFO]: g  starting pose " << t2v(_globalT).transpose() << std::endl;
    std::cout << "[INFO]: g  breaking angle " << _breakingAngle << std::endl;
    std::cout << "[INFO]: g  breaking distance " << _breakingDistance << std::endl;
    std::cout << "[INFO]: g  breaking inlier ratio " << _breakingInlierRatio << std::endl;
    if(_demoteToGICP) { std::cout << "[INFO]: g  running GICP " << std::endl; }
    else { std::cout << "[INFO]: g  running NICP " << std::endl;}

    // Point projector
    if((it = inputParameters.find("K")) != inputParameters.end()) {
        if((*it).second.size() == 4) {
            _K(0, 0) = ((*it).second)[0];
            _K(1, 1) = ((*it).second)[1];
            _K(0, 2) = ((*it).second)[2];
            _K(1, 2) = ((*it).second)[3];
        }
        else { std::cerr << "[WARNING]: K has bad formatting from file, expecting 4 values but " << (*it).second.size() << " where given... keeping default values" << std::endl; }
    }
    if((it = inputParameters.find("minDistance")) != inputParameters.end()) _projector.setMinDistance(((*it).second)[0]);
    if((it = inputParameters.find("maxDistance")) != inputParameters.end()) _projector.setMaxDistance(((*it).second)[0]);
    if((it = inputParameters.find("imageSize")) != inputParameters.end()) {
        if((*it).second.size() == 2) {
            _rows = ((*it).second)[0];
            _cols = ((*it).second)[1];
        }
        else { std::cerr << "[WARNING]: imageSize has bad formatting from file, expecting 2 values but " << (*it).second.size() << " where given... keeping default values" << std::endl; }
    }
    _projector.setTransform(Eigen::Isometry3f::Identity());
    _projector.setCameraMatrix(_K);
    _projector.setImageSize(_rows, _cols);
    _projector.scale(1.0f / (float)_imageScaling);
    std::cout << "[INFO]: pp offset " << t2v(_projector.transform()).transpose() << std::endl;
    std::cout << "[INFO]: pp minimum distance " << _projector.minDistance() << std::endl;
    std::cout << "[INFO]: pp maximum distance " << _projector.maxDistance() << std::endl;
    std::cout << "[INFO]: pp K " << std::endl << _projector.cameraMatrix() << std::endl;
    std::cout << "[INFO]: pp image size " << _projector.imageRows() << " --- " << _projector.imageCols() << std::endl;

    // Stats calculator and information matrix calculators
    if((it = inputParameters.find("minImageRadius")) != inputParameters.end()) _statsCalculator.setMinImageRadius(((*it).second)[0]);
    if((it = inputParameters.find("maxImageRadius")) != inputParameters.end()) _statsCalculator.setMaxImageRadius(((*it).second)[0]);
    if((it = inputParameters.find("minPoints")) != inputParameters.end()) _statsCalculator.setMinPoints(((*it).second)[0]);
    if((it = inputParameters.find("statsCalculatorCurvatureThreshold")) != inputParameters.end()) _statsCalculator.setCurvatureThreshold(((*it).second)[0]);
    if((it = inputParameters.find("worldRadius")) != inputParameters.end()) _statsCalculator.setWorldRadius(((*it).second)[0]);
    if((it = inputParameters.find("informationMatrixCurvatureThreshold")) != inputParameters.end()) {
        _pointInformationMatrixCalculator.setCurvatureThreshold(((*it).second)[0]);
        _normalInformationMatrixCalculator.setCurvatureThreshold(((*it).second)[0]);
    }
    std::cout << "[INFO]: sc minimum image radius " << _statsCalculator.minImageRadius() << std::endl;
    std::cout << "[INFO]: sc maximum image radius " << _statsCalculator.maxImageRadius() << std::endl;
    std::cout << "[INFO]: sc minimum points " << _statsCalculator.minPoints() << std::endl;
    std::cout << "[INFO]: sc curvature threshold " << _statsCalculator.curvatureThreshold() << std::endl;
    std::cout << "[INFO]: sc world radius " << _statsCalculator.worldRadius() << std::endl;
    std::cout << "[INFO]: ic curvature threshold " << _normalInformationMatrixCalculator.curvatureThreshold() << std::endl;

    // Correspondence finder
    if((it = inputParameters.find("inlierDistanceThreshold")) != inputParameters.end()) _correspondenceFinder.setInlierDistanceThreshold(((*it).second)[0]);
    if((it = inputParameters.find("inlierNormalAngularThreshold")) != inputParameters.end()) _correspondenceFinder.setInlierNormalAngularThreshold(((*it).second)[0]);
    if((it = inputParameters.find("inlierCurvatureRatioThreshold")) != inputParameters.end()) _correspondenceFinder.setInlierCurvatureRatioThreshold(((*it).second)[0]);
    if((it = inputParameters.find("correspondenceFinderCurvatureThreshold")) != inputParameters.end()) _correspondenceFinder.setFlatCurvatureThreshold(((*it).second)[0]);
    _correspondenceFinder.setImageSize(_projector.imageRows(), _projector.imageCols());
    _correspondenceFinder.setDemotedToGICP(_demoteToGICP);
    std::cout << "[INFO]: cf inlier distance threshold " << _correspondenceFinder.inlierDistanceThreshold() << std::endl;
    std::cout << "[INFO]: cf inlier normal angular threshold " << _correspondenceFinder.inlierNormalAngularThreshold() << std::endl;
    std::cout << "[INFO]: cf inlier curvature ratio threshold " << _correspondenceFinder.inlierCurvatureRatioThreshold() << std::endl;
    std::cout << "[INFO]: cf curvature threshold " << _correspondenceFinder.flatCurvatureThreshold() << std::endl;
    std::cout << "[INFO]: cf image size " << _correspondenceFinder.imageRows() << " --- " << _correspondenceFinder.imageCols() << std::endl;

    // Linearizer
    if((it = inputParameters.find("inlierMaxChi2")) != inputParameters.end()) _linearizer.setInlierMaxChi2(((*it).second)[0]);
    if((it = inputParameters.find("robustKernel")) != inputParameters.end()) _linearizer.setRobustKernel(((*it).second)[0]);
    if((it = inputParameters.find("zScaling")) != inputParameters.end()) _linearizer.setZScaling(((*it).second)[0]);
    _linearizer.setAligner(&_aligner);
    _linearizer.setDemotedToGeneralizedICP(_demoteToGICP);
    std::cout << "[INFO]: l  inlier maximum chi2 " << _linearizer.inlierMaxChi2() << std::endl;
    if(_linearizer.robustKernel()) { std::cout << "[INFO]: l  robust kernel enabled" << std::endl; }
    else { std::cout << "[INFO]: l  robust kernel disabled" << std::endl; }
    if(_linearizer.zScaling()) { std::cout << "[INFO]: l  z scaling enabled" << std::endl; }
    else { std::cout << "[INFO]: l  z scaling disabled" << std::endl; }
    std::cout << "[INFO]: l  aligner " << _linearizer.aligner() << std::endl;

    // Aligner
    if((it = inputParameters.find("outerIterations")) != inputParameters.end()) _aligner.setOuterIterations(((*it).second)[0]);
    if((it = inputParameters.find("innerIterations")) != inputParameters.end()) _aligner.setInnerIterations(((*it).second)[0]);
    if((it = inputParameters.find("minInliers")) != inputParameters.end()) _aligner.setMinInliers(((*it).second)[0]);
    if((it = inputParameters.find("translationalMinEigenRatio")) != inputParameters.end()) _aligner.setTranslationalMinEigenRatio(((*it).second)[0]);
    if((it = inputParameters.find("rotationalMinEigenRatio")) != inputParameters.end()) _aligner.setRotationalMinEigenRatio(((*it).second)[0]);
    if((it = inputParameters.find("lambda")) != inputParameters.end()) _aligner.setLambda(((*it).second)[0]);
    _aligner.setProjector(&_projector);
    _aligner.setCorrespondenceFinder(&_correspondenceFinder);
    _aligner.setLinearizer(&_linearizer);
    std::cout << "[INFO]: a  outer iterations " << _aligner.outerIterations() << std::endl;
    std::cout << "[INFO]: a  inner iterations " << _aligner.innerIterations() << std::endl;
    std::cout << "[INFO]: a  minimum inliers " << _aligner.minInliers() << std::endl;
    std::cout << "[INFO]: a  translation minimum eigen ratio " << _aligner.translationalMinEigenRatio() << std::endl;
    std::cout << "[INFO]: a  rotational minimum eigen ratio " << _aligner.rotationalMinEigenRatio() << std::endl;
    std::cout << "[INFO]: a  lambda " << _aligner.lambda() << std::endl;
    std::cout << "[INFO]: a  projector " << _aligner.projector() << std::endl;
    std::cout << "[INFO]: a  correspondence finder " << _aligner.correspondenceFinder() << std::endl;
    std::cout << "[INFO]: a  linearizer " << _aligner.linearizer() << std::endl;

    // Depth image converter
    _converter = DepthImageConverterIntegralImage(&_projector, &_statsCalculator,
                                                  &_pointInformationMatrixCalculator,
                                                  &_normalInformationMatrixCalculator);

    // Merger
    if((it = inputParameters.find("depthThreshold")) != inputParameters.end()) _merger.setMaxPointDepth(((*it).second)[0]);
    if((it = inputParameters.find("normalThreshold")) != inputParameters.end()) _merger.setNormalThreshold(((*it).second)[0]);
    if((it = inputParameters.find("distanceThreshold")) != inputParameters.end()) _merger.setDistanceThreshold(((*it).second)[0]);
    _merger.setDepthImageConverter(&_converter);
    _merger.setImageSize(424, 512);
    std::cout << "[INFO]: m  depth threshold " << _merger.maxPointDepth() << std::endl;
    std::cout << "[INFO]: m  normal threshold " << _merger.normalThreshold() << std::endl;
    std::cout << "[INFO]: m  distance threshold " << _merger.distanceThreshold() << std::endl;
    std::cout << "[INFO]: m  converter " << _merger.depthImageConverter() << std::endl;

    //My merger
    if((it = inputParameters.find("depthThreshold")) != inputParameters.end()) _mymerger.setMaxPointDepth(((*it).second)[0]);
    if((it = inputParameters.find("normalThreshold")) != inputParameters.end()) _mymerger.setNormalThreshold(((*it).second)[0]);
    if((it = inputParameters.find("distanceThreshold")) != inputParameters.end()) _mymerger.setDistanceThreshold(((*it).second)[0]);
    _mymerger.setDepthImageConverter(&_converter);
    _mymerger.setImageSize(424, 512);

}

bool NICPTrackerApp::fillInputParametersMap(map<string, std::vector<float> >& inputParameters,
                                            const string& configurationFilename) {
    ifstream is(configurationFilename.c_str());
    if(!is) {
        std::cerr << "[ERROR] impossible to open configuration file " << configurationFilename << std::endl;
        return false;
    }

    while(is.good()) {
        // Get a line from the configuration file
        char buf[1024];
        is.getline(buf, 1024);
        istringstream iss(buf);

        // Add the parameter to the map
        string parameter;
        if(!(iss >> parameter)) continue;
        if(parameter[0] == '#' || parameter[0] == '/') continue;
        float value;
        std::vector<float> values;
        while((iss >> value)) { values.push_back(value); }
        if(values.size() > 0) { inputParameters.insert(pair<string, std::vector<float> >(parameter, values)); }
    }

    return true;
}

void NICPTrackerApp::compareDepths(float& in_distance, int& in_num,
                                   float& out_distance, int& out_num,
                                   const FloatImage& depths1, const IntImage& indices1,
                                   const FloatImage& depths2, const IntImage& indices2,
                                   float dist, bool scale_z) {
    if(depths1.rows != indices1.rows ||
            depths1.cols != indices1.cols) {
        std::cerr << "[ERROR]: image1 size mismatch in compareDepths" << std::endl;
        exit(-1);
    }
    if(depths2.rows != indices2.rows ||
            depths2.cols != indices2.cols) {
        std::cerr << "[ERROR]: image2 size mismatch in compareDepths" << std::endl;
        exit(-1);
    }
    if(depths1.rows != depths2.rows ||
            depths1.cols != depths2.cols) {
        std::cerr << "[ERROR]: image1 - image2 size mismatch in compareDepths" << std::endl;
        exit(-1);
    }

    in_distance = 0;
    in_num = 0;
    out_distance = 0;
    out_num = 0;

    for(int r = 0; r < depths1.rows; ++r) {
        const float* d1_ptr = depths1.ptr<float>(r);
        const float* d2_ptr = depths2.ptr<float>(r);
        const int* i1_ptr = indices1.ptr<int>(r);
        const int* i2_ptr = indices2.ptr<int>(r);
        for(int c = 0; c<depths1.cols; ++c) {
            int i1 = *i1_ptr;
            float d1 = *d1_ptr;
            int i2 = *i2_ptr;
            float d2 = *d2_ptr;
            if(i1 > -1 && i2 > -1) {
                float avg = 0.5f * (d1 + d2);
                float d = fabs(d1 - d2);
                d = scale_z ? d / avg : d;
                if(d < dist) {
                    in_num++;
                    in_distance += d;
                }
                else {
                    out_num++;
                    out_distance += d;
                }
            }
            i1_ptr++;
            i2_ptr++;
            d1_ptr++;
            d2_ptr++;
        }
    }
}

void NICPTrackerApp::setStartingPose(const Eigen::Isometry3f startingPose){
    _globalT = startingPose;
}

double NICPTrackerApp::spinOnce(Eigen::Isometry3f& deltaT, const std::string& depthFilename, const std::string &rgbFileName) {
    // Generate new current cloud
    _tBegin = get_time();
    _rawDepth = imread(depthFilename, -1);
    _rgbImage = imread(rgbFileName, 1);
    cvtColor(_rgbImage, _rgbImage, CV_BGR2RGB);
    Mat element = getStructuringElement(MORPH_RECT, Size(5,5));
    medianBlur(_rawDepth, _rawDepth, 5);
    erode(_rawDepth, _rawDepth, element);

    if(!_rawDepth.data) {
        std::cerr << "Error: impossible to read image file " << depthFilename << std::endl;
        exit(-1);
    }
    if(_imageScaling > 1) {
        DepthImage_convert_16UC1_to_32FC1(_depth, _rawDepth, _depthScaling);
        DepthImage_scale(_scaledDepth, _depth, _imageScaling);
    }
    else {
        DepthImage_convert_16UC1_to_32FC1(_scaledDepth, _rawDepth, _depthScaling);
    }
    _scaledIndeces.create(_scaledDepth.rows, _scaledDepth.cols);
    //        _converter.compute(*_currentCloud, _scaledDepth, Eigen::Isometry3f::Identity());
    _converter.compute(*_currentCloud, _scaledDepth, _rgbImage);
    _tEnd = get_time();
    _tInput = _tEnd - _tBegin;

    //        if(_viewer) { _viewer->updateReferenceScene(_referenceScene, _globalT, true); }
    if(_viewer) { _viewer->updateReferenceScene(_finalCloud, _globalT, true); }

    // Align the new cloud
    //    _aligner.setInitialGuess(Eigen::Isometry3f::Identity());
    _aligner.setInitialGuess(deltaT);
    _aligner.setReferenceCloud(_referenceScene);
    _aligner.setCurrentCloud(_currentCloud);
    _tBegin = get_time();
    _aligner.align();
    _tEnd = get_time();
    _tAlign = _tEnd - _tBegin;

    // Update structures
    _tBegin = get_time();
    _deltaT = _aligner.T();
    _localT = _localT * _deltaT;
    _globalT = _globalT * _deltaT;
    deltaT = _deltaT;
    Eigen::Matrix3f R = _globalT.linear();
    Eigen::Matrix3f E = R.transpose() * R;
    E.diagonal().array() -= 1;
    _globalT.linear() -= 0.5 * R * E;

    _converter.projector()->project(_referenceScaledIndeces, _referenceScaledDepth, _referenceScene->points());
    //    compareDepths(_inDistance, _inNum, _outDistance, _outNum,
    //                  _referenceScaledDepth, _referenceScaledIndeces,
    //                  _scaledDepth, _scaledIndeces,
    //                  0.05f, false);

    //Look for a strange movement of camera and reset ICP
        Eigen::AngleAxisf aa(_localT.linear());
        if(_localT.translation().norm() > _breakingDistance ||
                fabs(aa.angle()) > _breakingAngle ||
                (float)_inNum / (float)(_inNum + _outNum) < _breakingInlierRatio) {
            _localT.setIdentity();
            _referenceScene = new CloudConfidence();
            if(_viewer) { _viewer->resetReferenceScene(); }
        }

    //
    _referenceScene->add(*_currentCloud, _deltaT);
    _referenceScene->transformInPlace(_deltaT.inverse());
    _merger.merge(_referenceScene);
    //    _merger.voxelize(_referenceScene, 0.005f);

    _finalCloud->add(*_currentCloud, _deltaT);
    _finalCloud->transformInPlace(_deltaT.inverse());
    _mymerger.mergeFinal(_finalCloud);
    //    cerr << "-------\n";
    //    cerr << _finalCloud->points().size() << endl;
    //    _mymerger.voxelize(_finalCloud, 0.004f);
    //    cerr << _finalCloud->points().size() << endl;
    //    _mymerger.mergeFinal(_finalCloud);

    count++;
    _seq++;
    _tEnd = get_time();
    _tUpdate = _tEnd - _tBegin;
    //    std::cout << "[INFO]: inlier ratio " << (float)_inNum / (float)(_inNum + _outNum) << std::endl;

    if(_viewer) {
        _viewer->updateCurrentCloud(_currentCloud, _globalT * _deltaT.inverse());
    }

    //    std::cout << "[INFO]: timings [input: " << _tInput << "] "
    //              << "[align: " << _tAlign << "] "
    //              << "[update: " << _tUpdate << "] "
    //              << "[total: " << _tInput + _tAlign + _tUpdate << "]" << std::endl;

    return _tAlign + _tUpdate;
}
