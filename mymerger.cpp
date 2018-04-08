#include "mymerger.h"

MyMerger::MyMerger() {
    _distanceThreshold = 0.1f;
    _normalThreshold = cosf(10 * M_PI / 180.0f);
    _maxPointDepth = 10.0f;
    _depthImageConverter = 0;
    _indexImage.create(0, 0);
    _depthImage.create(0, 0);
    _collapsedIndices.resize(0);
}

void MyMerger::merge(CloudConfidence *cloud, Eigen::Isometry3f transform) {
    assert(_indexImage.rows > 0 && _indexImage.cols > 0 && "Merger: _indexImage has zero size");
    assert(_depthImageConverter  && "Merger: missing _depthImageConverter");
    assert(_depthImageConverter->projector()  && "Merger: missing projector in _depthImageConverter");

    PointProjector *pointProjector = _depthImageConverter->projector();
    Eigen::Isometry3f oldTransform = pointProjector->transform();

    pointProjector->setTransform(transform);
    pointProjector->project(_indexImage,
                            _depthImage,
                            cloud->points());
    int target = 0;
    int distance = 0;
    _collapsedIndices.resize(cloud->points().size());
    std::fill(_collapsedIndices.begin(), _collapsedIndices.end(), -1);

    int killed = 0;
    int normalkilled = 0;
    int elses = 0;
    int currentIndex = 0;
    for(size_t i = 0; i < cloud->points().size(); currentIndex++ ,i++) {
        const Point currentPoint = cloud->points()[i];
        const Normal currentNormal = cloud->normals()[i];
        int r = -1, c = -1;
        float depth = 0.0f;
        pointProjector->project(c, r, depth, currentPoint);
        if(depth < 0 || /*depth < _maxPointDepth ||*/
                r < 0 || r >= _depthImage.rows ||
                c < 0 || c >= _depthImage.cols) {
            //distance++;
            continue;
        }
        float &targetZ = _depthImage(r, c);
        int targetIndex = _indexImage(r, c);
        if(targetIndex < 0) {
            target++;
            continue;
        }
        const Normal &targetNormal = cloud->normals().at(targetIndex);
        Eigen::Vector4f viewPointDirection = transform.matrix().col(3)-currentPoint;
        viewPointDirection(3)=0;
        viewPointDirection.normalize();

        if(targetIndex == currentIndex) {
            _collapsedIndices[currentIndex] = currentIndex;
            killed++;
        }
        if(fabs(depth - targetZ) < 0.002f
                /*&& acos(currentNormal.dot(targetNormal)) < 0.2f*/
                /*&& viewPointDirection.dot(targetNormal) > 0.9f*/) {
            Gaussian3f &targetGaussian = cloud->gaussians()[targetIndex];
            Gaussian3f &currentGaussian = cloud->gaussians()[currentIndex];
            targetGaussian.addInformation(currentGaussian);

            //increase confidence of point
            cloud->_pointsMatchingCounter[i] += 1;
            double ni = cloud->_pointsMatchingCounter[i];

            //calculate mean of points that project onto same pixel and has close depth
            cloud->points()[i].head<3>() = (ni * cloud->points()[i].head<3>() + cloud->points()[targetIndex].head<3>()) / (ni + 1);
            _collapsedIndices[currentIndex] = targetIndex;
            killed++;
        }
        else{
            elses++;
        }
    }

    int murdered = 0;
    int k = 0;
    for(size_t i = 0; i < _collapsedIndices.size(); i++) {
        int collapsedIndex = _collapsedIndices[i];
        if(collapsedIndex == (int)i) {
            cloud->points()[i].head<3>() = cloud->gaussians()[i].mean();
        }
        if(collapsedIndex < 0 || collapsedIndex == (int)i) {
            cloud->points()[k] = cloud->points()[i];
            cloud->normals()[k] = cloud->normals()[i];
            cloud->stats()[k] = cloud->stats()[i];
            cloud->pointInformationMatrix()[k] = cloud->pointInformationMatrix()[i];
            cloud->normalInformationMatrix()[k] = cloud->normalInformationMatrix()[i];
            cloud->gaussians()[k] = cloud->gaussians()[i];
            if(cloud->rgbs().size()){
                cloud->rgbs()[k]= cloud->rgbs()[i];
            }
            k++;
        }
        else {
            murdered ++;
        }
    }
    int originalSize = cloud->points().size();

    // Kill the leftover points
    cloud->points().resize(k);
    cloud->normals().resize(k);
    cloud->stats().resize(k);
    cloud->pointInformationMatrix().resize(k);
    cloud->normalInformationMatrix().resize(k);
    if(cloud->rgbs().size())
        cloud->rgbs().resize(k);
    std::cout << "[INFO]: number of suppressed points " << murdered << std::endl;
    std::cout << "[INFO]: number of killed points " << killed << std::endl;
    std::cout << "[INFO]: number of normal killed points " << normalkilled << std::endl;
    std::cout << "[INFO]: number of elses points " << elses << std::endl;
    std::cout << "[INFO]: resized cloud from " << originalSize << " to " << k << " points" <<std::endl;

    pointProjector->setTransform(oldTransform);
}

void MyMerger::mergeFinal(CloudConfidence *cloud, Eigen::Isometry3f transform) {
    assert(_indexImage.rows > 0 && _indexImage.cols > 0 && "Merger: _indexImage has zero size");
    assert(_depthImageConverter  && "Merger: missing _depthImageConverter");
    assert(_depthImageConverter->projector()  && "Merger: missing projector in _depthImageConverter");

    PointProjector *pointProjector = _depthImageConverter->projector();
    Eigen::Isometry3f oldTransform = pointProjector->transform();

    pointProjector->setTransform(transform);
    pointProjector->project(_indexImage,
                            _depthImage,
                            cloud->points());
    int confThreshold = 10;
    int target = 0;
    int distance = 0;
    _collapsedIndices.resize(cloud->points().size());
    std::fill(_collapsedIndices.begin(), _collapsedIndices.end(), -1);

    int killed = 0;    
    int elses = 0;
    int currentIndex = 0;
    int jumped = 0;
    for(size_t i = 0; i < cloud->points().size(); currentIndex++ ,i++) {

        //age of point increased
        cloud->_age[i] += 1;
        //        cloud->rgbs()[i][0] = cloud->_age[i] * 255/(confThreshold + 0) > 255 ? 255: cloud->_age[i] * 255/(confThreshold + 0);
        //        cloud->rgbs()[i][1] = 0;
        //        cloud->rgbs()[i][2] = 0;
//        cloud->rgbs()[i][0] = 200;
//        cloud->rgbs()[i][1] = 200;
//        cloud->rgbs()[i][2] = 200;

//        if(cloud->_pointsMatchingCounter[i] >= confThreshold){
//            jumped++;
//            continue;
//        }
        if(cloud->_age[i] > confThreshold + 5 && cloud->_pointsMatchingCounter[i] < confThreshold){
            _collapsedIndices[i] = 999999999;
            continue;
        }

        const Point currentPoint = cloud->points()[i];
        const Normal currentNormal = cloud->normals()[i];
        int r = -1, c = -1;
        float depth = 0.0f;
        pointProjector->project(c, r, depth, currentPoint);
        if(depth < 0 || r < 0 || r >= _depthImage.rows ||
                c < 0 || c >= _depthImage.cols) {
            continue;
        }
        float &targetZ = _depthImage(r, c);
        int targetIndex = _indexImage(r, c);
        if(targetIndex < 0) {
            target++;
            continue;
        }
        const Normal &targetNormal = cloud->normals().at(targetIndex);
        Eigen::Vector4f viewPointDirection = transform.matrix().col(3)-currentPoint;
        viewPointDirection(3)=0;
        viewPointDirection.normalize();

        if(fabs(depth - targetZ) <= 1.5f
                /*&& acos(currentNormal.dot(targetNormal)) < 0.2f*/
                /*&& fabs(acos(viewPointDirection.dot(targetNormal)) - 1) < 0.05f*/) {
            Gaussian3f &targetGaussian = cloud->gaussians()[targetIndex];
            Gaussian3f &currentGaussian = cloud->gaussians()[currentIndex];
            targetGaussian.addInformation(currentGaussian);

            //increase confidence of point
            cloud->_pointsMatchingCounter[i] += 1;
            cloud->_pointsMatchingCounter[targetIndex] += 1;
            double ni = cloud->_pointsMatchingCounter[i];

            //if(cloud->_pointsMatchingCounter[i] <= confThreshold){
                double factor = 1/fabs(acos(viewPointDirection.dot(targetNormal)));
                //calculate mean of points that project onto same pixel and has close depth
                cloud->points()[i].head<3>() = (ni * cloud->points()[i].head<3>() + factor*cloud->points()[targetIndex].head<3>()) / (ni + factor);
                cloud->rgbs()[i][0] = (ni * cloud->rgbs()[i][0] + cloud->rgbs()[targetIndex][0]) / (ni + 1);
                cloud->rgbs()[i][1] = (ni * cloud->rgbs()[i][1] + cloud->rgbs()[targetIndex][1]) / (ni + 1);
                cloud->rgbs()[i][2] = (ni * cloud->rgbs()[i][2] + cloud->rgbs()[targetIndex][2]) / (ni + 1);
                _collapsedIndices[currentIndex] = targetIndex;
                killed++;
//            }
        }
        else{
            elses++;
        }
    }

    int murdered = 0;
    int k = 0;
    for(size_t i = 0; i < _collapsedIndices.size(); i++) {
        int collapsedIndex = _collapsedIndices[i];
        if(collapsedIndex == (int)i) {
            cloud->points()[i].head<3>() = cloud->gaussians()[i].mean();
        }
        if(collapsedIndex < 0 || collapsedIndex == (int)i) {
            cloud->points()[k] = cloud->points()[i];
            cloud->normals()[k] = cloud->normals()[i];
            cloud->stats()[k] = cloud->stats()[i];
            cloud->pointInformationMatrix()[k] = cloud->pointInformationMatrix()[i];
            cloud->normalInformationMatrix()[k] = cloud->normalInformationMatrix()[i];
            cloud->gaussians()[k] = cloud->gaussians()[i];
            if(cloud->rgbs().size()){
                cloud->rgbs()[k]= cloud->rgbs()[i];
            }
            k++;
        }
        else {
            murdered ++;
        }
    }
    int originalSize = cloud->points().size();

    // Kill the leftover points
    cloud->points().resize(k);
    cloud->normals().resize(k);
    cloud->stats().resize(k);
    cloud->pointInformationMatrix().resize(k);
    cloud->normalInformationMatrix().resize(k);
    if(cloud->rgbs().size())
        cloud->rgbs().resize(k);

    std::cerr << "[INFO]: number of jumped points " << jumped << std::endl;
    std::cerr << "[INFO]: number of suppressed points " << murdered << std::endl;
    std::cerr << "[INFO]: number of killed points " << killed << std::endl;
    std::cerr << "[INFO]: number of elses points " << elses << std::endl;
    std::cerr << "[INFO]: resized cloud from " << originalSize << " to " << k << " points" <<std::endl;

    pointProjector->setTransform(oldTransform);
}

void MyMerger::voxelize(Cloud* model, float res) {
    float ires = 1.0f / res;
    std::vector<IndexTriplet> voxels(model->size());
    for(int i = 0; i < (int)model->size(); i++) { voxels[i] = IndexTriplet(model->points()[i], i , ires); }
    Cloud sparseModel;
    sparseModel.resize(model->size());
    sparseModel.rgbs().resize(model->size());
    std::sort(voxels.begin(), voxels.end());
    int k = -1;
    std::vector<StatsAccumulator> statsAccs;
    statsAccs.resize(model->size());
    std::fill(statsAccs.begin(), statsAccs.begin(), StatsAccumulator());
    for(size_t i = 0; i < voxels.size(); i++) {
        IndexTriplet& triplet = voxels[i];
        int idx = triplet.index;
        StatsAccumulator& statsAcc = statsAccs[i];
        if(k >= 0 && voxels[i].sameCell(voxels[i-1])) {
            Eigen::Matrix3f U = model->stats()[idx].eigenVectors();
            Eigen::Vector3f lambdas = model->stats()[idx].eigenValues();
            Eigen::Matrix3f sigma = U * Diagonal3f(lambdas.x(), lambdas.y(), lambdas.z()) * U.inverse();
            Eigen::Matrix3f omega = sigma.inverse();
            statsAcc.omega += omega;
            statsAcc.u += omega * model->stats()[idx].mean().head<3>();
            statsAcc.acc++;
        }
        else {
            k++;
            Eigen::Matrix3f U = model->stats()[idx].eigenVectors();
            Eigen::Vector3f lambdas = model->stats()[idx].eigenValues();
            Eigen::Matrix3f sigma = U * Diagonal3f(lambdas.x(), lambdas.y(), lambdas.z()) * U.inverse();
            Eigen::Matrix3f omega = sigma.inverse();
            statsAcc.omega += omega;
            statsAcc.u += omega * model->stats()[idx].mean().head<3>();
            statsAcc.acc++;
            sparseModel.points()[k] = model->points()[idx];
            sparseModel.normals()[k] = model->normals()[idx];
            if(model->rgbs().size())
                sparseModel.rgbs()[k] = model->rgbs()[idx];
            sparseModel.stats()[k] = model->stats()[idx];
            sparseModel.pointInformationMatrix()[k] = model->pointInformationMatrix()[idx];
            sparseModel.normalInformationMatrix()[k] = model->normalInformationMatrix()[idx];
            sparseModel.gaussians()[k] = model->gaussians()[idx];
        }
    }
    sparseModel.resize(k);
    statsAccs.resize(k);
    *model = sparseModel;

    InformationMatrix _flatPointInformationMatrix = InformationMatrix::Zero();
    InformationMatrix _nonFlatPointInformationMatrix = InformationMatrix::Zero();
    InformationMatrix _flatNormalInformationMatrix = InformationMatrix::Zero();
    InformationMatrix _nonFlatNormalInformationMatrix = InformationMatrix::Zero();
    _flatPointInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1000.0f, 0.001f, 0.001f));
    _nonFlatPointInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1.0f, 0.1f, 0.1f));
    _flatNormalInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1000.0f, 0.001f, 0.001f));
    _nonFlatNormalInformationMatrix.diagonal() = Normal(Eigen::Vector3f(0.1f, 1.0f, 1.0f));
    for(size_t i = 0; i < model->size(); i++) {
        StatsAccumulator& statsAcc = statsAccs[i];
        if(statsAcc.acc > 1) {
            statsAcc.u = statsAcc.omega * statsAcc.u;
            Point& p = model->points()[i];
            p = statsAcc.u;

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver;
            eigenSolver.computeDirect(statsAcc.omega.inverse(), Eigen::ComputeEigenvectors);
            Stats& stats = model->stats()[i];
            stats.setZero();
            stats.setEigenVectors(eigenSolver.eigenvectors());
            stats.setMean(statsAcc.u);
            InformationMatrix U = Eigen::Matrix4f::Zero();
            U.block<3, 3>(0, 0)  = eigenSolver.eigenvectors();
            Eigen::Vector3f eigenValues = eigenSolver.eigenvalues();
            if(eigenValues(0) < 0.0f) { eigenValues(0) = 0.0f; }
            stats.setEigenValues(eigenValues);
            stats.setN(statsAcc.acc);

            Normal& n = model->normals()[i];
            n = stats.block<4, 1>(0, 0);
            if(stats.curvature() < 0.3f) {
                if(n.dot(p) > 0) { n = -n; }
            }
            else { n.setZero(); }

            if(stats.curvature() < 0.1f) {
                model->pointInformationMatrix()[i] = U * _flatPointInformationMatrix * U.transpose();
                model->normalInformationMatrix()[i] = U * _flatNormalInformationMatrix * U.transpose();
            }
            else {
                model->pointInformationMatrix()[i] = U * _nonFlatPointInformationMatrix * U.transpose();
                model->normalInformationMatrix()[i] = U * _nonFlatNormalInformationMatrix * U.transpose();
            }
        }
    }
}
