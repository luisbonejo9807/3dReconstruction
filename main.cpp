#include "mainwindow.h"
#include <iostream>
#include <fstream>
#include <sys/time.h>

#include <QApplication>
#include "nicptrackerappviewer.h"

using namespace std;

int main(int argc, char *argv[])
{
    //Reading arguments
    if(argc < 5) {
        std::cout << "Usage: nicp_depth_camera_tracking <configuration.txt> <associations.txt> <odometry.txt> <enable_viewer> <groundtruth.txt>" << std::endl
                  << "\tconfiguration.txt\t-->\tinput text configuration filename" << std::endl
                  << "\tassociations.txt\t-->\tfiel containing a set of depth images associations for alignment in the format: " << std::endl
                  << "\t\t\t\t\ttimestampDepthImage depthImageFilename" << std::endl
                  << "\todometry.txt\t\t-->\toutput text filename containing the computed visual odometry" << std::endl
                  << "\tenable_viewer\t\t-->\t1 if you want to enable the viewer, 0 otherwise" << std::endl
                  << "\tground_truth.txt\t-->\toptional parameter to show groundtruth trajectory" << std::endl;
        return 0;
    }

    std::string groundtruthFile = "";
    std::string configurationFile = std::string(argv[1]);
    std::string associationsFile = std::string(argv[2]);
    std::string odometryFile = std::string(argv[3]);
    bool enableViewer = atoi(argv[4]);
    std::cout << "[INFO]: configuration file " << configurationFile << std::endl;
    std::cout << "[INFO]: associations file " << associationsFile << std::endl;
    std::cout << "[INFO]: odometry file " << odometryFile << std::endl;
    if(argc == 6) {
        groundtruthFile = std::string(argv[5]);
        std::cout << "[INFO]: groundtruth file " << groundtruthFile << std::endl;
    }
    if(enableViewer) { std::cout << "[INFO]: viewer enabled " << odometryFile << std::endl; }
    else { std::cout << "[INFO]: viewer disabled " << odometryFile << std::endl; }

    ifstream is(associationsFile.c_str());
    if(!is) {
        std::cerr << "[ERROR]: impossible to open depth images associations file " << associationsFile << std::endl;
        return -1;
    }
    ofstream os(odometryFile.c_str());
    if(!os) {
        std::cerr << "[ERROR]: impossible to open odometry file " << odometryFile << std::endl;
        return -1;
    }

    // GUI Initialization
    QApplication a(argc, argv);
    MainWindow mainWindow;
    mainWindow.setWindowTitle("3d reconstruction with depth cameras");
    mainWindow.showMaximized();
    mainWindow.initConfigs(configurationFile);

    //GUI LOOP
    Eigen::Isometry3f globalT;
    globalT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    bool lastDepth = false;
    int counter = 0;
    double totTime = 0;
    while(mainWindow.isVisible()) {
        a.processEvents();
        if(mainWindow.getViewer()->needRedraw()) { mainWindow.getViewer()->updateGL(); }
        else {
            usleep(10000);
        }
        while(is.good() && (mainWindow.getViewer()->spinOnce() || mainWindow.getViewer()->spin())) {
            char buf[4096];
            is.getline(buf, 4096);
            istringstream iss(buf);
            string timestamp1, depthFilename, timestamp2, rgbFilename;
            if(!(iss >> timestamp1 >> depthFilename >> timestamp2 >> rgbFilename)) { continue; }
            if(timestamp1[0] == '#') { continue; }
            std::cout << "---------------------------------------------------------------------------- " << std::endl;
            std::cout << "[INFO]: new frame " << depthFilename << std::endl;
            double time = 0;
            Eigen::Isometry3f groundtruthT = Eigen::Isometry3f::Identity();
//            if(groundtruthFile != "") { groundtruthT = getGroundtruth(groundtruthFile, timestamp1); }
//            if(counter == 0) { tracker->setStartingPose(groundtruthT); }
//            if(enableViewer && groundtruthFile != "") { viewer->addGroundtruthPose(groundtruthT); }
            Eigen::Isometry3f deltaT = Eigen::Isometry3f::Identity();
            deltaT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
            time = mainWindow.getTracker()->spinOnce(deltaT, depthFilename, rgbFilename);
            globalT = mainWindow.getTracker()->globalT();
            std::cout << "[INFO]: delta   T " << t2v(deltaT).transpose() << std::endl;
            std::cout << "[INFO]: global  T " << t2v(globalT).transpose() << std::endl;
            std::cout << "[INFO]: ground  T " << t2v(groundtruthT).transpose() << std::endl;
            std::cout << "[INFO]: computation time " << time << " ms" << std::endl;
            Quaternionf globalRotation = Quaternionf(globalT.linear());
            globalRotation.normalize();
            os << timestamp1 << " "
               << globalT.translation().x() << " "  << globalT.translation().y() << " " << globalT.translation().z() << " "
               << globalRotation.x() << " " << globalRotation.y() << " " << globalRotation.z() << " " << globalRotation.w()
               << std::endl;
            totTime += time;
            counter++;
            a.processEvents();
            if(mainWindow.getViewer()->needRedraw()) { mainWindow.getViewer()->updateGL(); }
        }
        if(!is.good() && !lastDepth) {
            lastDepth = true;
            std::cout << "Mean time frame: " << totTime / (double)counter << std::endl;
            if(!enableViewer) { break; }
        }
    }

    return a.exec();
}
