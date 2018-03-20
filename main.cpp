#include "mainwindow.h"
#include <iostream>
#include <fstream>
#include <sys/time.h>

#include <QApplication>
#include "nicptrackerappviewer.h"
#include "filereading.h"

using namespace std;

int main(int argc, char *argv[])
{
    //Reading arguments
    if(argc < 4) {
        std::cout << "Usage: 3d reconstruction <configuration.txt> <associations.txt> <odometry.txt> <groundtruth.txt>" << std::endl
                  << "\tconfiguration.txt\t-->\tinput text configuration filename" << std::endl
                  << "\tassociations.txt\t-->\tfiel containing a set of depth images associations for alignment in the format: " << std::endl
                  << "\t\t\t\t\ttimestampDepthImage depthImageFilename" << std::endl
                  << "\todometry.txt\t\t-->\toutput text filename containing the computed visual odometry" << std::endl
                  << "\tground_truth.txt\t-->\toptional parameter to show groundtruth trajectory" << std::endl;
        return 0;
    }

    std::string groundtruthFile = "";
    std::string configurationFile = std::string(argv[1]);
    std::string associationsFile = std::string(argv[2]);
    std::string odometryFile = std::string(argv[3]);    
    std::cout << "[INFO]: configuration file " << configurationFile << std::endl;
    std::cout << "[INFO]: odometry file " << odometryFile << std::endl;
    if(argc == 5) {
        groundtruthFile = std::string(argv[4]);
        std::cout << "[INFO]: groundtruth file " << groundtruthFile << std::endl;
    }    

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
    //Reading INPUT
    vector<string> filesDepth, filesRGB;
    readFilenames(filesDepth, "./depth");
    readFilenames(filesRGB, "./rgb");

    // GUI Initialization
    QApplication app(argc, argv);
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

    int frame = 0;
    while(mainWindow.isVisible()) {
        app.processEvents();
        if(mainWindow.getViewer()->needRedraw()){
            mainWindow.getViewer()->updateGL();
        }
        else {
            usleep(1000);
        }
        while(mainWindow.isVisible() && (mainWindow.getViewer()->spinOnce() || mainWindow.getViewer()->spin()) || frame >= filesDepth.size()) {
            string depthFilename, rgbFilename;
            depthFilename = "depth/" + filesDepth[frame];
            rgbFilename = "rgb/"+ filesRGB[frame];
            frame += 1;
            double time = 0;
            Eigen::Isometry3f groundtruthT = Eigen::Isometry3f::Identity();
            Eigen::Isometry3f deltaT = Eigen::Isometry3f::Identity();
            time = mainWindow.getTracker()->spinOnce(deltaT, depthFilename, rgbFilename);
            app.processEvents();
            if(mainWindow.getViewer()->needRedraw()) {
                mainWindow.getViewer()->updateGL();
            }
        }
    }
    return 0;
}
