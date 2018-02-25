#include "mainwindow.h"
#include <QApplication>
#include "nicptrackerappviewer.h"

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
    NICPTrackerAppViewer* viewer = new NICPTrackerAppViewer(mainWindow);


    return a.exec();
}
