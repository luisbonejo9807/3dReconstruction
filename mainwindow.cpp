#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::initConfigs(std::string configFile)
{    
    tracker = new NICPTrackerApp(configFile, ui->viewer);
}

NICPTrackerAppViewer* MainWindow::getViewer(){
    return ui->viewer;
}

NICPTrackerApp* MainWindow::getTracker(){
    return tracker;
}
