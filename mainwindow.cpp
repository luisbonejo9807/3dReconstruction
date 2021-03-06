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
    tracker = new NICPTrackerApp(configFile, ui->viewer, ui->viewDepth, ui->viewRGB, ui->viewProjection);
}

NICPTrackerAppViewer* MainWindow::getViewer(){
    return ui->viewer;
}

NICPTrackerApp* MainWindow::getTracker(){
    return tracker;
}

void MainWindow::closeEvent(QCloseEvent *ev){
    ev->accept();
}

void MainWindow::on_buttonSaveCloud_clicked()
{

}
