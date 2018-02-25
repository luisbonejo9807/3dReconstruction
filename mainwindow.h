#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "nicptrackerappviewer.h"
#include "nicptrackerapp.h"
#include <string>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);    
    ~MainWindow();
    void initConfigs(std::string configFile);
    NICPTrackerAppViewer* getViewer();
    NICPTrackerApp* getTracker();

private:
    Ui::MainWindow *ui;
    NICPTrackerApp* tracker;
    NICPTrackerAppViewer* viewer;
};

#endif // MAINWINDOW_H
