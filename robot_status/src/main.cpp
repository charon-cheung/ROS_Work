/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/robot_status/main_window.hpp"

int main(int argc, char **argv) {

    QApplication app(argc, argv);
    ros::init(argc,argv,"robot_status");
    robot_status::MainWindow w(argc,argv);
    if(w.hasRoscore())
        w.show();
    else
        return -1;
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();
    return result;
}
