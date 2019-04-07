/**
 * @file /include/robot_status/main_window.hpp
 *
 * @brief Qt based gui for robot_status.
 *
 * @date November 2010
 **/
#ifndef robot_status_MAIN_WINDOW_H
#define robot_status_MAIN_WINDOW_H


#include "ui_main_window.h"
#include "qnode.hpp"
#include <QMainWindow>
#include <QTimer>
namespace robot_status {

class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
    void InitUI();
	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    bool hasRoscore();

public Q_SLOTS:	
    void quit();
    void freshUI(QString value,QString type);
    void freshDevice(QNode::Device value);
    void freshHealth(QNode::Health value);
    void freshTask(QNode::Task value);
    void freshWave(QNode::Wave value);
    void freshDriver(QNode::Driver value);
    void freshLiftSensor(QNode::LiftSensor value);
    void freshRed(QNode::Red value);
    void checkROS();
private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    bool hasROS;
    QTimer *timer;
};

}  // namespace robot_status

#endif // robot_status_MAIN_WINDOW_H
