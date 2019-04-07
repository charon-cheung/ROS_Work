#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtWebSockets/QtWebSockets>
#include <QFile>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();
public slots:
    void onconnected();
    void ondisconnected();

private:
    Ui::MainWindow *ui;
    QUrl m_url;
    QWebSocket m_websocket;
    QFile* file;
};

#endif // MAINWINDOW_H
