#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QJsonObject>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->textEdit->setFont(QFont("Consolas",18));
    QUrl url = QUrl("ws://192.168.31.104:9090");
    m_websocket.open(url);
    QString err=m_websocket.errorString();
    file = new QFile("/home/zzp/test.json");
    file->open(QIODevice::ReadOnly);
    if(err!="Unknown error")
        ui->label->setText(err);
    connect(&m_websocket,SIGNAL(connected()),this,SLOT(onconnected()));
    connect(&m_websocket,SIGNAL(disconnected()),this,SLOT(ondisconnected()));
//    connect(&m_websocket,SIGNAL(disconnected()),this,SLOT(closeConnection()));
}

MainWindow::~MainWindow()
{
    delete ui;
}
//{
//    id: "advertise:/cmd_vel:1"
//    latch: false
//    op: "advertise"
//    queue_size: 100
//    topic: "/cmd_vel"
//    type: "geometry_msgs/Twist"
//}

//{
//       "op":"publish",
//       "id":"publish:/cmd_vel:2",
//       "topic":"/cmd_vel",
//       "msg":{"linear":{"x":0.1,"y":0,"z":0},
//       "angular":{"x":0,"y":0,"z":0}},
//       "latch":false
// }
void MainWindow::on_pushButton_clicked()
{
    QString msg = ui->textEdit->document()->toPlainText();
    m_websocket.sendTextMessage(msg);
//    QTextStream stream(file);
//    QString data = stream.readAll();
//    qDebug()<<data;
//    m_websocket.sendTextMessage(data);
}

void MainWindow::onconnected()
{
    ui->label->setText(QStringLiteral("连接成功!"));
}

void MainWindow::ondisconnected()
{
    ui->label->setText(QStringLiteral("未连接!"));
}
