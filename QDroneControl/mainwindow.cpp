#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QKeyEvent>
#include <QTimer>

extern float maxEulerAngle; //degrees
extern float maxYaw; //degrees
extern int maxAltitude; //mm
extern int maxVerticalSpeed; // mm/s

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setlocale(LC_NUMERIC, "POSIX");
    if (!drone.open()) {
        qDebug() << "Failed to initialize ARDrone.";
        //return -1;
    }

    qDebug() << "Battery = " << drone.getBatteryPercentage() << "[%]";


    for(int i=0;i<8;i++) {
        isPressed[i] = false;
        lastRepeat[i] = 0;
    }

    settings = new QSettings("settings.ini", QSettings::IniFormat);
    maxEulerAngle = settings->value("Drone/MaxEulerAngle", 9.0f).toFloat();
    maxEulerAngle *= DEG_TO_RAD;
    maxYaw        = settings->value("Drone/MaxYaw", 30.0f).toFloat();
    maxYaw        *= DEG_TO_RAD;
    maxAltitude   = settings->value("Drone/MaxAltitude", 1500).toInt();
    maxVerticalSpeed = settings->value("Drone/MaxVerticalSpeed", 500).toInt();

    ui->videoImageLabel->setBackgroundRole(QPalette::Base);
    ui->videoImageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    ui->videoImageLabel->setScaledContents(true);

    videoTimer = new QTimer(this);
    connect(videoTimer, SIGNAL(timeout()), this, SLOT(updateVideo()));
    videoTimer->start(35); //35 msec ~ 30fps
}

MainWindow::~MainWindow()
{
    delete settings;
    delete videoTimer;
    delete ui;
}

cv::Mat MainWindow::getImage()
{
    QMutexLocker lck(&imageMutex);
    return drone.getImage();
}

void MainWindow::updateVideo()
{
    QMutexLocker lck(&imageMutex);

    cv::Mat image = drone.getImage();
    QImage img(image.data, image.cols, image.rows, QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(img.rgbSwapped());

    ui->videoImageLabel->setPixmap(pixmap);
}

// KB control stuff
int MainWindow::mapKey(int k)
{
    switch(k)
    {
        case Qt::Key_J: //j
            return 0;
        case Qt::Key_K: //k
            return 1;
        case Qt::Key_L: //l
            return 2;
        case Qt::Key_I: //i
            return 3;
        case Qt::Key_A: //u
            return 4;
        case Qt::Key_D: //o
            return 5;
        case Qt::Key_W: //q
            return 6;
        case Qt::Key_S: //a
            return 7;
    }
    return -1;
}

void MainWindow::keyReleaseEvent( QKeyEvent * key)
{
    int idx = mapKey(key->key());
    if(idx >= 0)
    {
        bool changed = false;
        if(!key->isAutoRepeat())	// ignore autorepeat-releases (!)
        {
            changed = isPressed[idx];
            isPressed[idx] = false;
        }

        if(changed)
        {
            ControlCommand c = calcKBControl();
            drone.setVelocity(c);
            //qDebug() << "R: " << c.roll << " " << c.pitch << " " << c.yaw << " " << c.gaz;
        }
    } else {
        key->ignore();
    }

}

void MainWindow::keyPressEvent( QKeyEvent * key)
{
    int idx = mapKey(key->key());
    if(idx >= 0)
    {
        bool changed = !isPressed[idx];

        isPressed[idx] = true;

        if(changed)
        {
            ControlCommand c = calcKBControl();
            drone.setVelocity(c);
            qDebug() << "P: " << c.roll << " " << c.pitch << " " << c.yaw << " " << c.gaz;
        }
    } else if(key->key() == Qt::Key_Space){
        if(drone.onGround()){
            drone.takeoff();
        } else {
            drone.landing();
        }
    } else if(key->key() == Qt::Key_T){ // BE CAREFUL
        drone.emergency();
    } else {
        key->ignore();
    }
}

ControlCommand MainWindow::calcKBControl()
{
    ControlCommand c;

    if(isPressed[0]) c.roll = -1; // j
    if(isPressed[1]) c.pitch = -1; // k
    if(isPressed[2]) c.roll = 1; // l
    if(isPressed[3]) c.pitch = 1; // i
    if(isPressed[4]) c.yaw = -1; // u
    if(isPressed[5]) c.yaw = 1; // o
    if(isPressed[6]) c.gaz = 1; // q
    if(isPressed[7]) c.gaz = -1; // a

    return c;
}
