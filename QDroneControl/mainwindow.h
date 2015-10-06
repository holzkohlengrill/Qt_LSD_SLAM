#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSettings>
#include <QMutex>

#include <3rdParty/cvdrone/src/ardrone.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    cv::Mat getImage();

private:
    Ui::MainWindow *ui;

public slots:
    void updateVideo();

protected:
    void keyPressEvent( QKeyEvent * key);
    void keyReleaseEvent( QKeyEvent * key);
    int mapKey(int k);
    ControlCommand calcKBControl();
    bool isPressed[8];	//{j k l i u o q a}
    unsigned int lastRepeat[8];

    ARDrone drone;
    QTimer *videoTimer;

    QSettings *settings;

    //QImage* videoImage;
    QMutex imageMutex;
};

#endif // MAINWINDOW_H
