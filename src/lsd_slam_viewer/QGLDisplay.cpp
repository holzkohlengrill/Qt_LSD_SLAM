#include "QGLDisplay.h"

QGLDisplay::QGLDisplay(QWidget* parent)
    : QGLWidget(parent)
{
    imgReady = false;
    //img = QImage("/home/adam/1.png");
}

void QGLDisplay::setImage(const QImage& image)
{
    // Synchronization?
    mutex.lock();
    img = image;
    imgReady = true;
    mutex.unlock();

    // Force repaint
    update();
}

void QGLDisplay::paintEvent(QPaintEvent*)
{
    mutex.lock();
    if (imgReady){
        QPainter p(this);

        //Set the painter to use a smooth scaling algorithm.
        //p.SetRenderHint(QPainter::SmoothPixmapTransform, 1);

        p.drawImage(this->rect(), img);
    }
    mutex.unlock();
}
