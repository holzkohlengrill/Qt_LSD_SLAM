#include "keypresshandler.h"
#include "lsd_slam_viewer/PointCloudViewer.h"

#include <QEvent>
#include <QKeyEvent>

KeyPressHandler::KeyPressHandler(QObject *parent) :
  QObject(parent)
{
}

bool KeyPressHandler::eventFilter(QObject *obj, QEvent *event)
{
  if (event->type() == QEvent::KeyPress) {
      QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
      qDebug("Handled key press %d", keyEvent->key());
      if (keyEvent->key()==Qt::Key_B){
          *start = true;
        }else if (keyEvent->key()==Qt::Key_R){
          *reset = true;
        } else  if(keyEvent->key() == Qt::Key_P){
          //PointCloudViewer.
        }
      return false;
    } else {
      // standard event processing
      //return QObject::eventFilter(obj, event);
      return false;
    }
  event->ignore();
}

void KeyPressHandler::setStartVar(bool *b){
  start = b;
}

void KeyPressHandler::setResetVar(bool *b){
  reset = b;
}
