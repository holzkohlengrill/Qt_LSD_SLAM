#ifndef KEYPRESSHANDLER_H
#define KEYPRESSHANDLER_H

#include <QObject>

class KeyPressHandler : public QObject
{
    Q_OBJECT
public:
    explicit KeyPressHandler(QObject *parent = 0);    
    void setStartVar(bool *b);
    void setResetVar(bool *b);
protected:
    bool eventFilter(QObject *obj, QEvent *event);
signals:

public slots:

private:
    bool *start;
    bool *reset;
};

#endif // KEYPRESSHANDLER_H
