#include "zmainwidget.h"

#include <QApplication>
#include <QDebug>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ZMainWidget w;
    if(!w.ZInit())
    {
        qDebug()<<"failed to Initial,exit!";
        return -1;
    }
    w.show();
    return a.exec();
}
