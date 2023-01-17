#ifndef ZMAINWIDGET_H
#define ZMAINWIDGET_H

#include <QWidget>
#include <QThread>
#include <QTimer>
#include "zcomservice.h"

class ZMainWidget : public QWidget
{
    Q_OBJECT

public:
    ZMainWidget(QWidget *parent = nullptr);
    ~ZMainWidget();

    void refresh(void);
    bool ZInit(void);
public slots:
    void ZSlotJpegData(QByteArray ba);
    void ZSlotHTData(QByteArray ba);
protected:
    void paintEvent(QPaintEvent *e);

private:
    QThread *m_thread;
    ZComService *m_comService;
    QByteArray m_baJpeg;
};
#endif // ZMAINWIDGET_H
