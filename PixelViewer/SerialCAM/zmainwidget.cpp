#include "zmainwidget.h"
#include <QPainter>
#include <QDebug>
ZMainWidget::ZMainWidget(QWidget *parent)
    : QWidget(parent)
{

}
bool ZMainWidget::ZInit(void)
{
    this->m_comService=new ZComService;
    if(!this->m_comService->Open("COM5",500000))
    {
        qDebug()<<"failed to open";
        return false;
    }
    this->m_thread=new QThread;
    this->m_comService->moveToThread(this->m_thread);
    this->m_thread->start();
    QObject::connect(this->m_comService,SIGNAL(ZSigJpegData(QByteArray)),this,SLOT(ZSlotJpegData(QByteArray)));
    QObject::connect(this->m_comService,SIGNAL(ZSigHTData(QByteArray)),this,SLOT(ZSlotHTData(QByteArray)));
    return true;
}
ZMainWidget::~ZMainWidget()
{

}

void ZMainWidget::refresh(void)
{

}
void ZMainWidget::ZSlotJpegData(QByteArray ba)
{
    qDebug()<<"SlotJpeg";
}
void ZMainWidget::ZSlotHTData(QByteArray ba)
{
qDebug()<<"SlotHT";
}
void ZMainWidget::paintEvent(QPaintEvent *e)
{
    Q_UNUSED(e);
    QPixmap img;
    if(img.loadFromData(this->m_baJpeg))
    {
        QPainter p(this);
        p.drawPixmap(this->rect(),img);
    }
}
