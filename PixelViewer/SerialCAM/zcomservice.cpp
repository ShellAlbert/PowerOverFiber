#include "zcomservice.h"
#include <QDebug>
#define BUFFER_SIZE (128*1024)
ZComService::ZComService(QObject *parent)
    : QObject(parent)
{
    this->m_serialPort=nullptr;
    this->m_fsm=FSM_JPEG_SIZE;

    this->m_buffer=new char[BUFFER_SIZE];//128KBytes.
    this->m_bufLen=0;
    this->m_rdbytes=0;

    this->m_baBuffer.resize(128*1024);//128KBytes.
}
ZComService::~ZComService(void)
{
    delete [] this->m_buffer;
    this->m_baBuffer.squeeze();

    if(nullptr!=this->m_serialPort)
    {
        if(this->m_serialPort->isOpen())
        {
            this->m_serialPort->clear();
            this->m_serialPort->close();
            this->m_serialPort->deleteLater();
        }
        this->m_serialPort=nullptr;
    }
}
bool ZComService::Open(const QString &strPortName, const int nBaudRate)
{
    if(strPortName.isEmpty())
    {
        return false;
    }
    this->m_serialPort=new QSerialPort;
    QObject::connect(this->m_serialPort,&QSerialPort::readyRead,this,&ZComService::ZSlotRxData);
    this->m_serialPort->setPortName(strPortName);
    this->m_serialPort->setBaudRate(nBaudRate);
    this->m_serialPort->setDataBits(QSerialPort::Data8);
    this->m_serialPort->setStopBits(QSerialPort::OneStop);
    this->m_serialPort->setParity(QSerialPort::NoParity);
    this->m_serialPort->setFlowControl(QSerialPort::NoFlowControl);
    if(!this->m_serialPort->open(QIODevice::ReadWrite))
    {
        this->m_serialPort->close();
        this->m_serialPort->deleteLater();
        this->m_serialPort=nullptr;
        return false;
    }
    return true;
}
void ZComService::Close(void)
{
    if(nullptr!=this->m_serialPort)
    {
        if(this->m_serialPort->isOpen())
        {
            this->m_serialPort->clear();
            this->m_serialPort->close();
            this->m_serialPort->deleteLater();
        }
        this->m_serialPort=nullptr;
    }
}

void ZComService::ZSlotRxData(void)
{
    if(nullptr==this->m_serialPort)
    {
        return;
    }
    if(!this->m_serialPort->isOpen())
    {
        return;
    }
    QByteArray ba=this->m_serialPort->readAll();
    if(this->m_bufLen+ba.size()<BUFFER_SIZE)
    {
        memcpy(this->m_buffer+this->m_bufLen,ba.constData(),ba.size());
        this->m_bufLen+=ba.size();
        qDebug()<<"bufLen:"<<this->m_bufLen;
    }else
    {
        //overflow,maybe error,reset all.
        this->m_bufLen=0;
        return;
    }

    //scan buffer to parse one complete frame.
    if(this->m_bufLen<4)
    {
        //no enough data to parse out JpegSize, 4 bytes needed at least.
        return;
    }
    int sizeJpeg=this->m_buffer[3]<<24|this->m_buffer[2]<<16|this->m_buffer[1]<<8|this->m_buffer[0];
    qDebug()<<"sizeJpeg:"<<sizeJpeg;
    if(sizeJpeg<=0 || sizeJpeg>BUFFER_SIZE)
    {
        qDebug()<<"sizeJpeg error!";
        this->m_bufLen=0; //maybe error, reset all.
        return;
    }

    //HT data size.
    if(this->m_bufLen<(4+sizeJpeg+4))
    {
        //no enough data to parse out HTSize, (4+sizeJpeg+4) bytes needed at least.
        return;
    }
    int sizeHT=this->m_buffer[4+sizeJpeg+3]<<24|this->m_buffer[4+sizeJpeg+2]<<16|this->m_buffer[4+sizeJpeg+1]<<8|this->m_baBuffer[4+sizeJpeg+0];
    qDebug()<<"sizeHT:"<<sizeHT;
    if(sizeHT!=4)
    {
        qDebug()<<"sizeHT error!";
        this->m_bufLen=0; //maybe error, reset all.
        return;
    }
    if(this->m_bufLen<(4+sizeJpeg+4+sizeHT))
    {
        //no enough data to parse out HTData, (4+sizeJpeg+4+sizeHT) bytes needed at least.
        return;
    }

    //now, we have enough data to parse out one complete frame.
    //JpegSize(4 bytes), JpegData(N bytes), HTSize(4 bytes), HTData(N bytes).
    QByteArray baJpeg;
    baJpeg.resize(sizeJpeg);
    memcpy(baJpeg.data(),&this->m_buffer[4],sizeJpeg);
    emit this->ZSigJpegData(baJpeg);

    QByteArray baHT;
    baHT.resize(sizeHT);
    memcpy(baHT.data(),&this->m_buffer[4+sizeJpeg+4],sizeHT);
    emit this->ZSigHTData(baHT);

    //move the remain data to the head position of buffer.
    if((this->m_bufLen-(4+sizeJpeg+4+sizeHT))>0)
    {
        memmove(this->m_buffer, &this->m_buffer[4+sizeJpeg+4+sizeHT], this->m_bufLen-(4+sizeJpeg+4+sizeHT));
        this->m_bufLen=this->m_bufLen-(4+sizeJpeg+4+sizeHT);
        qDebug()<<"remain:"<<this->m_bufLen-(4+sizeJpeg+4+sizeHT);
    }
}
void ZComService::ZSlotTxData(QByteArray ba)
{
    if(nullptr==this->m_serialPort)
    {
        return;
    }
    if(!this->m_serialPort->isOpen())
    {
        return;
    }
    this->m_serialPort->write(ba,ba.size());
    return;
}
