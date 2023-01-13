#include "zwidget.h"
#include <QFile>
#include <iostream>
#include <QDebug>
Widget::Widget(QWidget *parent)
    : QWidget(parent)
{

}

Widget::~Widget()
{

}
bool Widget::loadRAWData()
{
    //640*512*16-bits
    //320*256*16-bits/8-bits=163840bytes.
    char buffer[163840];
    //load (0,0) 1/4 part.
    FILE *pFile=fopen("C:\\Users\\Administrator\\Desktop\\APP\\sscom5.13.1\\iRay.DAT","rb");
    if(pFile==NULL)
    {
        qDebug()<<"open file error!";
        return false;
    }
    for(qint32 i=0;i<1;i++)
    {
        if(fread(buffer,sizeof(buffer),1,pFile)!=1)
        {
            qDebug()<<"fread() error!";
            return false;
        }
        switch(i)
        {
        case 0:
            this->m_img00=QImage(320,256,QImage::Format_Grayscale16);
            for(int i=0;i<256;i++)
            {
                quint16 *dst=(quint16*)(this->m_img00.bits()+i*this->m_img00.bytesPerLine());
                qint32 offset=0;
                for(int j=0;j<320;j++)
                {
                    qint8 iHighByte= buffer[offset+1+i*320*2];
                    qint8 iLowByte = buffer[offset+0+i*320*2];
                    offset+=2;
                    qint16 iData=0x0ed1;
                    quint16 pixelData=(iHighByte<<8) | iLowByte;
                    dst[j]= 0x0;//pixelData;
                }
            }
            break;
        case 1:
            this->m_img01=QImage(320,256,QImage::Format_Grayscale16);
            for(int i=0;i<256;i++)
            {
                quint16 *dst=(quint16*)(this->m_img01.bits()+i*this->m_img01.bytesPerLine());
                for(int j=0;j<320;j++)
                {
                    dst[j]=(buffer[j+i*320]<<8)|(buffer[j+i*320+1]);
                }
            }
            break;
        case 2:
            this->m_img10=QImage(320,256,QImage::Format_Grayscale16);
            for(int i=0;i<256;i++)
            {
                quint16 *dst=(quint16*)(this->m_img10.bits()+i*this->m_img10.bytesPerLine());
                for(int j=0;j<320;j++)
                {
                    dst[j]=(buffer[j+i*320]<<8)|(buffer[j+i*320+1]);
                }
            }
            break;
        case 3:
            this->m_img11=QImage(320,256,QImage::Format_Grayscale16);
            for(int i=0;i<256;i++)
            {
                quint16 *dst=(quint16*)(this->m_img11.bits()+i*this->m_img11.bytesPerLine());
                for(int j=0;j<320;j++)
                {
                    dst[j]=(buffer[j+i*320]<<8)|(buffer[j+i*320+1]);
                }
            }
            break;
        default:
            break;
        }
    }
    fclose(pFile);
    return true;
}
void Widget::paintEvent(QPaintEvent *event)
{
    QPainter p(this);
    p.drawImage(0,0,this->m_img00);
    p.drawImage(320,0,this->m_img01);
    p.drawImage(0,256,this->m_img10);
    p.drawImage(320,256,this->m_img11);
}
