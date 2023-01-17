#ifndef ZCOMSERVICE_H
#define ZCOMSERVICE_H

#include <QObject>
#include <QSerialPort>
class ZComService : public QObject
{
    Q_OBJECT
public:
    enum{
        FSM_JPEG_SIZE=0,
        FSM_JPEG_DATA=1,
        FSM_HT_SIZE=2,
        FSM_HT_DATA=3,
    };
    explicit ZComService(QObject *parent = nullptr);
    virtual ~ZComService(void);
public slots:
    bool Open(const QString &, const int);
    void Close(void);

    void ZSlotRxData(void);
    void ZSlotTxData(QByteArray ba);
signals:
    void ZSigJpegData(QByteArray ba);
    void ZSigHTData(QByteArray ba);
private:
    QSerialPort *m_serialPort;
private:
    int m_fsm;
private:
    char *m_buffer;
    int m_bufLen;
    int m_rdbytes;

    QByteArray m_baBuffer;
    int m_baSize;
};

#endif // ZCOMSERVICE_H
