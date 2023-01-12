#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QImage>
#include <QPainter>

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

    bool loadRAWData();
protected:
    void paintEvent(QPaintEvent *event);

private:
    QImage m_img00;
    QImage m_img01;
    QImage m_img10;
    QImage m_img11;
};
#endif // WIDGET_H
