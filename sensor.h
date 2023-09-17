#ifndef SENSOR_H
#define SENSOR_H

#include <QObject>
#include <SR7Link.h>
#include <fstream>
#include <QFileDialog>
#include <QTextEdit>
#include <QTime>
#include <QCoreApplication>
#include <QDebug>
#include <memory>
#include "CallOneTimes.h"

class Sensor : public QObject
{
    Q_OBJECT
public:
    explicit Sensor(QObject *parent = nullptr);
    ~Sensor();
    bool ethenetConnect();

    bool initBatch();

    bool singleBatch();

    void InitCallOneTimesBeforeDisConnect();

    void InitConfigBeforeDisConnect();

    void ethenetDisconnect();

    QImage BatchDataShow(int *_BatchData,
                       double max_height,
                       double min_height,
                       int _ColorMax ,
                       int img_w,
                       int img_h,
                       int _xscale,
                       int _yscale,
                       int _scaleW,
                       int _scaleH );
    QImage GrayDataShow(unsigned char* _grayData,
                         int img_w,
                         int img_h,
                         int _xscale,
                         int _yscale ,
                         int _scaleW,
                         int _scaleH);

    double dheight_upper_;
    double dheight_lower_;
    double fscale_;
    QImage height_image_;
    CallOneTimes* call_one_times_ptr_;

private:

    int getEncoderParameters();

    bool setEncoderParameters();

    void getHeightUpperLower(double& _upper, double& _lower);

    void connectFunc();

    int device_id_;
    std::shared_ptr<unsigned char> gray_batch_ptr_;
    std::shared_ptr<unsigned char> height_batch_ptr_;

signals:

};

#endif // SENSOR_H
