#ifndef HANDEYE_H
#define HANDEYE_H

#include <QMainWindow>
#include <QDebug>
#include <QKeyEvent>
#include <Eigen/Dense>
#include "sensor.h"
#include "imageproc.h"

QT_BEGIN_NAMESPACE
namespace Ui { class HandEye; }
QT_END_NAMESPACE

class HandEye : public QMainWindow
{
    Q_OBJECT

public:
    HandEye(QWidget *parent = nullptr);
    ~HandEye();

private slots:
    void showImage(int _width, int _height);

    void keyPressEvent(QKeyEvent *event) override;

    void on_pBtnConnect_on_connect_clicked();

    void on_pBtnConnect_on_single_clicked();

    void on_pBtnConnect_on_single_2_clicked();

    void on_pBtnConnect_on_single_3_clicked();

    void on_pBtnConnect_on_single_4_clicked();

private:

    void disableWidget();
    inline void enableWidget();
    Eigen::Matrix4d svd(std::vector<cv::Point3f> _cam_points_vec, std::vector<cv::Point3f> _base_points_vec);

    Ui::HandEye *ui;
    bool init_scan_;
    std::vector<std::vector<cv::Point3f>> cam_points_vecs_;
    std::vector<std::vector<cv::Point3f>> base_points_vecs_;

    Sensor* sensor_ptr_;
    ImageProc* image_proc_ptr_;
};
#endif // HANDEYE_H
