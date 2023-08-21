#ifndef HANDEYE_H
#define HANDEYE_H

#include <QMainWindow>
#include <QDebug>
#include <QKeyEvent>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <Eigen/Dense>
#include <QMessageBox>
#include <QCloseEvent>
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

    void on_pBtnConnect_on_single_3_clicked();

    void on_pBtnConnect_on_single_4_clicked();

    void on_pBtnConnect_on_connect_2_clicked();

    void on_lineEdit_textEdited(const QString &arg1);

    void on_lineEdit_2_textEdited(const QString &arg1);

    void on_lineEdit_3_textEdited(const QString &arg1);

    void on_lineEdit_4_textEdited(const QString &arg1);

    void on_lineEdit_5_textEdited(const QString &arg1);

    void on_lineEdit_6_textEdited(const QString &arg1);

    void on_pBtnConnect_on_connect_3_clicked();

    void closeEvent(QCloseEvent* event) override;

    void judgeAndInputBase(cv::Mat &_mat, const std::vector<cv::Vec3f> &_circle);

    bool closeInquiry();

    bool showMsgBox();




private:

    void disableWidget();
    inline void enableWidget();
    Eigen::Matrix4d svd(std::vector<cv::Point3f> _cam_points_vec, std::vector<cv::Point3f> _base_points_vec);

    Ui::HandEye *ui;

    int dp_;
    int minDist_;
    int param1_;
    int param2_;
    int minRadius_;
    int maxRadius_;
    int mXscale_;
    int mYscale_;

    bool init_scan_;
    std::vector<std::vector<cv::Point3f>> cam_points_vecs_;
    std::vector<std::vector<cv::Point3f>> base_points_vecs_;
    QString parameters_path_;
    cv::Mat tmp_mat_;
    QImage tmp_height_;

    Sensor* sensor_ptr_;
    ImageProc* image_proc_ptr_;
};
#endif // HANDEYE_H
