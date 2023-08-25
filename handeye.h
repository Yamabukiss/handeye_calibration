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

class Verification;

class HandEye : public QMainWindow
{
    Q_OBJECT

public:
    HandEye(QWidget *parent = nullptr);
    ~HandEye();

private slots:
    void showImage(int _width, int _height);

    void keyPressEvent(QKeyEvent *event) override;

    void on_connect_button_clicked();

    void on_scan_button_clicked();

    void on_input_button_clicked();

    void on_calculate_button_clicked();

    void on_save_button_clicked();

    void on_lineEdit_textEdited(const QString &arg1);

    void on_lineEdit_2_textEdited(const QString &arg1);

    void on_lineEdit_3_textEdited(const QString &arg1);

    void on_lineEdit_4_textEdited(const QString &arg1);

    void on_lineEdit_5_textEdited(const QString &arg1);

    void on_lineEdit_6_textEdited(const QString &arg1);

    void on_update_button_clicked();

    void closeEvent(QCloseEvent* event) override;

    void judgeAndInputBase(cv::Mat &_mat, const std::vector<cv::Vec3d> &_circle);

    void resetTableWidget();

    bool closeInquiry();

    void on_test_button_clicked();

    void on_drop_button_clicked();

private:

    void disableWidget();

    void judgePointsNum();

    void disableFunctionButton();

    void enableFunctionButton();

    void showPointsSum();

    Eigen::Matrix4d svd(std::vector<cv::Point3d> _cam_points_vec, std::vector<cv::Point3d> _base_points_vec);

    int dp_;
    int minDist_;
    int param1_;
    int param2_;
    int minRadius_;
    int maxRadius_;
    int mXscale_;
    int mYscale_;

    bool init_scan_;
    std::vector<std::vector<cv::Point3d>> cam_points_vecs_;
    std::vector<std::vector<cv::Point3d>> base_points_vecs_;
    QString parameters_path_;
    cv::Mat tmp_mat_;
    QImage tmp_height_;

    Sensor* sensor_ptr_;
    ImageProc* image_proc_ptr_;
    Verification* verify_ptr_;
    Ui::HandEye *ui;
};
#endif // HANDEYE_H
