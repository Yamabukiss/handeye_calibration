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
#include <QTableWidgetItem>
#include <QIntValidator>
#include <thread>
#include "sensor.h"
#include "imageproc.h"
#include "utils.h"
#include "solver.h"

QT_BEGIN_NAMESPACE
namespace Ui { class HandEye; }
QT_END_NAMESPACE

class Verification;

class HandEye : public QMainWindow
{
    Q_OBJECT

public:
    explicit HandEye(QWidget *parent = nullptr);
    ~HandEye();

    Sensor* sensor_ptr_;

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

    void on_reset_button_clicked();

private:

    void disableWidget();

    void judgePointsNum();

    void disableFunctionButton();

    void enableFunctionButton();

    void showPointsSum();

    void resetWidget();

    void closeReset();

    inline void setLineEditText(QLineEdit* line_edit, int value);

    bool init_scan_;
    int dp_;
    int minDist_;
    int param1_;
    int param2_;
    int minRadius_;
    int maxRadius_;
    int mXscale_;
    int mYscale_;
    int scale_;
    int table_init_num_;
    QString parameters_path_;


    cv::Mat tmp_mat_;
    QImage tmp_height_;
    std::vector<std::vector<cv::Point3d>> cam_points_vecs_;
    std::vector<std::vector<cv::Point3d>> base_points_vecs_;
    std::vector<std::unique_ptr<QTableWidgetItem>> vp_input_item_;


    ImageProc* image_proc_ptr_;
    Utils* utils_ptr_;
    Verification* verify_ptr_;
    Solver* solver_ptr_;
    Ui::HandEye *ui;
};
#endif // HANDEYE_H
