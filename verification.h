#ifndef VERIFICATION_H
#define VERIFICATION_H

#include <memory.h>
#include "handeye.h"
#include "utils.h"

struct Points
{
    Points(double _x, double _y, double _z) : x(_x), y(_y), z(_z){}
    Points operator-(const Points &tmp_point)
    {
        Points result(0, 0, 0);
        result.x = abs(x - tmp_point.x);
        result.y = abs(y - tmp_point.y);
        result.z = abs(z - tmp_point.z);

        return result;
    }
    double x;
    double y;
    double z;
};

namespace Ui {
class Verification;
}

class Verification : public QWidget
{
    Q_OBJECT

public:
    explicit Verification(Utils* utils_ptr, QWidget *parent = nullptr);

    void setTextLog(const QString &_text);

    void resetWidget();

    void setHandEyeMatrix(const Eigen::Matrix4d &mat);

    ~Verification();

    size_t max_row_count_;
    Eigen::Matrix4d handeye_mat_;
    Eigen::Matrix4d tmp_handeye_mat_;
    Utils* utils_ptr_;

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_table_matrix_itemChanged(QTableWidgetItem *item);

    void on_pushButton_3_clicked();

private:
    void keyPressEvent(QKeyEvent *event) override;

    void handEyeMatItemInit();
    void addErrorItem(const std::vector<Points> &_error_vec, int _row);
    void updateHandEyeMatrix();

    int mat_cols_;
    int mat_rows_;
    int table_init_num_;

    std::vector<std::unique_ptr<QTableWidgetItem>> vp_handeye_item_;
    std::vector<std::unique_ptr<QTableWidgetItem>> vp_error_item_;
    std::vector<std::unique_ptr<QTableWidgetItem>> vp_base_item_;
    std::vector<std::unique_ptr<QTableWidgetItem>> vp_cam_item_;

    Ui::Verification *ui;
};

#endif // VERIFICATION_H
