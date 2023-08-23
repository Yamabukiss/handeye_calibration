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
    explicit Verification(QWidget *parent = nullptr);

    void setHandEyeMatrix(const Eigen::Matrix4d &_handeye_mat);

    void setTextLog(const QString &text);

    ~Verification();

    size_t max_row_count_;

    Utils* utils_ptr_;

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

private:
    void keyPressEvent(QKeyEvent *event) override;

    Eigen::Matrix4d handeye_mat_;
    std::vector<std::shared_ptr<QTableWidgetItem>> vp_handeye_item_;
    std::vector<std::shared_ptr<QTableWidgetItem>> vp_error_item_;

    Ui::Verification *ui;
};

#endif // VERIFICATION_H
