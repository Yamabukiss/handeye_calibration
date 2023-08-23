#pragma execution_character_set("utf-8")
#include "verification.h"
#include "ui_verification.h"

Verification::Verification(QWidget *parent) :
    QWidget(parent),
    max_row_count_(0), utils_ptr_(new Utils), ui(new Ui::Verification)
{
    ui->setupUi(this);

    QStringList horizontal_headers;
    horizontal_headers << "x" << "y" << "z";
    ui->table_base->setHorizontalHeaderLabels(horizontal_headers);
    ui->table_cam->setHorizontalHeaderLabels(horizontal_headers);
    ui->table_error->setHorizontalHeaderLabels(horizontal_headers);

    utils_ptr_->setValidator(ui->table_base);
    utils_ptr_->setValidator(ui->table_cam);
}

void Verification::keyPressEvent(QKeyEvent *event)
{
    if (ui->table_base->hasFocus() && ui->table_base->rowCount() < max_row_count_ &&
        (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter))
    {
        //base
        auto base_table = ui->table_base;
        utils_ptr_->insertNewRow(base_table);
    }

    else if (ui->table_cam->hasFocus() && ui->table_cam->rowCount() < max_row_count_ &&
             (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter))
    {
        //cam
        auto cam_table = ui->table_cam;
        utils_ptr_->insertNewRow(cam_table);
    }

    else
        QWidget::keyPressEvent(event);
};

void Verification::on_pushButton_clicked()
{
    std::vector<Points> base_points_vec;
    int base_row_size = ui->table_base->rowCount();

    auto base_table = ui->table_base;
    auto cam_table = ui->table_cam;

    bool base_check = utils_ptr_->tableCheck(base_table);
    bool cam_check = utils_ptr_->tableCheck(cam_table);

    if (!base_check || !cam_check)
    {
        utils_ptr_->showWarnMsg("表格内容不完整，请补充");
        return;
    }

    for (int row = 0; row < base_row_size; row++)
    {
        double x = base_table->item(row, 0)->text().toDouble();
        double y = base_table->item(row, 1)->text().toDouble();
        double z = base_table->item(row, 2)->text().toDouble();

        base_points_vec.emplace_back(Points(x, y, z));
    }

    std::vector<Points> cam_points_vec;
    int cam_row_size = cam_table->rowCount();

    for (int row = 0; row < cam_row_size; row++)
    {
        double x = cam_table->item(row, 0)->text().toDouble();
        double y = cam_table->item(row, 1)->text().toDouble();
        double z = cam_table->item(row, 2)->text().toDouble();

        Eigen::Vector4d cam_point(x, y, z, 1.0);
        auto c_base_point = handeye_mat_ * cam_point;

        cam_points_vec.emplace_back(Points(c_base_point[0], c_base_point[1], c_base_point[2]));
    }

    if (base_points_vec.size() != cam_points_vec.size())
    {
        utils_ptr_->showWarnMsg("点数不匹配，请补充");
        on_pushButton_2_clicked();
        return;
    }

    std::vector<Points> error_vec;
    for (size_t i = 0; i < base_points_vec.size(); i++)
        error_vec.emplace_back(base_points_vec[i] - cam_points_vec[i]);

    size_t rows = error_vec.size();
    for (size_t row = 0; row < rows; row++)
    {

        QString text_x = QString::number(error_vec[row].x);
        QTableWidgetItem* item_x = new QTableWidgetItem(text_x);
        ui->table_error->setItem(row, 0, item_x);

        QString text_y = QString::number(error_vec[row].y);
        QTableWidgetItem* item_y = new QTableWidgetItem(text_y);
        ui->table_error->setItem(row, 1, item_y);

        QString text_z = QString::number(error_vec[row].z);
        QTableWidgetItem* item_z = new QTableWidgetItem(text_z);
        ui->table_error->setItem(row, 2, item_z);

        vp_error_item_.emplace_back(std::make_shared<QTableWidgetItem>(*item_x));
        vp_error_item_.emplace_back(std::make_shared<QTableWidgetItem>(*item_y));
        vp_error_item_.emplace_back(std::make_shared<QTableWidgetItem>(*item_z));
    }
}

void Verification::on_pushButton_2_clicked()
{
    ui->table_base->clear();
    ui->table_base->setRowCount(1);

    ui->table_cam->clear();
    ui->table_cam->setRowCount(1);

    ui->table_error->clear();
    ui->table_error->setRowCount(1);

}

void Verification::setHandEyeMatrix(const Eigen::Matrix4d &_handeye_mat)
{
    int cols = ui->table_matrix->columnCount();
    int rows = ui->table_matrix->rowCount();

    for (int row = 0; row < rows; row++)
        for (int col = 0; col < cols; col++)
        {
            QString text = QString::number(_handeye_mat(row, col));
            QTableWidgetItem* item = new QTableWidgetItem(text);

            vp_handeye_item_.emplace_back(std::make_shared<QTableWidgetItem>(*item));
            ui->table_matrix->setItem(row, col, item);
        }

    handeye_mat_ = _handeye_mat;
}

void Verification::setTextLog(const QString &text)
{
    ui->textBrowser_log->setText(text);
}

Verification::~Verification()
{
    delete ui;
    delete utils_ptr_;
}


