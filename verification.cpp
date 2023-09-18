#pragma execution_character_set("utf-8")
#include "verification.h"
#include "ui_verification.h"

Verification::Verification(Utils* utils_ptr, QWidget *parent) :
    QWidget(parent),
    handeye_mat_(Eigen::Matrix4d::Zero()), tmp_handeye_mat_(Eigen::Matrix4d::Zero()), utils_ptr_(utils_ptr),
    table_init_num_(5), ui(new Ui::Verification)
{
    ui->setupUi(this);

    QStringList horizontal_headers;
    horizontal_headers << "x" << "y" << "z";
    ui->table_base->setHorizontalHeaderLabels(horizontal_headers);
    ui->table_cam->setHorizontalHeaderLabels(horizontal_headers);
    ui->table_error->setHorizontalHeaderLabels(horizontal_headers);

    utils_ptr_->setValidator(ui->table_base);
    utils_ptr_->setValidator(ui->table_cam);
    utils_ptr_->setValidator(ui->table_matrix);

    vp_handeye_item_.reserve(16);
    vp_error_item_.reserve(3 * table_init_num_);
    vp_base_item_.reserve(3 * table_init_num_);
    vp_cam_item_.reserve(3 * table_init_num_);

    mat_cols_ = ui->table_matrix->columnCount();
    mat_rows_ = ui->table_matrix->rowCount();

    handEyeMatItemInit();
    utils_ptr_->tableItemInit(vp_error_item_, ui->table_error, table_init_num_);
    utils_ptr_->tableItemInit(vp_base_item_, ui->table_base, table_init_num_);
    utils_ptr_->tableItemInit(vp_cam_item_, ui->table_cam, table_init_num_);
}

void Verification::handEyeMatItemInit()
{
    for (int row = 0; row < mat_rows_; row++)
        for (int col = 0; col < mat_cols_; col++)
        {
            QTableWidgetItem* item = new QTableWidgetItem("");
            ui->table_matrix->setItem(row, col, item);
            vp_handeye_item_.emplace_back(item);
        }
}

void Verification::keyPressEvent(QKeyEvent *event)
{
    if (ui->table_base->hasFocus() && (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter))
    {
        //base
        utils_ptr_->judgeEnterTable(vp_base_item_, ui->table_base);
    }

    else if (ui->table_cam->hasFocus() &&(event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter))
    {
        //cam
        utils_ptr_->judgeEnterTable(vp_cam_item_, ui->table_cam);
    }

    else
        QWidget::keyPressEvent(event);
};

void Verification::addErrorItem(const std::vector<Points> &_error_vec, int _rows)
{
    int vec_size = static_cast<int>(vp_error_item_.size());

    if (_rows < vec_size)   //clear
    {
        utils_ptr_->hiddenTable(ui->table_error, _rows, vec_size);
    }
    auto item_iter = vp_error_item_.begin();
    for (size_t row = 0; row < _rows; row++)
    {
        ui->table_error->setRowHidden(row, false);

        QString text_x = QString::number(_error_vec[row].x);
        QString text_y = QString::number(_error_vec[row].y);
        QString text_z = QString::number(_error_vec[row].z);

        item_iter->get()->setText(text_x);
        item_iter++;
        item_iter->get()->setText(text_y);
        item_iter++;
        item_iter->get()->setText(text_z);

        if (item_iter != vp_error_item_.end())
            item_iter++;
    }
}

void Verification::on_pushButton_clicked()
{
    auto base_table = ui->table_base;
    auto cam_table = ui->table_cam;

    std::vector<Points> base_points_vec;
    int base_row_size = utils_ptr_->judgeAppearNum(base_table);

    bool base_check = utils_ptr_->tableCheck(base_table);
    bool cam_check = utils_ptr_->tableCheck(cam_table);

    if (!base_check || !cam_check)
    {
        utils_ptr_->showWarnMsg("表格内容不完整，请补充");
        return;
    }

    if (handeye_mat_.isZero())
    {
        utils_ptr_->showWarnMsg("标定矩阵内容为空");
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
    int cam_row_size = utils_ptr_->judgeAppearNum(cam_table);

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

    int rows = static_cast<int>(error_vec.size());
    int table_rows = static_cast<int>(vp_error_item_.size()) / 3;

    if (rows > table_rows)
    {
        // add extra row
        int substract = rows - table_rows;

        for (int i = 0, current_row = table_rows; i < substract; i++, current_row++)
        {
            ui->table_error->insertRow(current_row);

            QTableWidgetItem* item_x = new QTableWidgetItem();
            ui->table_error->setItem(current_row, 0, item_x);

            QTableWidgetItem* item_y = new QTableWidgetItem();
            ui->table_error->setItem(current_row, 1, item_y);

            QTableWidgetItem* item_z = new QTableWidgetItem();
            ui->table_error->setItem(current_row, 2, item_z);

            vp_error_item_.emplace_back(item_x);
            vp_error_item_.emplace_back(item_y);
            vp_error_item_.emplace_back(item_z);
        }
        addErrorItem(error_vec, rows);
    }

    else
        addErrorItem(error_vec, rows);
}

void Verification::on_pushButton_2_clicked()
{
    utils_ptr_->clearTable(vp_base_item_, ui->table_base);
    utils_ptr_->clearTable(vp_cam_item_, ui->table_cam);
    utils_ptr_->clearTable(vp_error_item_, ui->table_error);
}

void Verification::on_table_matrix_itemChanged(QTableWidgetItem *item)
{
    handeye_mat_(item->row(), item->column()) = item->text().toDouble();
}

void Verification::setHandEyeMatrix(const Eigen::Matrix4d &mat)
{
    auto item_iter = vp_handeye_item_.begin();
    for (int row = 0; row < mat_rows_; row++)
        for (int col = 0; col < mat_cols_; col++)
        {
            QString text = QString::number(mat(row, col));
            item_iter->get()->setText(text);

            if (item_iter != vp_handeye_item_.end())
                item_iter++;
        }
}

void Verification::setTextLog(const QString &_text)
{
    ui->textBrowser_log->setText(_text);
}

void Verification::resetWidget()
{
    utils_ptr_->clearTable(vp_base_item_, ui->table_base);
    utils_ptr_->clearTable(vp_cam_item_, ui->table_cam);
    utils_ptr_->clearTable(vp_error_item_, ui->table_error);
    utils_ptr_->eraseAllRow(vp_handeye_item_);

    ui->textBrowser_log->clear();
}

void Verification::on_pushButton_3_clicked()
{
    if (tmp_handeye_mat_.isZero())
    {
        utils_ptr_->showWarnMsg("未进行标定");
        return;
    }
    else
    {
        setHandEyeMatrix(tmp_handeye_mat_);
    }
}

Verification::~Verification()
{
    delete ui;
}


