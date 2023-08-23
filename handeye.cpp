#pragma execution_character_set("utf-8")
#include "handeye.h"
#include "ui_handeye.h"
#include "verification.h"

HandEye::HandEye(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::HandEye), mXscale_(0), mYscale_(0), init_scan_(true),
    sensor_ptr_(new Sensor), image_proc_ptr_(new ImageProc), verify_ptr_(new Verification)
{
    ui->setupUi(this);

    QStringList horizontal_headers;
    horizontal_headers << "x" << "y" << "z";
    ui->tableWidget->setHorizontalHeaderLabels(horizontal_headers);

    connect(sensor_ptr_->call_one_times_ptr_, &CallOneTimes::SignalDataShow,
            this, &HandEye::showImage);

    parameters_path_ = QCoreApplication::applicationDirPath()+"/config/parameters.json";
    QFile file(parameters_path_);
    if (!file.open(QIODevice::ReadOnly))
    {
        ui->textBrowser_log->append("参数文件读取失败");
        dp_ = 0;
        minDist_= 0;
        param1_ = 0;
        param2_ = 0;
        minRadius_ = 0;
        maxRadius_ = 0;
    }

    else
    {
        QByteArray json_data = file.readAll();
        QJsonDocument json_document = QJsonDocument::fromJson(json_data);
        QJsonObject json_object = json_document.object();
        int dp = json_object.take("dp").toVariant().toString().toInt();
        int minDist = json_object.take("minDist").toVariant().toString().toInt();
        int param1 = json_object.take("param1").toVariant().toString().toInt();
        int param2 = json_object.take("param2").toVariant().toString().toInt();
        int minRadius = json_object.take("minRadius").toVariant().toString().toInt();
        int maxRadius = json_object.take("maxRadius").toVariant().toString().toInt();
        file.close();

        ui->lineEdit->setText(QString::number(dp));
        ui->lineEdit_2->setText(QString::number(minDist));
        ui->lineEdit_3->setText(QString::number(param1));
        ui->lineEdit_4->setText(QString::number(param2));
        ui->lineEdit_5->setText(QString::number(minRadius));
        ui->lineEdit_6->setText(QString::number(maxRadius));

        dp_ = dp;
        minDist_= minDist;
        param1_ = param1;
        param2_ = param2;
        minRadius_ = minRadius;
        maxRadius_ = maxRadius;
    }

    ui->pBtnConnect_on_connect_3->setEnabled(false);
    ui->pBtnConnect_on_connect_4->setEnabled(false);
    ui->pBtnConnect_on_single->setEnabled(false);
    ui->pBtnConnect_on_single_3->setEnabled(false);
    ui->pBtnConnect_on_single_4->setEnabled(false);
    ui->pBtnConnect_on_single_5->setEnabled(false);

    verify_ptr_->utils_ptr_->setValidator(ui->tableWidget);
}

void HandEye::on_pBtnConnect_on_connect_clicked()
{
    bool ret = sensor_ptr_->ethenetConnect();

    if(ret)
    {
        ui->pBtnConnect_on_connect_3->setEnabled(true);
        ui->pBtnConnect_on_connect_4->setEnabled(true);
        ui->pBtnConnect_on_single->setEnabled(true);
        ui->pBtnConnect_on_single_3->setEnabled(true);
        ui->pBtnConnect_on_connect->setEnabled(false);
        ui->textBrowser_log->append("连接成功");
    }
    else
    {
        ui->textBrowser_log->append("连接失败");
    }

}

void HandEye::on_pBtnConnect_on_single_clicked()
{
    ui->label_gray->clear();
    disableFunctionButton();
    bool ret = false;

    if (init_scan_)
    {
        ret = sensor_ptr_->initBatch();
        init_scan_ = false;
    }

    else
        ret = sensor_ptr_->singleBatch();

    if(!ret)
        ui->textBrowser_log->append("执行扫图失败");
    else
        ui->textBrowser_log->append("执行扫图成功");
}

void HandEye::keyPressEvent(QKeyEvent *event)
{
    auto base_table = ui->tableWidget;

    if (base_table->hasFocus())
    {
        bool judge = true;
        if (cam_points_vecs_.empty())
            judge = false;

        else
        {
            size_t cam_vec_size = cam_points_vecs_[cam_points_vecs_.size() - 1].size();
            judge = base_table->rowCount() < cam_vec_size;
        }

        if (judge && (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter))
            verify_ptr_->utils_ptr_->insertNewRow(base_table);

        else
        {
            QWidget::keyPressEvent(event);
        }

    }

    else
    {
        QWidget::keyPressEvent(event);
    }
};

void HandEye::enableWidget()
{
    ui->pBtnConnect_on_single->setEnabled(true);

    static size_t count = 0;

    if (!cam_points_vecs_.empty())
        count += cam_points_vecs_[cam_points_vecs_.size() - static_cast<size_t>(1)].size();

    if (!ui->pBtnConnect_on_single_4->isEnabled() && count >= 5)
    {
        ui->pBtnConnect_on_single_4->setEnabled(true);
        ui->pBtnConnect_on_single_5->setEnabled(true);
    }
}

void HandEye::disableWidget()
{
    ui->pBtnConnect_on_single->setEnabled(false);
    if (ui->pBtnConnect_on_single_4->isEnabled())
    {
        ui->pBtnConnect_on_single_4->setEnabled(false);
        ui->pBtnConnect_on_single_5->setEnabled(false);
    }
}

void HandEye::disableFunctionButton()
{
    ui->pBtnConnect_on_connect_3->setEnabled(false);
    ui->pBtnConnect_on_single_3->setEnabled(false);
}

void HandEye::enableFunctionButton()
{
    ui->pBtnConnect_on_connect_3->setEnabled(true);
    ui->pBtnConnect_on_single_3->setEnabled(true);
}

void HandEye::showImage(int _width, int _height)
{
    ui->textBrowser_log->append("请等待图像处理...");
    enableFunctionButton();
    //scale size
    int mScaleW = ui->label_gray->width();
    int mScaleH = ui->label_gray->height();
    int mXscale = int(double(_width) / mScaleW);
    int mYscale = int(double(_height) / mScaleH);

    if (mXscale_ == 0 || mYscale == 0)
    {
        mXscale_ = mXscale;
        mYscale_ = mYscale;
    }

    int mCamId = 0;

    auto gray_image = sensor_ptr_->GrayDataShow(sensor_ptr_->call_one_times_ptr_->getIntensityData(mCamId),
                                                _width,
                                                _height,
                                                mXscale,
                                                mYscale,
                                                mScaleW,
                                                mScaleH);


    auto height_image = sensor_ptr_->BatchDataShow(sensor_ptr_->call_one_times_ptr_->getBatchData(mCamId),
                  sensor_ptr_->dheight_upper_,
                  sensor_ptr_->dheight_lower_,
                  255,
                  _width,
                  _height,
                  mXscale,
                  mYscale,
                  mScaleW,
                  mScaleH);

    if (height_image.isNull() || gray_image.isNull())
    {
        ui->textBrowser_log->append("扫图错误，请检查相机");
        return;
    }

    auto gray_pixmap = QPixmap::fromImage(gray_image);

    cv::Mat mat = image_proc_ptr_->pixmapToCvMat(gray_pixmap);

    tmp_mat_ = mat.clone();
    tmp_height_ = height_image;

    auto circles = image_proc_ptr_->getCircle(mat, dp_, minDist_, param1_, param2_, minRadius_, maxRadius_);

    if (circles.empty())
    {
        ui->textBrowser_log->append("没有找到点，请重新扫图");
        ui->label_gray->setPixmap(gray_pixmap);
        return;
    }

    judgeAndInputBase(mat, circles);
}

void HandEye::judgeAndInputBase(cv::Mat &_mat, const std::vector<cv::Vec3d> &_circle)
{
    for (size_t i = 0; i < _circle.size(); i++)
    {
        cv::Point center(cvRound(_circle[i][0]), cvRound(_circle[i][1]));
        int radius = cvRound(_circle[i][2]);
        cv::circle(_mat, center, radius, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        cv::putText(_mat, std::to_string(i + 1), center + cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    }

    auto drawed_pixmap = image_proc_ptr_->matToPixmap(_mat);
    ui->label_gray->setPixmap(drawed_pixmap);

    ui->textBrowser_log->append("寻找成功 请按照顺序输入任意数量base的xyz坐标");
    std::vector<cv::Point3d> cam_points_vec;

    QString batch_text;
    for (size_t i = 0; i < _circle.size(); i++)
    {
        auto corner = _circle[i];
        double x = corner[0];
        double y = corner[1];
        unsigned char gray_value = tmp_height_.pixel(x, y);
        double z = static_cast<double>(gray_value) / sensor_ptr_->fscale_ + sensor_ptr_->dheight_lower_;

        x = x * mXscale_ * sensor_ptr_->call_one_times_ptr_->mXinterVal;
        y = y * mYscale_ * sensor_ptr_->call_one_times_ptr_->mYinterVal;

        cv::Point3d point_3d(x, y, z);

        cam_points_vec.emplace_back(point_3d);
        QString text = QString("点%1物理坐标为: %2, %3, %4").arg(i + 1).arg(x).arg(y).arg(z);

        ui->textBrowser_log->append(text);
        batch_text += text + "\n";
    }

    verify_ptr_->setTextLog(batch_text);

    cam_points_vecs_.emplace_back(cam_points_vec);

    disableWidget();
}

void HandEye::showPointsSum()
{
    size_t points_count = 0;
    for (const auto &vec : cam_points_vecs_)
        points_count += vec.size();

    ui->textBrowser_log->append("当前总点数为:" + QString::number(points_count));
}

void HandEye::on_pBtnConnect_on_single_3_clicked()
{
    std::vector<cv::Point3d> base_points_vec;
    int row_size = ui->tableWidget->rowCount();

    bool base_check = verify_ptr_->utils_ptr_->tableCheck(ui->tableWidget);
    if (!base_check)
    {
        verify_ptr_->utils_ptr_->showWarnMsg("表格内容不完整，请补充");
        return;
    }

    for (int row = 0; row < row_size; row++)
    {
        double x = ui->tableWidget->item(row, 0)->text().toDouble();
        double y = ui->tableWidget->item(row, 1)->text().toDouble();
        double z = ui->tableWidget->item(row, 2)->text().toDouble();
        base_points_vec.emplace_back(cv::Point3d(x, y, z));

    }
        size_t vec_size = base_points_vec.size();
        cam_points_vecs_[cam_points_vecs_.size() - static_cast<size_t>(1)].resize(vec_size);
        base_points_vecs_.emplace_back(base_points_vec);

        showPointsSum();
        resetTableWidget();
}

void HandEye::on_pBtnConnect_on_single_4_clicked()
{
    std::vector<cv::Point3d> cam_points_vec;
    std::vector<cv::Point3d> base_points_vec;

    for (const auto &point_vec : cam_points_vecs_)
        cam_points_vec.insert(cam_points_vec.end(), point_vec.begin(), point_vec.end());

    for (const auto &point_vec : base_points_vecs_)
        base_points_vec.insert(base_points_vec.end(), point_vec.begin(), point_vec.end());

    auto handeye_mat = svd(cam_points_vec, base_points_vec);
    verify_ptr_->setHandEyeMatrix(handeye_mat);
    QString mat_string;

    for (int row = 0; row < 4; ++row)
    {
        mat_string += "[";

        for (int col = 0; col < 4; ++col)
        {
            if (col != 3)
                mat_string +=  QString::number(handeye_mat(row, col)) + ",";
            else
                mat_string +=  QString::number(handeye_mat(row, col));
        }

        mat_string += "]";

        mat_string += "\n";
    }

    ui->textBrowser_log->append("手眼标定矩阵为:" + mat_string);
}

Eigen::Matrix4d HandEye::svd(std::vector<cv::Point3d> _cam_points_vec, std::vector<cv::Point3d> _base_points_vec)
{
    Eigen::MatrixXd A(4, _cam_points_vec.size());
    Eigen::MatrixXd B(4, _base_points_vec.size());

    for (size_t i = 0; i < _cam_points_vec.size(); i++)
    {
        A.col(i) << _cam_points_vec[i].x, _cam_points_vec[i].y, _cam_points_vec[i].z, 1.0;
        B.col(i) << _base_points_vec[i].x, _base_points_vec[i].y, _base_points_vec[i].z, 1.0;
    }

    Eigen::Vector4d mA = A.rowwise().mean();
    Eigen::Vector4d mB = B.rowwise().mean();

    A = A.colwise() - mA;
    B = B.colwise() - mB;

    Eigen::Matrix4d H = A * B.transpose();

    Eigen::JacobiSVD<Eigen::Matrix4d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix4d U = svd.matrixU();
    Eigen::Matrix4d V = svd.matrixV();

    Eigen::Matrix4d R = V * U.transpose();

    double det_R = R.determinant();
    if (det_R < 0) {
        V.col(3) *= -1;
        R = V * U.transpose();
    }

    Eigen::Vector4d t = mB - R * mA;

    Eigen::Matrix4d transformation_mat = Eigen::Matrix4d::Identity();
    transformation_mat.block(0, 0, 3, 3) = R.block(0, 0, 3, 3);
    transformation_mat.block(0, 3, 3, 1) = t.block(0, 0, 3, 1);

    return transformation_mat;
}

void HandEye::on_pBtnConnect_on_connect_2_clicked()
{
    QFile file(parameters_path_);

    if(!file.open(QIODevice::ReadWrite))
    {
        ui->textBrowser_log->append("参数保存失败");
        return;
    }

    QJsonDocument json_document(QJsonDocument::fromJson(file.readAll()));
    QJsonObject json_obj = json_document.object();

    json_obj["dp"] = ui->lineEdit->text();
    json_obj["minDist"] = ui->lineEdit_2->text();
    json_obj["param1"] = ui->lineEdit_3->text();
    json_obj["param2"] = ui->lineEdit_4->text();
    json_obj["minRadius"] = ui->lineEdit_5->text();
    json_obj["maxRadius"] = ui->lineEdit_6->text();

    json_document.setObject(json_obj);
    QByteArray jsonData = json_document.toJson();
    file.close();

    QFile::remove(parameters_path_);

    QFile newfile(parameters_path_);
    if(!newfile.open(QIODevice::WriteOnly)){
        qDebug()<<"jsonFile open error";
    }
    newfile.write(jsonData);
    ui->textBrowser_log->append("参数保存成功");
}

void HandEye::on_lineEdit_textEdited(const QString &arg1)
{
    dp_ = arg1.toInt();
}


void HandEye::on_lineEdit_2_textEdited(const QString &arg1)
{
    minDist_ = arg1.toInt();
}

void HandEye::on_lineEdit_3_textEdited(const QString &arg1)
{
    param1_ = arg1.toInt();
}


void HandEye::on_lineEdit_4_textEdited(const QString &arg1)
{
    param2_ = arg1.toInt();
}


void HandEye::on_lineEdit_5_textEdited(const QString &arg1)
{
    minRadius_ = arg1.toInt();
}

void HandEye::on_lineEdit_6_textEdited(const QString &arg1)
{
    maxRadius_ = arg1.toInt();
}

void HandEye::on_pBtnConnect_on_connect_3_clicked()
{
    cv::Mat mat = tmp_mat_.clone();
    auto circles = image_proc_ptr_->getCircle(mat, dp_, minDist_, param1_, param2_, minRadius_, maxRadius_);
    if (circles.empty())
    {
        ui->textBrowser_log->append("没有找到点，请重新扫图");
        return;
    }
    if (!cam_points_vecs_.empty())
        cam_points_vecs_.pop_back();
    judgeAndInputBase(mat, circles);
    resetTableWidget();
}

bool HandEye::closeInquiry()
{
    QMessageBox msg_box;
    QFont font;
    font.setFamily("黑体");
    font.setPointSize(11);

    msg_box.setFont(font);

    msg_box.setWindowTitle("保存参数");
    msg_box.setText("是否需要保存参数？");

    QPushButton *okButton = msg_box.addButton("确认", QMessageBox::AcceptRole);
    QPushButton *cancelButton = msg_box.addButton("放弃", QMessageBox::RejectRole);

    msg_box.exec();

    if (msg_box.clickedButton() == okButton)
        return true;
    else if (msg_box.clickedButton() == cancelButton)
        return false;
    else
        return false;
}

void HandEye::closeEvent(QCloseEvent* event)
{
    if (closeInquiry())
        on_pBtnConnect_on_connect_2_clicked();
    event->accept();
}

void HandEye::resetTableWidget()
{
    ui->tableWidget->clear();
    ui->tableWidget->setRowCount(1);
    enableWidget();
}

void HandEye::on_pBtnConnect_on_single_5_clicked()
{
    verify_ptr_->show();
    verify_ptr_->max_row_count_ = cam_points_vecs_[cam_points_vecs_.size() - static_cast<size_t>(1)].size();
}

void HandEye::on_pBtnConnect_on_connect_4_clicked()
{
    if (!cam_points_vecs_.empty())
        cam_points_vecs_.pop_back();

    showPointsSum();
    enableWidget();
    ui->label_gray->clear();
}

HandEye::~HandEye()
{
    delete ui;
    delete sensor_ptr_;
    delete image_proc_ptr_;
    delete verify_ptr_;
}

