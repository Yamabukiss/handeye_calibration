#pragma execution_character_set("utf-8")
#include "handeye.h"
#include "ui_handeye.h"

HandEye::HandEye(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::HandEye), mXscale_(0), mYscale_(0), init_scan_(true), sensor_ptr_(new Sensor), image_proc_ptr_(new ImageProc)
{
    ui->setupUi(this);

    parameters_path_ = QCoreApplication::applicationDirPath()+"/config/parameters.json";
    QFile file(parameters_path_);
    if (!file.open(QIODevice::ReadOnly))
    {
        ui->textBrowser_log->append("参数文件读取失败");
    }

    QStringList horizontal_headers;
    horizontal_headers << "x" << "y" << "z";
    ui->tableWidget->setHorizontalHeaderLabels(horizontal_headers);

    connect(sensor_ptr_->call_one_times_ptr_, &CallOneTimes::SignalDataShow,
            this, &HandEye::showImage);
    ui->pBtnConnect_on_single_4->setDisabled(false);

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

void HandEye::on_pBtnConnect_on_connect_clicked()
{
    bool ret = sensor_ptr_->ethenetConnect();

    if(ret)
    {
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
    bool ret = false;
    if (init_scan_)
    {
        ret = sensor_ptr_->initBatch();
        init_scan_ = false;
    }

    else
        ret = sensor_ptr_->singleBatch();

    if(!ret)
    {
        ui->textBrowser_log->append("执行扫图失败");
    }
    else
    {
        ui->textBrowser_log->append("执行扫图成功");
    }

}

void HandEye::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter)
    {
        int new_row = ui->tableWidget->rowCount();
        bool new_row_empty = true;
        for (int col = 0; col < ui->tableWidget->columnCount(); ++col)
        {
            QTableWidgetItem *item = ui->tableWidget->item(new_row - 1, col);
            if (item && !item->text().isEmpty())
            {
                new_row_empty = false;
                break;
            }
        }
        if (!new_row_empty)
            ui->tableWidget->insertRow(new_row);

        ui->tableWidget->setCurrentCell(new_row, 0);
    }
    else
    {
        QWidget::keyPressEvent(event);
    }
};

inline void HandEye::enableWidget()
{
    ui->pBtnConnect_on_single->setEnabled(true);

    int count = 0;

    for (const auto& innerVector : cam_points_vecs_)
        count += innerVector.size();

    if (count >= 5)
        ui->pBtnConnect_on_single_4->setEnabled(true);
}

inline void HandEye::disableWidget()
{
    ui->pBtnConnect_on_single->setEnabled(false);
    ui->pBtnConnect_on_single_4->setEnabled(false);
}

void HandEye::showImage(int _width, int _height)
{
    ui->textBrowser_log->append("请等待图像处理...");
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

    tmp_mat_ = mat;
    tmp_height_ = height_image;

    auto circles = image_proc_ptr_->getCircle(mat, dp_, minDist_, param1_, param2_, minRadius_, maxRadius_);
    judgeAndInputBase(mat, circles);
}

bool HandEye::showMsgBox()
{
    QMessageBox msg_box;
    QFont font;
    font.setFamily("黑体");
    font.setPointSize(11);

    msg_box.setFont(font);

    msg_box.setWindowTitle("确认点位");
    msg_box.setText("是否寻找到了点位？");

    msg_box.addButton("否", QMessageBox::RejectRole);
    QPushButton *okButton = msg_box.addButton("是", QMessageBox::AcceptRole);

    msg_box.exec();

    bool judge = msg_box.clickedButton() == okButton;

    return judge;
}

void HandEye::judgeAndInputBase(cv::Mat &_mat, const std::vector<cv::Vec3f> &_circle)
{
    for (size_t i = 0; i < _circle.size(); ++i)
    {
        cv::Point center(cvRound(_circle[i][0]), cvRound(_circle[i][1]));
        int radius = cvRound(_circle[i][2]);
        cv::circle(_mat, center, radius, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        cv::putText(_mat, std::to_string(i + 1), center + cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    }

    auto drawed_pixmap = image_proc_ptr_->matToPixmap(_mat);
    ui->label_gray->setPixmap(drawed_pixmap);

    bool judge = showMsgBox();

    if (!_circle.empty() && judge)
    {
        ui->textBrowser_log->append("寻找成功 请按照顺序输入任意数量base的xyz坐标");
        std::vector<cv::Point3f> cam_points_vec;
        for (const auto &corner : _circle)
        {
            float x = corner[0];
            float y = corner[1];
            unsigned char gray_value = tmp_height_.pixel(x, y);
            float z = static_cast<float>(gray_value) / sensor_ptr_->fscale_ + sensor_ptr_->dheight_lower_;
            cv::Point3f point_3f(x * mXscale_ * sensor_ptr_->call_one_times_ptr_->mXinterVal,
                                 y * mYscale_ * sensor_ptr_->call_one_times_ptr_->mYinterVal,
                                 z);

            cam_points_vec.emplace_back(point_3f);
            ui->textBrowser_log->append(QString("%1, %2, %3").arg(x).arg(y).arg(z));
        }

        cam_points_vecs_.emplace_back(cam_points_vec);
        disableWidget();
    }

    else
    {
        ui->textBrowser_log->append("寻找失败，请重新扫图或调整参数");
        ui->tableWidget->clear();
        ui->tableWidget->setRowCount(1);
    }

}


void HandEye::on_pBtnConnect_on_single_3_clicked()
{
    std::vector<cv::Point3f> base_points_vec;
    int row_size = ui->tableWidget->rowCount();
    if (row_size == 0)
    {
        ui->textBrowser_log->append("您未输入任何有效数据");
        return;
    }

    for (int row = 0; row < row_size; row++)
    {
        try
        {
            float x = ui->tableWidget->item(row, 0)->text().toFloat();
            float y = ui->tableWidget->item(row, 1)->text().toFloat();
            float z = ui->tableWidget->item(row, 2)->text().toFloat();
            base_points_vec.emplace_back(cv::Point3f(x, y, z));
        }
        catch (std::exception e)
        {
            ui->textBrowser_log->append("未完全输入xyz三列数据，请重新输入");
            return;
        }
    }
    int vec_size = base_points_vec.size();
    cam_points_vecs_[cam_points_vecs_.size() - 1].resize(vec_size);
    base_points_vecs_.emplace_back(base_points_vec);

    ui->textBrowser_log->append("输入成功, 当前点集数为:" + QString::number(cam_points_vecs_.size()));
    ui->tableWidget->clear();
    enableWidget();
    ui->tableWidget->setRowCount(1);
}


void HandEye::on_pBtnConnect_on_single_4_clicked()
{
    std::vector<cv::Point3f> cam_points_vec;
    std::vector<cv::Point3f> base_points_vec;

    for (const auto &point_vec : cam_points_vecs_)
        cam_points_vec.insert(cam_points_vec.end(), point_vec.begin(), point_vec.end());

    for (const auto &point_vec : base_points_vecs_)
        base_points_vec.insert(base_points_vec.end(), point_vec.begin(), point_vec.end());

    auto handeye_mat = svd(cam_points_vec, base_points_vec);

    QString mat_string;

    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            mat_string += QString::number(handeye_mat(row, col)) + " ";
        }
        mat_string += "\n";
    }

    ui->textBrowser_log->append("手眼标定矩阵为:" + mat_string);

}

Eigen::Matrix4d HandEye::svd(std::vector<cv::Point3f> _cam_points_vec, std::vector<cv::Point3f> _base_points_vec)
{
    Eigen::MatrixXd A(4, _cam_points_vec.size());
    Eigen::MatrixXd B(4, _base_points_vec.size());

    for (int i = 0; i < _cam_points_vec.size(); ++i)
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

HandEye::~HandEye()
{
    delete ui;
    delete sensor_ptr_;
    delete image_proc_ptr_;
}

void HandEye::on_pBtnConnect_on_connect_2_clicked()
{
    QFile file(parameters_path_);

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
    judgeAndInputBase(mat, circles);
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

