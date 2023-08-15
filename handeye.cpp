#pragma execution_character_set("utf-8")
#include "handeye.h"
#include "ui_handeye.h"

HandEye::HandEye(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::HandEye), init_scan_(true), sensor_ptr_(new Sensor), image_proc_ptr_(new ImageProc)
{
    ui->setupUi(this);
    QStringList horizontal_headers;
    horizontal_headers << "x" << "y" << "z";
    ui->tableWidget->setHorizontalHeaderLabels(horizontal_headers);

    connect(sensor_ptr_->call_one_times_ptr_, &CallOneTimes::SignalDataShow,
            this, &HandEye::showImage);
    ui->pBtnConnect_on_single_4->setDisabled(false);

}

HandEye::~HandEye()
{
    delete ui;
}

// TODO: bug exist when it has connected
void HandEye::on_pBtnConnect_on_connect_clicked()
{
    bool ret = sensor_ptr_->ethenetConnect();

    if(ret)
    {
        QTime current_time = QTime::currentTime();
        QString log_text = "[INFO] " + QString::number(current_time.hour()) + ":" + QString::number(current_time.minute())
                           + ":" + QString::number(current_time.second()) + " 连接成功";
        ui->textBrowser_log->append(log_text);
    }
    else
    {
        QTime current_time = QTime::currentTime();
        QString log_text = "[ERROR] " + QString::number(current_time.hour()) + ":" + QString::number(current_time.minute())
                           + ":" + QString::number(current_time.second()) + " 连接失败";
        ui->textBrowser_log->append(log_text);
    }

}


void HandEye::on_pBtnConnect_on_single_clicked()
{
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
        auto text = QString("执行扫图失败，返回值:%1").arg(ret);
        QTime current_time = QTime::currentTime();
        QString log_text = "[ERROR] " + QString::number(current_time.hour()) + ":" + QString::number(current_time.minute())
                           + ":" + QString::number(current_time.second()) + " " + text;
        ui->textBrowser_log->append(log_text);
    }
    else
    {
        QTime current_time = QTime::currentTime();
        QString log_text = "[INFO] " + QString::number(current_time.hour()) + ":" + QString::number(current_time.minute())
                           + ":" + QString::number(current_time.second()) + " 执行扫图成功";
        ui->textBrowser_log->append(log_text);
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
    ui->pBtnConnect_on_single_2->setEnabled(true);
    ui->pBtnConnect_on_single_3->setEnabled(true);

    if (cam_points_vecs_.size() >= 5)
        ui->pBtnConnect_on_single_4->setEnabled(true);
}

inline void HandEye::disableWidget()
{
    ui->pBtnConnect_on_single->setEnabled(false);
    ui->pBtnConnect_on_single_2->setEnabled(false);
    ui->pBtnConnect_on_single_3->setEnabled(false);
    ui->pBtnConnect_on_single_4->setEnabled(false);
}

void HandEye::showImage(int _width, int _height)
{
    //scale size
    int mScaleW = ui->label_gray->width();
    int mScaleH = ui->label_gray->height();
    int mXscale = int(double(_width) / mScaleW);
    int mYscale = int(double(_height) / mScaleH);

    int mCamId = 0;

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


    auto gray_image = sensor_ptr_->GrayDataShow(sensor_ptr_->call_one_times_ptr_->getIntensityData(mCamId),
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
    ui->label_gray->setPixmap(gray_pixmap);

    cv::Mat mat = image_proc_ptr_->pixmapToCvMat(gray_pixmap);
    auto ret_matcorners = image_proc_ptr_->findCorner(mat);

    auto ret = ret_matcorners.first;
    auto corners = ret_matcorners.second;

    auto drawed_pixmap = image_proc_ptr_->matToPixmap(mat);
    ui->label_gray->setPixmap(drawed_pixmap);

    if (ret)
    {
        ui->textBrowser_log->append("寻找角点成功 请按照顺序输入任意数量base的xyz坐标");
        std::vector<cv::Point3f> cam_points_vec;
        for (const auto &corner : corners)
        {
            int x = corner.x;
            int y = corner.y;
            int gray_value = height_image.pixel(x, y);
            float height = gray_value / sensor_ptr_->fscale_ + sensor_ptr_->dheight_lower_;
            cv::Point3f point_3f(corner.x * mXscale * sensor_ptr_->call_one_times_ptr_->mXinterVal,
                                 corner.y * mYscale * sensor_ptr_->call_one_times_ptr_->mYinterVal,
                                 height);

            cam_points_vec.emplace_back(point_3f);
        }

        cam_points_vecs_.emplace_back(cam_points_vec);
        disableWidget();
    }

    else
    {
        ui->textBrowser_log->append("寻找角点失败，请重新扫图");
    }

    ui->textBrowser_log->append("当前点集总数:" + QString::number(cam_points_vecs_.size()));
}


void HandEye::on_pBtnConnect_on_single_2_clicked()
{
    sensor_ptr_->InitConfigBeforeDisConnect();
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
    cam_points_vecs_[-1].resize(vec_size);
    base_points_vecs_.emplace_back(base_points_vec);

    ui->textBrowser_log->append("输入成功, 当前点集数为:" + QString::number(cam_points_vecs_.size()));
    ui->tableWidget->clear();
    enableWidget();
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
