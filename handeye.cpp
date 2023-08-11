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

    QByteArray byte_array(reinterpret_cast<const char*>(handeye_mat.data), handeye_mat.total() * handeye_mat.elemSize());

    QString result(byte_array);

    ui->textBrowser_log->append("手眼标定矩阵为:" + result);

}

cv::Mat HandEye::svd(std::vector<cv::Point3f> _cam_points_vec, std::vector<cv::Point3f> _base_points_vec)
{
    cv::Mat A = cv::Mat::zeros(3 * _cam_points_vec.size(), 12, CV_64F);
    cv::Mat B = cv::Mat::zeros(3 * _cam_points_vec.size(), 1, CV_64F);

    for (size_t i = 0; i < _cam_points_vec.size(); ++i)
    {
        const cv::Point3f& camPoint = _cam_points_vec[i];
        const cv::Point3f& basePoint = _base_points_vec[i];

        A.at<double>(3 * i + 0, 0) = -camPoint.x;
        A.at<double>(3 * i + 0, 1) = -camPoint.y;
        A.at<double>(3 * i + 0, 2) = -camPoint.z;
        A.at<double>(3 * i + 0, 3) = -1;
        A.at<double>(3 * i + 0, 9) = camPoint.x * basePoint.x;
        A.at<double>(3 * i + 0, 10) = camPoint.y * basePoint.x;
        A.at<double>(3 * i + 0, 11) = camPoint.z * basePoint.x;

        B.at<double>(3 * i + 0, 0) = basePoint.x;
        B.at<double>(3 * i + 1, 0) = basePoint.y;
        B.at<double>(3 * i + 2, 0) = basePoint.z;
    }

    cv::Mat handeye_mat = A.inv(cv::DECOMP_SVD) * B;

    handeye_mat = handeye_mat.reshape(1, 4);

    return handeye_mat;
}
