﻿#pragma execution_character_set("utf-8")
#include "handeye.h"
#include "ui_handeye.h"
#include "verification.h"

HandEye::HandEye(QWidget *parent)
    : QMainWindow(parent)
    , sensor_ptr_(new Sensor), init_scan_(true), mXscale_(3), mYscale_(3), scale_(1),
    table_init_num_(5), width_size_(3200), batch_size_(6000), image_proc_ptr_(new ImageProc),
    utils_ptr_(new Utils), verify_ptr_(new Verification(utils_ptr_)), solver_ptr_(new Solver(3)),
    ui(new Ui::HandEye)
{
    ui->setupUi(this);

    QStringList horizontal_headers;
    horizontal_headers << "x" << "y" << "z";
    ui->tableWidget->setHorizontalHeaderLabels(horizontal_headers);

    connect(sensor_ptr_->call_one_times_ptr_, &CallOneTimes::SignalDataShow,
            this, &HandEye::showImage);

    connect(sensor_ptr_->call_one_times_ptr_, &CallOneTimes::scanFinishSignal,
            this,
            [this](QString text)
            {
                ui->textBrowser_log->append(text);
            });

    connect(sensor_ptr_, &Sensor::postBatchNum,
            this,
            [this](int batch_num)
            {
                ui->batch_label->setText(QString::number(batch_num));
            });

    QString path = QCoreApplication::applicationDirPath()+"/config";
    QDir dir(path);
    if (!dir.exists())
        dir.mkdir(path);

    parameters_path_ = QCoreApplication::applicationDirPath()+"/config/handeye.json";
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

    ui->scan_button->setEnabled(false);
    ui->drop_button->setEnabled(false);
    ui->input_button->setEnabled(false);
    ui->calculate_button->setEnabled(false);
    ui->update_button->setEnabled(false);

    utils_ptr_->setValidator(ui->tableWidget);
    utils_ptr_->tableItemInit(vp_input_item_, ui->tableWidget, table_init_num_);

    QIntValidator *validator = new QIntValidator(this);
    QIntValidator *cut_validator = new QIntValidator(0, ceil(batch_size_ * 0.175), this);
    ui->lineEdit->setValidator(validator);
    ui->lineEdit_2->setValidator(validator);
    ui->lineEdit_3->setValidator(validator);
    ui->lineEdit_4->setValidator(validator);
    ui->lineEdit_5->setValidator(validator);
    ui->lineEdit_6->setValidator(validator);
    ui->cut_edit->setValidator(cut_validator);
    int init_size = floor(batch_size_ * 0.175);
    ui->cut_edit->setText(QString::number(init_size));
}

void HandEye::on_connect_button_clicked()
{
    bool ret = sensor_ptr_->ethenetConnect();

    if(ret)
    {
        ui->scan_button->setEnabled(true);
        ui->connect_button->setEnabled(false);
        ui->textBrowser_log->append("连接成功");
    }
    else
    {
        ui->textBrowser_log->append("连接失败");
    }
}

void HandEye::on_scan_button_clicked()
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
        ui->textBrowser_log->append("执行扫图失败");
    else
    {
        ui->textBrowser_log->append("执行扫图成功");
    }

    std::thread thread_batch_display(&Sensor::getBatchNum, sensor_ptr_, batch_size_);
    thread_batch_display.detach();
}

void HandEye::cutQImage(QImage &image)
{
    int cut_vertical_size = ceil(ui->cut_edit->text().toInt() / 0.175);
    QRect mask(0, 0, width_size_, cut_vertical_size);
    image = image.copy(mask);
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
            judge = utils_ptr_->judgeAppearNum(base_table) < cam_vec_size;
        }

        if (judge && (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter))
            utils_ptr_->judgeEnterTable(vp_input_item_, base_table);

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

void HandEye::judgePointsNum()
{
    size_t count = 0;

    if (!base_points_vecs_.empty())
        for (const auto &vec : base_points_vecs_)
            count += vec.size();

    if (count >= 5)
    {
        ui->calculate_button->setEnabled(true);
    }
    else if (count < 5)
    {
        ui->calculate_button->setEnabled(false);
    }
}

void HandEye::disableFunctionButton()
{
    ui->drop_button->setEnabled(false);
    ui->update_button->setEnabled(false);
    ui->input_button->setEnabled(false);
    ui->scan_button->setEnabled(true);
}

void HandEye::enableFunctionButton()
{
    ui->drop_button->setEnabled(true);
    ui->update_button->setEnabled(true);
    ui->input_button->setEnabled(true);
    ui->scan_button->setEnabled(false);
}

void HandEye::showImage(int _width, int _height)
{
    //scale size
//    int mScaleW = ui->label_gray->width();
//    int mScaleH = ui->label_gray->height();
//    int mXscale = int(double(_width) / mScaleW);
//    int mYscale = int(double(_height) / mScaleH);
    int mYscale = scale_;
    int mXscale = scale_;
    int mScaleH = _height / mYscale;
    int mScaleW = _width / mXscale;

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

    cutQImage(gray_image);
    cutQImage(height_image);

    gray_image = gray_image.scaled(gray_image.width() / mXscale_, gray_image.height() / mYscale_);

    auto gray_pixmap = QPixmap::fromImage(gray_image);

    cv::Mat mat = image_proc_ptr_->pixmapToCvMat(gray_pixmap);

    tmp_mat_ = mat.clone();
    tmp_height_ = height_image;

    auto circles = image_proc_ptr_->getCircle(mat, dp_, minDist_, param1_, param2_, minRadius_, maxRadius_);

    if (circles.empty())
    {
        ui->textBrowser_log->append("没有找到点，请重新扫图");
        ui->label_gray->setPixmap(gray_pixmap.scaled(QSize(ui->label_gray->width(), ui->label_gray->height()), Qt::KeepAspectRatio));
        return;
    }

    judgeAndInputBase(mat, circles);

    enableFunctionButton();
}

void HandEye::judgeAndInputBase(cv::Mat &_mat, const std::vector<cv::Vec3d> &_circle)
{
    for (size_t i = 0; i < _circle.size(); i++)
    {
        cv::Point center(cvRound(_circle[i][0]), cvRound(_circle[i][1]));
        int radius = cvRound(_circle[i][2]);
        cv::circle(_mat, center, radius, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        cv::putText(_mat, std::to_string(i + 1), center + cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    }

    auto drawed_pixmap = image_proc_ptr_->matToPixmap(_mat);
    ui->label_gray->setPixmap(drawed_pixmap.scaled(QSize(ui->label_gray->width(), ui->label_gray->height()), Qt::KeepAspectRatio));

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
}

void HandEye::showPointsSum()
{
    size_t points_count = 0;
    for (const auto &vec : cam_points_vecs_)
        points_count += vec.size();

    ui->textBrowser_log->append("当前总点数为:" + QString::number(points_count));
}

void HandEye::on_input_button_clicked()
{
    std::vector<cv::Point3d> base_points_vec;

    bool base_check = utils_ptr_->tableCheck(ui->tableWidget);
    if (!base_check)
    {
        utils_ptr_->showWarnMsg("表格内容不完整，请补充");
        return;
    }

    int row_size = utils_ptr_->judgeAppearNum(ui->tableWidget);

    for (int row = 0; row < row_size; row++)
    {
        int count = row * 3;
        double x = vp_input_item_[count]->text().toDouble();
        double y = vp_input_item_[count + 1]->text().toDouble();
        double z = vp_input_item_[count + 2]->text().toDouble();
        base_points_vec.emplace_back(cv::Point3d(x, y, z));
    }
    size_t vec_size = base_points_vec.size();
    cam_points_vecs_[cam_points_vecs_.size() - static_cast<size_t>(1)].resize(vec_size);
    base_points_vecs_.emplace_back(base_points_vec);

    showPointsSum();
    judgePointsNum();
    disableFunctionButton();
    resetTableWidget();
}

void HandEye::on_calculate_button_clicked()
{
    std::vector<cv::Point3d> cam_points_vec;
    std::vector<cv::Point3d> base_points_vec;

    for (const auto &point_vec : cam_points_vecs_)
        cam_points_vec.insert(cam_points_vec.end(), point_vec.begin(), point_vec.end());

    for (const auto &point_vec : base_points_vecs_)
        base_points_vec.insert(base_points_vec.end(), point_vec.begin(), point_vec.end());

    auto handeye_mat = solver_ptr_->svd(cam_points_vec, base_points_vec);

    verify_ptr_->handeye_mat_ = handeye_mat;
    verify_ptr_->tmp_handeye_mat_ = handeye_mat;
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

void HandEye::on_save_button_clicked()
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

inline void HandEye::setLineEditText(QLineEdit* line_edit, int value)
{
        line_edit->setText(QString::number(value));
}

void HandEye::on_lineEdit_textEdited(const QString &arg1)
{
    bool ret = utils_ptr_->checkLineEdit(arg1, dp_);
    if (!ret)
        setLineEditText(ui->lineEdit, dp_);
}


void HandEye::on_lineEdit_2_textEdited(const QString &arg1)
{
    bool ret = utils_ptr_->checkLineEdit(arg1, minDist_);
    if (!ret)
       setLineEditText(ui->lineEdit_2, minDist_);
}

void HandEye::on_lineEdit_3_textEdited(const QString &arg1)
{
    bool ret = utils_ptr_->checkLineEdit(arg1, param1_);
    if (!ret)
        setLineEditText(ui->lineEdit_3, param1_);
}


void HandEye::on_lineEdit_4_textEdited(const QString &arg1)
{
    bool ret = utils_ptr_->checkLineEdit(arg1, param2_);
    if (!ret)
        setLineEditText(ui->lineEdit_4, param2_);
}


void HandEye::on_lineEdit_5_textEdited(const QString &arg1)
{
    bool ret = utils_ptr_->checkLineEdit(arg1, minRadius_);
    if (!ret)
        setLineEditText(ui->lineEdit_5, minRadius_);
}

void HandEye::on_lineEdit_6_textEdited(const QString &arg1)
{
    bool ret = utils_ptr_->checkLineEdit(arg1, maxRadius_);
    if (!ret)
        setLineEditText(ui->lineEdit_6, maxRadius_);
}

void HandEye::on_update_button_clicked()
{
    cv::Mat mat = tmp_mat_.clone();
    auto circles = image_proc_ptr_->getCircle(mat, dp_, minDist_, param1_, param2_, minRadius_, maxRadius_);
    if (circles.empty())
    {
        ui->textBrowser_log->append("没有找到点，请重新调整参数");
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
        on_save_button_clicked();

    closeReset();
//    sensor_ptr_->ethenetDisconnect();

    event->accept();
}

void HandEye::resetTableWidget()
{
    utils_ptr_->clearTable(vp_input_item_, ui->tableWidget);
}

void HandEye::on_test_button_clicked()
{
    verify_ptr_->setWindowTitle("精度测试");
    verify_ptr_->show();
//    verify_ptr_->max_row_count_ = cam_points_vecs_[cam_points_vecs_.size() - static_cast<size_t>(1)].size();
}

void HandEye::on_drop_button_clicked()
{
    if (!cam_points_vecs_.empty())
        cam_points_vecs_.pop_back();

    showPointsSum();
    judgePointsNum();
    disableFunctionButton();
    resetTableWidget();
    ui->label_gray->clear();
}

void HandEye::resetWidget()
{
    ui->textBrowser_log->clear();
    resetTableWidget();
    verify_ptr_->resetWidget();
    disableFunctionButton();
    ui->calculate_button->setEnabled(false);
}

void HandEye::on_reset_button_clicked()
{
    if (!base_points_vecs_.empty())
    {
        resetWidget();
        base_points_vecs_.clear();
        cam_points_vecs_.clear();
    }
    else if (!cam_points_vecs_.empty())
    {
        resetWidget();
        cam_points_vecs_.clear();
    }
    else
    {
        utils_ptr_->showWarnMsg("您没有进行采样");
    }
}

void HandEye::closeReset()
{
    if (!base_points_vecs_.empty())
    {
        resetWidget();
        base_points_vecs_.clear();
        cam_points_vecs_.clear();
    }
    else if (!cam_points_vecs_.empty())
    {
        resetWidget();
        cam_points_vecs_.clear();
    }
}

HandEye::~HandEye()
{
    delete ui;
    delete sensor_ptr_;
    delete image_proc_ptr_;
    delete verify_ptr_;
    delete utils_ptr_;
    delete solver_ptr_;
}
